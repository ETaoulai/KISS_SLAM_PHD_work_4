"""ROS2 bags without type definitions, possibly split into several folders, as a KISS dataset (#104).

kiss_icp's RosbagDataset opens one ROS1 bag (or a folder of ROS1 *.bag) and expects the bag to carry its message definitions.  ROS2
sqlite bags (*.db3 + metadata.yaml, e.g. the Hard Point Cloud Localization Dataset, Zenodo 10122133) carry none, and one sequence can be
split into consecutive bag folders (outdoor_hard_01a, outdoor_hard_01b).  This reader opens all of them as one sequence in time order with
the ROS2 Humble type store (sensor_msgs/PointCloud2 is standard), and reads each message through `self.read_point_cloud`, which
SlamPipeline replaces by kiss_slam's readers as for RosbagDataset (intensity / raw: intensity + ring, here the Livox `line`).
"""
import os
from pathlib import Path

POINTCLOUD2 = "sensor_msgs/msg/PointCloud2"


def ros2_bag_dirs(path: Path):
    """The ROS2 bag folders under `path` (itself, or its subfolders with a metadata.yaml), sorted by name; [] if none."""
    path = Path(path)
    if (path / "metadata.yaml").is_file():
        return [path]
    return sorted(p for p in path.iterdir() if p.is_dir() and (p / "metadata.yaml").is_file()) if path.is_dir() else []


class Ros2Bags:
    def __init__(self, data_dir: Path, topic: str, *_, **__):
        from kiss_icp.tools.point_cloud2 import read_point_cloud
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore

        self.paths = ros2_bag_dirs(data_dir)
        if not self.paths:
            raise ValueError(f"no ROS2 bag (metadata.yaml) in {data_dir}")
        self.sequence_id = os.path.basename(os.path.normpath(data_dir))
        self.bag = AnyReader(self.paths, default_typestore=get_typestore(Stores.ROS2_HUMBLE))
        self.bag.open()
        clouds = [t for t, info in self.bag.topics.items() if info.msgtype == POINTCLOUD2]
        if topic not in clouds:
            raise ValueError(f"no PointCloud2 topic {topic!r}; PointCloud2 topics: {clouds}")
        self.topic = topic
        self.connections = [c for c in self.bag.connections if c.topic == topic]
        self.n_scans = sum(c.msgcount for c in self.connections)
        self.msgs = self.bag.messages(connections=self.connections)
        self.read_point_cloud = read_point_cloud
        self.timestamps = []
        self.use_global_visualizer = True
        if len(self.paths) > 1:
            print("Reading consecutive ROS2 bags: " + ", ".join(p.name for p in self.paths))

    def __del__(self):
        if hasattr(self, "bag"):
            self.bag.close()

    def __len__(self):
        return self.n_scans

    def __getitem__(self, idx):
        connection, timestamp, raw = next(self.msgs)
        self.timestamps.append(timestamp * 1e-9)
        return self.read_point_cloud(self.bag.deserialize(raw, connection.msgtype))

    def reset(self):
        self.timestamps = []
        self.bag.close()
        self.bag.open()
        self.msgs = self.bag.messages(connections=self.connections)

    def get_frames_timestamps(self) -> list:
        return self.timestamps
