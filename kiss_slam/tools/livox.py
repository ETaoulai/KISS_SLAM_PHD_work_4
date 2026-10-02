"""Livox solid-state LiDARs in a ROS1 bag (`livox_ros_driver/CustomMsg`), as a KISS dataset (#098, open_tasks B.10).

kiss_icp's RosbagDataset only accepts sensor_msgs/PointCloud2 topics.  This reader takes a CustomMsg topic and parses the message bytes
directly (rosbags would build one Python object per point - minutes per sequence, #095).  CustomMsg (ROS1 serialisation):
    header (seq u32, stamp sec u32 + nsec u32, frame_id string), timebase u64, point_num u32, lidar_id u8, rsvd u8[3],
    points[] (u32 length, then per point: offset_time u32 ns from timebase, x y z f32, reflectivity u8, tag u8, line u8 - 19 bytes).
Point time = header stamp + offset_time.  Clock: when the header is device time (seconds since power-on, < 1e6 s - TIERS), every time is
mapped to ROS time with one constant, bag time - header of the FIRST message, so evaluation against ROS-time ground truth works (the
remaining transport delay is constant; the evaluation's own time handling applies).  Zero points (no return) are dropped.

Returns what the installed reader of the pipeline asks for (SlamPipeline swaps `read_point_cloud`): upstream (xyz, t); kiss_slam's
intensity reader (xyz, t, reflectivity / 255); the raw reader (xyz, t, reflectivity 0-255, line).  `line` is the Livox laser index
(Avia 6, Horizon 6, Mid-360 4), NOT an image row: the ring x azimuth panorama of the image motion does not apply to these sensors.
"""
import os
import struct
from pathlib import Path

import numpy as np

CUSTOM_MSG = "livox_ros_driver/msg/CustomMsg"
POINT = np.dtype([("offset_time", "<u4"), ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                  ("reflectivity", "u1"), ("tag", "u1"), ("line", "u1")])          # 19 bytes, packed


def parse_custom_msg(raw: bytes):
    """(header stamp s, points structured array) from the ROS1 bytes of one CustomMsg."""
    seq, sec, nsec, flen = struct.unpack_from("<IIII", raw, 0)
    o = 16 + flen
    timebase, point_num = struct.unpack_from("<QI", raw, o)
    o += 8 + 4 + 1 + 3                                   # timebase, point_num, lidar_id, rsvd[3]
    (n,) = struct.unpack_from("<I", raw, o)
    o += 4
    if n * POINT.itemsize != len(raw) - o:
        raise ValueError(f"CustomMsg layout: {n} points x {POINT.itemsize} B != {len(raw) - o} B left")
    return sec + nsec * 1e-9, np.frombuffer(raw, dtype=POINT, count=n, offset=o)


class LivoxRosbag:
    def __init__(self, data_dir: Path, topic: str, *_, **__):
        from rosbags.highlevel import AnyReader

        data_dir = Path(data_dir)
        paths = [data_dir] if data_dir.is_file() else sorted(data_dir.glob("*.bag"))
        self.sequence_id = os.path.basename(paths[0]).split(".")[0]
        self.bag = AnyReader(paths)
        self.bag.open()
        livox = [t for t, info in self.bag.topics.items() if info.msgtype == CUSTOM_MSG]
        if topic not in livox:
            raise ValueError(f"no {CUSTOM_MSG} topic {topic!r} in the bag; Livox topics: {livox}")
        self.topic = topic
        self.n_scans = self.bag.topics[topic].msgcount
        self.connections = [c for c in self.bag.connections if c.topic == topic]
        self.msgs = self.bag.messages(connections=self.connections)
        self.timestamps = []
        self.clock_offset = None
        self.read_point_cloud = None                     # SlamPipeline may install kiss_slam's readers; only their identity is used
        self.use_global_visualizer = True

    @staticmethod
    def is_livox(bag_path: Path, topic: str) -> bool:
        from rosbags.highlevel import AnyReader

        bag_path = Path(bag_path)
        paths = [bag_path] if bag_path.is_file() else sorted(bag_path.glob("*.bag"))
        with AnyReader(paths) as r:
            info = r.topics.get(topic)
            return info is not None and info.msgtype == CUSTOM_MSG

    def __del__(self):
        if hasattr(self, "bag"):
            self.bag.close()

    def __len__(self):
        return self.n_scans

    def __getitem__(self, idx):
        connection, timestamp, raw = next(self.msgs)
        bag_t = timestamp * 1e-9
        stamp, p = parse_custom_msg(raw)
        if self.clock_offset is None:
            self.clock_offset = bag_t - stamp if stamp < 1e6 else 0.0      # device time -> ROS time, one constant for the run
        stamp += self.clock_offset
        self.timestamps.append(stamp)
        keep = (p["x"] != 0) | (p["y"] != 0) | (p["z"] != 0)
        p = p[keep]
        xyz = np.column_stack([p["x"], p["y"], p["z"]]).astype(np.float64)
        t = stamp + p["offset_time"].astype(np.float64) * 1e-9
        name = getattr(self.read_point_cloud, "__name__", "")
        mod = getattr(self.read_point_cloud, "__module__", "")
        if mod.startswith("kiss_slam") and name == "read_point_cloud_raw":
            return xyz, t, p["reflectivity"].astype(np.float64), p["line"].astype(np.int64)
        if mod.startswith("kiss_slam") and name == "read_point_cloud":
            return xyz, t, p["reflectivity"].astype(np.float32) / 255.0
        return xyz, t

    def reset(self):
        self.timestamps = []
        self.bag.close()
        self.bag.open()
        self.msgs = self.bag.messages(connections=self.connections)

    def get_frames_timestamps(self) -> list:
        return self.timestamps
