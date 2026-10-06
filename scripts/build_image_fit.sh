#!/usr/bin/env bash
# Builds the C++ parts of the image motion (#182: RANSAC hypotheses, soft_l1 time fit, panorama scatter; kiss_slam/cpp/image_fit.cpp)
# into kiss_slam/_image_fit<EXT_SUFFIX>.  Run inside the kiss-slam-main environment.  Without it, kiss_slam.intensity_deskew uses Python.
set -e
cd "$(dirname "$0")/.."
SUFFIX=$(python -c "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'))")
g++ -O3 -march=native -shared -fPIC -std=c++17 -I/usr/include/eigen3 $(python -m pybind11 --includes) kiss_slam/cpp/image_fit.cpp -o kiss_slam/_image_fit$SUFFIX
echo "built kiss_slam/_image_fit$SUFFIX"
