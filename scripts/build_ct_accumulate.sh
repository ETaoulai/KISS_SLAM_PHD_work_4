#!/usr/bin/env bash
# Builds the C++ inner loop of the B.9 continuous-time registration (#158, kiss_slam/cpp/ct_accumulate.cpp) into
# kiss_slam/_ct_accumulate<EXT_SUFFIX>.  Run inside the kiss-slam-main environment.  Without it, ct_registration uses the Python version.
set -e
cd "$(dirname "$0")/.."
SUFFIX=$(python -c "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'))")
g++ -O3 -march=native -shared -fPIC -std=c++17 $(python -m pybind11 --includes) kiss_slam/cpp/ct_accumulate.cpp -o kiss_slam/_ct_accumulate$SUFFIX
echo "built kiss_slam/_ct_accumulate$SUFFIX"
