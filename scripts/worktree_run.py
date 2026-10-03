#!/usr/bin/env python3
"""Run a script of THIS checkout (a git worktree) instead of the editable install (#115).

    python scripts/worktree_run.py scripts/run_ncd.py <args...>

The editable install of kiss_slam puts an import hook in front of sys.path that always loads the package from the main checkout
(/home/photogrammetry/Kiss_SLAM-main).  This removes that hook and puts this checkout first, so a worktree on another branch can run its
own code while batches keep running from the main checkout.  The compiled modules (kiss_slam_pybind, _guided_match) must be copied into
this checkout's kiss_slam/ (gitignored).  Not for --parallel (the spawned worker would import the main checkout again).
"""
import runpy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.meta_path[:] = [f for f in sys.meta_path if "editable" not in (type(f).__module__ + type(f).__name__).lower()]
sys.path.insert(0, str(ROOT))
script = sys.argv[1]
sys.argv = sys.argv[1:]
runpy.run_path(script, run_name="__main__")
