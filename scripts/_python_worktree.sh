#!/usr/bin/env bash
# #157: Python for spawned workers of a worktree run - without site's .pth hooks (-S: no _kiss_slam_editable redirect to the main
# checkout), with this checkout first and the environment's site-packages after it.  Set by scripts/worktree_run.py.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="${WORKTREE_PYTHON:-python}"
SPK="$("$PY" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])' 2>/dev/null)"
PYTHONPATH="$HERE:$SPK${PYTHONPATH:+:$PYTHONPATH}" exec "$PY" -S "$@"
