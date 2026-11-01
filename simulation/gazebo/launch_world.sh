#!/usr/bin/env bash
# Launch a facade world with this project's model path configured (no prior shell setup needed).
#
# Usage (any working directory):
#     simulation/gazebo/launch_world.sh facade_flat                 # server + GUI
#     simulation/gazebo/launch_world.sh facade_complex --headless   # server only, running (gz sim -s -r)
#     simulation/gazebo/launch_world.sh facade_flat -- -v 4         # extra arguments after -- go to gz sim
#
# The aircraft in the world is STATIC: Gazebo does not simulate its flight. Use
# simulation/gazebo/replay_run.py to replay a Python-simulator log (docs/gazebo_milestone3.md).
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "$here/env.sh"

usage() { sed -n '4,7p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' >&2; exit 2; }
[ $# -ge 1 ] || usage
world="$1"; shift
sdf="$(cd "$here/../worlds" && pwd)/${world}.sdf"
[ -f "$sdf" ] || { echo "unknown world '$world' (no $sdf)" >&2; exit 2; }

mode=()
if [ "${1:-}" = "--headless" ]; then mode=(-s -r); shift; fi
if [ "${1:-}" = "--" ]; then shift; fi

exec gz sim "${mode[@]}" "$@" "$sdf"
