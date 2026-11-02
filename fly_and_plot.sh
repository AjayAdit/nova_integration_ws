#!/usr/bin/env bash
# One command: set up the environment, fly once, score it, and open the plot.
#
#   ./fly_and_plot.sh                   3 m/s, UKF, AcroMode, both windows
#   ./fly_and_plot.sh --speed 2         any run_flight.py option can be added
set -e
ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

source "$HOME/anaconda3/etc/profile.d/conda.sh"
conda activate All_Drones
source /opt/ros/humble/setup.bash
export ACADOS_SOURCE_DIR="$HOME/acados"
export LD_LIBRARY_PATH="$HOME/acados/lib:${LD_LIBRARY_PATH:-}"
[ -f install/setup.bash ] || ./setup_ws.sh
source install/setup.bash

python test/run_flight.py flight --viewer "$@"

RUN=$(ls -dt post_analysis/runs/* | head -1)
PLOT="$RUN/plots/trajectory_3d.png"
echo "Plot: $ROOT/$PLOT"
xdg-open "$PLOT" > /dev/null 2>&1 &
