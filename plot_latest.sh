#!/usr/bin/env bash
# Plot a recorded flight and open one image: 3D error, and expected, actual and car in 3D.
#
#   ./plot_latest.sh           the latest flight
#   ./plot_latest.sh run44     an older flight
set -e
ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"
source "$HOME/anaconda3/etc/profile.d/conda.sh"
conda activate All_Drones

if [ -n "$1" ]; then
  RUN=$(ls -d post_analysis/runs/*_"$1"_* | head -1)
else
  RUN=$(ls -dt post_analysis/runs/*/ | head -1)
fi
RUN="${RUN%/}"
python post_analysis/scripts/pa.py analyze "$(basename "$RUN")" > /dev/null
python post_analysis/scripts/plot_flight.py "$RUN"
echo "Plot: $ROOT/$RUN/plots/trajectory_3d.png"
xdg-open "$RUN/plots/trajectory_3d.png" > /dev/null 2>&1 &
