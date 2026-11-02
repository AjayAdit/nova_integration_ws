#!/usr/bin/env bash
# Clones the commits in repos.yaml into src/, generates the NOVA solver and builds.
# Run with the conda env that owns rclpy and mujoco active, and ROS 2 sourced.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"
: "${ACADOS_SOURCE_DIR:?set ACADOS_SOURCE_DIR}"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"

mkdir -p src
python - <<'PY'
import os, subprocess, yaml
for name, repo in yaml.safe_load(open('repos.yaml')).items():
    path = os.path.join('src', name)
    if not os.path.isdir(path):
        subprocess.run(['git', 'clone', '-q', repo['url'], path], check=True)
    subprocess.run(['git', '-C', path, 'fetch', '-q', 'origin'], check=True)
    subprocess.run(['git', '-C', path, 'checkout', '-q', repo['commit']], check=True)
    if subprocess.run(['git', '-C', path, 'status', '--porcelain'], capture_output=True, text=True).stdout:
        raise SystemExit(f'{path} has local changes: fixes belong in the owning repo')
    print(name, repo['commit'])
PY

python solver/generate_nova_solver.py src/quad_nmpc_ros/quad_nmpc_ros_cpp solver/acados_generated_nova
MUJOCO_ROOT_DIR=$(python -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))")
python -m colcon build --symlink-install \
  --cmake-args -DMUJOCO_ROOT_DIR="$MUJOCO_ROOT_DIR" -DCMAKE_BUILD_TYPE=Release \
  -DQUAD_NMPC_SOLVER_DIR="$ROOT/solver/acados_generated_nova"
