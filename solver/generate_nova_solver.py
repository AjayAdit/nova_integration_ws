#!/usr/bin/env python3
"""Generate the C solver for the NOVA airframe and pixel cost.

Uses the Python NMPC in quad_nmpc_ros, with the same settings as NOVA_PAMPC's
nmpc_NOVA.py and the baseline run.

  python generate_nova_solver.py <quad_nmpc_ros_cpp dir> <output dir>
"""
import glob
import os
import shutil
import sys
import tempfile

import numpy as np

pkg, out = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
sys.path.insert(0, os.path.join(pkg, 'scripts'))

from quad_nmpc_ros.nmpc_controller import NMPC  # noqa: E402

FOVY_DEG = 58.0
IMG_H = 480
FY = (IMG_H / 2.0) / np.tan(np.deg2rad(FOVY_DEG) / 2.0)
Q_BC = np.array([-0.27059805, 0.65328148, -0.65328148, 0.27059805])
Q_BC = Q_BC / np.linalg.norm(Q_BC)
T_BC = np.array([0.061, 0.047, -0.065])
CAMERA = {'q_bc': Q_BC, 't_bc': T_BC, 'fx': FY, 'fy': FY}
Q_PIXEL = 250.0
Q_PIXEL_VEL = 0.0

workdir = tempfile.mkdtemp(prefix='nova_solver_')
os.chdir(workdir)
NMPC(N=20, dt=0.05, q_pixel=Q_PIXEL, q_pixel_vel=Q_PIXEL_VEL, camera=CAMERA, export_dir=workdir)

src = os.path.join(workdir, 'c_generated_code')
if os.path.exists(out):
    shutil.rmtree(out)
shutil.copytree(src, out)

# Same cleanup as scripts/regenerate.sh: main() files and the sim solver would
# clash at link time, and compiled objects are machine-specific.
for pattern in ('main_*.c', 'acados_sim_solver_*.c', 'acados_sim_solver_*.h'):
    for path in glob.glob(os.path.join(out, pattern)):
        os.remove(path)
for root, _, files in os.walk(out):
    for name in files:
        if name.endswith(('.o', '.so')):
            os.remove(os.path.join(root, name))
shutil.rmtree(workdir)
print(f'wrote the NOVA solver to {out}')
