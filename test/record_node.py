#!/usr/bin/env python3
"""Started by nova.launch.py: records the flight, and saves one post-analysis run when it stops.

Saves on a normal end and on Ctrl+C. An interrupted flight is marked as not finished. If no
flight data arrived, it saves nothing.
"""
import json
import os
import signal
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402
from rclpy.signals import SignalHandlerOptions  # noqa: E402

from run_flight import PA, ROOT, Recorder, verdict, write_results  # noqa: E402

FINISH_TOLERANCE_S = 0.1


def main():
    stop = []
    # Ctrl+C reaches this process twice, from the terminal and from launch. Keep saving.
    signal.signal(signal.SIGINT, lambda *_: stop.append(True))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(True))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('flight_recorder')
    duration = node.declare_parameter('duration', 0.0).value
    name = node.declare_parameter('name', 'launch').value
    rec = Recorder(node)
    while not stop:
        rclpy.spin_once(node, timeout_sec=0.01)

    if not rec.commands or not rec.truth:
        print('recorder: no flight data, nothing saved', flush=True)
        return
    out = subprocess.run([sys.executable, PA, 'new', name, '--changed', f'{name} (ros2 launch)'],
                         check=True, capture_output=True, text=True, cwd=ROOT).stdout
    run_dir = os.path.join(ROOT, out.split('Created ')[1].split('\n')[0].strip())
    summary = write_results(rec, os.path.join(run_dir, 'raw'))
    finished = duration > 0.0 and rec.sim_ns * 1e-9 >= duration - FINISH_TOLERANCE_S
    verdict(summary, finished)
    with open(os.path.join(run_dir, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=2)
    print(f'recorder: saved {os.path.basename(run_dir)}, '
          f'mean 3D error {summary["mean_3d_error_cm"]:.2f} cm, '
          f'{summary["lost_frames"]} lost, finished: {finished}', flush=True)


if __name__ == '__main__':
    main()
