#!/usr/bin/env python3
"""
run_3d_nav.py
Command-line runner to trigger the 3D AGV Simscape Multibody navigation simulation via Python.

Usage:
    python run_3d_nav.py
    python run_3d_nav.py --goal 22.0 22.0
    python run_3d_nav.py --goal 12.5 8.0
"""

import argparse
import sys
from simulink_connector import SimulinkBridge


def main():
    parser = argparse.ArgumentParser(description="Run 3D Simscape Navigation for Custom AGV")
    parser.add_argument("--goal", nargs=2, type=float, default=[22.0, 22.0],
                        metavar=("X", "Y"), help="Goal target coordinates (default: 22.0 22.0)")
    args = parser.parse_args()

    goal_x, goal_y = args.goal
    print(f"=======================================================")
    print(f" AGV 3D Simscape Navigation Launcher (Python API)")
    print(f" Target Destination: [{goal_x:.1f}, {goal_y:.1f}]")
    print(f"=======================================================")

    bridge = SimulinkBridge()
    try:
        bridge.set_navigation_target(goal_x, goal_y)
        bridge.open_model('agv_3d_nav')
        bridge.run_simulation('agv_3d_nav')
        print(f"AGV arrived at target [{goal_x:.1f}, {goal_y:.1f}] in Mechanics Explorer!")
    finally:
        bridge.disconnect()


if __name__ == '__main__':
    main()
