"""Read and print the current end-effector pose with crisp_py."""

import argparse
import time

import numpy as np

from crisp_py.robot import make_robot


def pose_to_matrix(pose):
    transform = np.eye(4)
    transform[:3, :3] = pose.orientation.as_matrix()
    transform[:3, 3] = pose.position
    return transform


def print_pose(pose):
    position = pose.position
    quaternion_xyzw = pose.orientation.as_quat()
    transform = pose_to_matrix(pose)

    print("position [x, y, z]:")
    print(np.array2string(position, precision=6, suppress_small=True))
    print()

    print("quaternion [x, y, z, w]:")
    print(np.array2string(quaternion_xyzw, precision=6, suppress_small=True))
    print()

    print("pose matrix:")
    print(np.array2string(transform, precision=6, suppress_small=True))
    print()

    print("C++ PoseMatrix:")
    print("const my_controller::PoseMatrix target_pose_matrix = {{")
    for row in transform:
        print("  {{" + ", ".join(f"{value:.9f}" for value in row) + "}},")
    print("}};")
    print()


def main():
    parser = argparse.ArgumentParser(description="Read the current CRISP end-effector pose.")
    parser.add_argument("--robot", default="fr3", help="CRISP robot config name.")
    parser.add_argument("--rate", type=float, default=1.0, help="Print rate in Hz.")
    parser.add_argument("--once", action="store_true", help="Print once and exit.")
    args = parser.parse_args()

    np.set_printoptions(precision=6, suppress=True)

    robot = make_robot(args.robot)
    try:
        robot.wait_until_ready()

        while True:
            print_pose(robot.end_effector_pose)
            if args.once:
                break
            time.sleep(1.0 / args.rate)
    finally:
        robot.shutdown()


if __name__ == "__main__":
    main()
