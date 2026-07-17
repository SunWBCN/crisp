#!/usr/bin/env python3
"""Inspect and run pickled pose trajectories through cartesian_admittance_trajectory."""

from __future__ import annotations

import argparse
import csv
import math
import pickle
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np


DEFAULT_CONFIG = (
    "/home/wenbin/github_repo/crisp_franka/src/my_controller/scripts/cartesian_admittance.yaml"
)
DEFAULT_TRAJS = ["trajs/trajectory1.pkl", "trajs/trajectory2.pkl"]
DEFAULT_SAMPLE_PERIOD = 1.0 / 30.0
FLANGE_TO_TCP = np.array([0.0, 0.0, 0.1215], dtype=float)


def describe_streams(path: Path, data: list[dict]) -> None:
    keys = sorted(set().union(*(sample.keys() for sample in data if isinstance(sample, dict))))
    print(f"\n{path}")
    print(f"  streams: {keys}")
    for key in keys:
        try:
            values = np.asarray([sample[key] for sample in data], dtype=float)
        except Exception:
            continue
        print(f"  {key}: shape {values.shape}")
    print("  pose_data layout: [x, y, z, qx, qy, qz, qw]")
    if "box_pos" in keys or "box_rot" in keys:
        print("  object pose streams: box_pos / box_rot are inspected but not commanded")


def load_pose_data(path: Path) -> np.ndarray:
    with path.open("rb") as f:
        data = pickle.load(f)
    if not isinstance(data, list) or not data:
        raise ValueError(f"{path}: expected a non-empty list of samples")
    describe_streams(path, data)
    try:
        pose = np.asarray([sample["pose_data"] for sample in data], dtype=float)
    except Exception as exc:
        raise ValueError(f"{path}: every sample must contain numeric pose_data") from exc
    if pose.ndim != 2 or pose.shape[1] != 7:
        raise ValueError(f"{path}: pose_data must have shape (N, 7), got {pose.shape}")
    if not np.isfinite(pose).all():
        raise ValueError(f"{path}: pose_data contains NaN or infinity")

    q_norm = np.linalg.norm(pose[:, 3:7], axis=1)
    if np.any(q_norm < 1.0e-9):
        raise ValueError(f"{path}: pose_data contains a zero quaternion")
    pose[:, 3:7] /= q_norm[:, None]
    for i in range(1, len(pose)):
        if float(np.dot(pose[i - 1, 3:7], pose[i, 3:7])) < 0.0:
            pose[i, 3:7] *= -1.0
    return pose


def load_initial_joints(path: Path) -> np.ndarray:
    with path.open("rb") as f:
        data = pickle.load(f)
    try:
        joints = np.asarray(data[0]["franka_dof_pos"], dtype=float)
    except Exception as exc:
        raise ValueError(f"{path}: first sample must contain numeric franka_dof_pos") from exc
    if joints.shape != (7,) or not np.isfinite(joints).all():
        raise ValueError(f"{path}: first franka_dof_pos must contain 7 finite values")
    return joints


def quaternion_step_angles(quat_xyzw: np.ndarray) -> np.ndarray:
    dots = np.sum(quat_xyzw[:-1] * quat_xyzw[1:], axis=1)
    dots = np.clip(np.abs(dots), -1.0, 1.0)
    return 2.0 * np.arccos(dots)


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        dtype=float,
    )


def quat_normalize(q: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(q))
    if norm < 1.0e-9:
        raise ValueError("zero quaternion")
    return q / norm


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    qv = np.array([v[0], v[1], v[2], 0.0], dtype=float)
    return quat_multiply(quat_multiply(q, qv), quat_conjugate(q))[:3]


def quat_slerp(q0: np.ndarray, q1: np.ndarray, u: float) -> np.ndarray:
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return quat_normalize(q0 + u * (q1 - q0))
    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * u
    return (
        math.sin(theta_0 - theta) / sin_theta_0 * q0
        + math.sin(theta) / sin_theta_0 * q1
    )


def minimum_jerk01(u: float) -> float:
    s = min(1.0, max(0.0, u))
    return s * s * s * (10.0 + s * (-15.0 + 6.0 * s))


def rebase_pose_position_to_anchor(pose: np.ndarray, anchor: np.ndarray) -> np.ndarray:
    rebased = pose.copy()
    rebased[:, :3] = anchor[:3] + (pose[:, :3] - pose[0, :3])
    return rebased


def convert_pose_frame(pose: np.ndarray, pose_frame: str) -> np.ndarray:
    if pose_frame == "tcp":
        return pose.copy()
    if pose_frame != "flange":
        raise ValueError(f"unknown pose frame {pose_frame!r}")
    converted = pose.copy()
    for i, q in enumerate(converted[:, 3:7]):
        converted[i, :3] = converted[i, :3] + quat_rotate(q, FLANGE_TO_TCP)
    return converted


def make_return_segment(start: np.ndarray, goal: np.ndarray, sample_period: float, return_time: float) -> np.ndarray:
    if return_time <= 0.0:
        return goal.reshape(1, 7)
    steps = max(2, int(math.ceil(return_time / sample_period)) + 1)
    segment = np.zeros((steps, 7), dtype=float)
    for i in range(steps):
        u = minimum_jerk01(i / (steps - 1))
        segment[i, :3] = start[:3] + u * (goal[:3] - start[:3])
        segment[i, 3:7] = quat_slerp(start[3:7], goal[3:7], u)
    return segment


def combine_relative_trajectories(
    poses: list[np.ndarray], sample_period: float, return_time: float
) -> np.ndarray:
    if not poses:
        raise ValueError("no trajectories to combine")
    anchor = poses[0][0]
    chunks = [poses[0]]
    current = poses[0][-1]
    for pose in poses[1:]:
        next_start_at_anchor = rebase_pose_position_to_anchor(pose, anchor)[0]
        reset = make_return_segment(current, next_start_at_anchor, sample_period, return_time)
        chunks.append(reset[1:])
        rebased = rebase_pose_position_to_anchor(pose, anchor)
        chunks.append(rebased[1:])
        current = rebased[-1]
    return np.vstack(chunks)


def inspect_pose(
    path: Path,
    raw_pose: np.ndarray,
    command_pose: np.ndarray,
    sample_period: float,
    max_linear_speed: float,
    max_angular_speed: float,
    pose_frame: str,
) -> None:
    pose = command_pose
    pos = pose[:, :3]
    quat = pose[:, 3:7]
    dpos = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    dtheta = quaternion_step_angles(quat)
    duration = sample_period * (len(pose) - 1)
    max_v = float(dpos.max() / sample_period)
    mean_v = float(dpos.mean() / sample_period)
    max_omega = float(dtheta.max() / sample_period)

    print(f"  recorded pose_data frame: {pose_frame}")
    print(f"  samples: {len(pose)}, playback duration: {duration:.3f} s")
    print(f"  recorded pose_data xyz start: {raw_pose[0, :3]}, end: {raw_pose[-1, :3]}")
    if pose_frame == "flange":
        print(f"  controller TCP xyz start:    {pos[0]}, end: {pos[-1]}")
    else:
        print(f"  controller TCP xyz start: {pos[0]}, end: {pos[-1]}")
    print(f"  controller TCP xyz min:   {pos.min(axis=0)}, max: {pos.max(axis=0)}")
    print(f"  path length: {dpos.sum():.4f} m, start-to-end: {np.linalg.norm(pos[-1] - pos[0]):.4f} m")
    print(f"  max target speed at dt={sample_period:.3f}s: {max_v:.3f} m/s")
    print(f"  mean target speed at dt={sample_period:.3f}s: {mean_v:.3f} m/s")
    print(f"  max target angular speed: {max_omega:.3f} rad/s")
    if max_v > max_linear_speed:
        print(f"  warning: max target speed is above --max-linear-speed={max_linear_speed:.3f} m/s")
    if max_omega > max_angular_speed:
        print(f"  warning: max target angular speed is above --max-angular-speed={max_angular_speed:.3f} rad/s")
    if pos[:, 2].min() < 0.25:
        print("  warning: low TCP z value; check table/workpiece clearance before absolute playback")


def print_speed_summary(label: str, pose: np.ndarray, sample_period: float) -> None:
    dpos = np.linalg.norm(np.diff(pose[:, :3], axis=0), axis=1)
    dtheta = quaternion_step_angles(pose[:, 3:7])
    print(
        f"  {label} max target speed: {float(dpos.max() / sample_period):.3f} m/s, "
        f"max angular speed: {float(dtheta.max() / sample_period):.3f} rad/s"
    )


def write_csv(path: Path, pose: np.ndarray, sample_period: float) -> None:
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw"])
        for i, row in enumerate(pose):
            writer.writerow([f"{i * sample_period:.9f}", *[f"{v:.12g}" for v in row]])


def build_command(args: argparse.Namespace, csv_path: Path, initial_joints: np.ndarray | None = None) -> list[str]:
    cmd = [
        "ros2",
        "run",
        "my_controller",
        "cartesian_admittance_trajectory",
        args.robot_hostname,
        "--config",
        args.config,
        "--profile",
        args.profile,
        "--trajectory-csv",
        str(csv_path),
        "--hold-time",
        str(args.hold_time),
        "--prealign-time",
        str(args.prealign_time),
        "--final-return-time",
        str(args.return_time),
        "--post-hold-time",
        str(args.post_hold_time),
        "--max-linear-speed",
        str(args.max_linear_speed),
        "--max-angular-speed",
        str(args.max_angular_speed),
        "--auto-start",
    ]
    cmd.append("--absolute" if args.absolute else "--relative")
    if initial_joints is not None:
        cmd += [
            "--initial-joints",
            ",".join(f"{value:.12g}" for value in initial_joints),
            "--joint-prepose-time",
            str(args.joint_prepose_time),
        ]
    if args.plot_filter_comparison is not None:
        cmd += ["--plot-filter-comparison", args.plot_filter_comparison]
    return cmd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Inspect and run trajectory*.pkl through the tuned Cartesian admittance controller."
    )
    parser.add_argument("robot_hostname", nargs="?", default="10.90.90.10")
    parser.add_argument("--trajs", nargs="+", default=DEFAULT_TRAJS)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--profile", default="soft")
    parser.add_argument(
        "--pose-frame",
        choices=["flange", "tcp"],
        default="flange",
        help="Frame stored in pose_data. Default flange: convert pose_data with the 0.1215 m tool offset before commanding the TCP controller.",
    )
    parser.add_argument(
        "--sample-period",
        type=float,
        default=DEFAULT_SAMPLE_PERIOD,
        help="Seconds between pickle samples when writing CSV. Default is 1/30 s for faithful 30 Hz playback.",
    )
    parser.add_argument(
        "--max-linear-speed",
        type=float,
        default=0.50,
        help="Override the controller max_linear_speed while replaying. Default 0.50 m/s avoids clipping the 30 Hz target velocity in the current files.",
    )
    parser.add_argument(
        "--max-angular-speed",
        type=float,
        default=1.0,
        help="Override the controller max_angular_speed while replaying.",
    )
    parser.add_argument("--hold-time", type=float, default=1.0)
    parser.add_argument(
        "--prealign-time",
        type=float,
        default=3.0,
        help="Seconds to smoothly align from measured reset orientation to the first recorded quaternion before playback.",
    )
    parser.add_argument(
        "--return-time",
        type=float,
        default=5.0,
        help="Seconds for each smooth return to the measured reset pose. 0 means hold the final trajectory pose.",
    )
    parser.add_argument(
        "--post-hold-time",
        type=float,
        default=3.0,
        help="Seconds to hold the final target before commanding MotionFinished.",
    )
    parser.add_argument("--absolute", action="store_true", help="Use raw world/base-frame poses.")
    parser.add_argument(
        "--move-to-initial-joints",
        action="store_true",
        help="Before each Cartesian replay, move slowly to that pickle's first franka_dof_pos using libfranka joint control.",
    )
    parser.add_argument(
        "--joint-prepose-time",
        type=float,
        default=8.0,
        help="Seconds for --move-to-initial-joints.",
    )
    parser.add_argument(
        "--combined",
        action="store_true",
        help="Run both pickles as one CSV with an internal reset return. Default is separate runs so traj1 returns to reset before traj2 starts.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Inspect and print commands without running the robot.")
    parser.add_argument("--yes", action="store_true", help="Do not prompt before launching each controller run.")
    parser.add_argument("--plot-filter-comparison", choices=["true", "false"])
    return parser.parse_args()


def confirm_start(label: str, args: argparse.Namespace) -> bool:
    if args.yes:
        return True
    if not sys.stdin.isatty():
        print(f"No interactive terminal detected; not starting {label}. Use --yes to run without prompts.", file=sys.stderr)
        return False
    try:
        input(f"\nPress Enter to start {label} (Ctrl+C to abort)...")
    except (EOFError, KeyboardInterrupt):
        print(f"\nAborted before {label}.", file=sys.stderr)
        return False
    return True


def main() -> int:
    args = parse_args()
    if args.sample_period <= 0.0 or not math.isfinite(args.sample_period):
        print("--sample-period must be positive and finite", file=sys.stderr)
        return 1
    if args.max_linear_speed <= 0.0 or not math.isfinite(args.max_linear_speed):
        print("--max-linear-speed must be positive and finite", file=sys.stderr)
        return 1
    if args.max_angular_speed <= 0.0 or not math.isfinite(args.max_angular_speed):
        print("--max-angular-speed must be positive and finite", file=sys.stderr)
        return 1
    if args.hold_time < 0.0 or not math.isfinite(args.hold_time):
        print("--hold-time must be non-negative and finite", file=sys.stderr)
        return 1
    if args.prealign_time < 0.0 or not math.isfinite(args.prealign_time):
        print("--prealign-time must be non-negative and finite", file=sys.stderr)
        return 1
    if args.return_time < 0.0 or not math.isfinite(args.return_time):
        print("--return-time must be non-negative and finite", file=sys.stderr)
        return 1
    if args.post_hold_time < 0.0 or not math.isfinite(args.post_hold_time):
        print("--post-hold-time must be non-negative and finite", file=sys.stderr)
        return 1
    if args.joint_prepose_time <= 0.0 or not math.isfinite(args.joint_prepose_time):
        print("--joint-prepose-time must be positive and finite", file=sys.stderr)
        return 1

    traj_paths = [Path(p) for p in args.trajs]
    poses = []
    initial_joints = []
    print("Inspecting trajectories before execution:")
    print(
        f"Faithful timing: sample period {args.sample_period:.6f} s "
        f"({1.0 / args.sample_period:.3f} Hz)"
    )
    print(
        f"Controller speed overrides: max linear {args.max_linear_speed:.3f} m/s, "
        f"max angular {args.max_angular_speed:.3f} rad/s"
    )
    print(f"Post-trajectory hold before finish: {args.post_hold_time:.3f} s")
    if args.move_to_initial_joints:
        print(
            f"Joint prepose experiment: enabled, duration {args.joint_prepose_time:.3f} s "
            "to each trajectory's first franka_dof_pos"
        )
    for path in traj_paths:
        raw_pose = load_pose_data(path)
        initial_joints.append(load_initial_joints(path))
        command_pose = convert_pose_frame(raw_pose, args.pose_frame)
        inspect_pose(
            path,
            raw_pose,
            command_pose,
            args.sample_period,
            args.max_linear_speed,
            args.max_angular_speed,
            args.pose_frame,
        )
        poses.append(command_pose)

    if args.absolute:
        print("\nPlayback mode: absolute world/base-frame controller TCP targets.")
    elif not args.combined:
        end_behavior = (
            "holds the final trajectory pose"
            if args.return_time == 0.0
            else "returns to the measured reset pose"
        )
        print(
            "\nPlayback mode: separate relative-position / faithful-rotation runs. "
            "Each run first pre-aligns to the recorded orientation, then replays the path "
            f"at the recorded frequency, then {end_behavior}."
        )
    else:
        print(
            "\nPlayback mode: combined relative-position / faithful-rotation replay. "
            "Traj1 starts at the measured reset pose, "
            "then a smooth return goes back to reset before traj2 starts."
        )

    with tempfile.TemporaryDirectory(prefix="crisp_pickled_trajs_") as tmp:
        tmpdir = Path(tmp)
        if not args.absolute and args.combined:
            combined = combine_relative_trajectories(poses, args.sample_period, args.return_time)
            csv_path = tmpdir / "combined_reset_replay.csv"
            write_csv(csv_path, combined, args.sample_period)
            print(
                f"\nCombined command: {len(combined)} samples, "
                f"{args.sample_period * (len(combined) - 1):.3f} s trajectory time, "
                f"including {args.return_time:.3f} s internal reset return before trajectory2"
            )
            print_speed_summary("combined replay", combined, args.sample_period)
            cmd = build_command(args, csv_path)
            print("  " + " ".join(cmd))
            if args.dry_run:
                return 0
            if not confirm_start("combined trajectory replay", args):
                return 1
            result = subprocess.run(cmd)
            print(f"Combined trajectory replay exited with code {result.returncode}.")
            return result.returncode

        for index, (src, pose) in enumerate(zip(traj_paths, poses), start=1):
            csv_path = tmpdir / f"{src.stem}.csv"
            write_csv(csv_path, pose, args.sample_period)
            joints = initial_joints[index - 1] if args.move_to_initial_joints else None
            cmd = build_command(args, csv_path, joints)
            print(f"\nTrajectory {index}/{len(poses)} command:")
            print("  " + " ".join(cmd))
            if args.dry_run:
                continue
            if not confirm_start(f"trajectory {index}/{len(poses)} ({src})", args):
                return 1
            result = subprocess.run(cmd)
            print(f"Trajectory {index}/{len(poses)} exited with code {result.returncode}.")
            if result.returncode != 0:
                return result.returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
