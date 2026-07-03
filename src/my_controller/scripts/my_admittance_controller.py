"""Joint-space admittance outer loop on top of CRISP joint_impedance_controller.

This script does not claim hardware interfaces itself. It uses the already
configured CRISP joint impedance controller as the inner torque controller and
streams a compliant target to /target_joint:

    tau_ext -> virtual joint MSD -> q_adm
    q_target = q_center + q_adm -> joint_impedance_controller

Run after launching the robot with config/controllers.yaml:

    pixi run -e jazzy-crisp python src/my_controller/scripts/my_admittance_controller.py

The inner joint impedance gains are the ones currently loaded in
joint_impedance_controller. This script only computes the admittance displacement
offset and streams the resulting joint target.
"""

import argparse
import threading
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


FR3_Q_LOWER = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159])
FR3_Q_UPPER = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])

CONTROLLER = "joint_impedance_controller"
DEFAULT_TORQUE_TOPIC = "/franka_robot_state_broadcaster/external_joint_torques"
POS_MARGIN = 0.05


@dataclass
class TorqueState:
    value: np.ndarray
    stamp: float | None = None


def parse_vector(values: list[float] | None, default: list[float], name: str) -> np.ndarray:
    """Accept either one scalar for all joints or seven per-joint values."""
    data = default if values is None else values
    if len(data) == 1:
        return np.full(7, float(data[0]))
    if len(data) == 7:
        return np.array(data, dtype=float)
    raise ValueError(f"{name} must have either 1 value or 7 values.")


def apply_deadband(x: np.ndarray, threshold: np.ndarray) -> np.ndarray:
    """Deadband with threshold subtraction so the output is continuous at zero."""
    mag = np.abs(x)
    return np.sign(x) * np.maximum(mag - threshold, 0.0)


def clamp_target(center: np.ndarray, offset: np.ndarray, max_offset: np.ndarray) -> np.ndarray:
    """Clamp the virtual target around center and away from FR3 joint limits."""
    lower = np.maximum(center - max_offset, FR3_Q_LOWER + POS_MARGIN)
    upper = np.minimum(center + max_offset, FR3_Q_UPPER - POS_MARGIN)
    return np.clip(center + offset, lower, upper)


def make_torque_callback(
    torque_state: TorqueState,
    lock: threading.Lock,
    joint_names: list[str],
    alpha: float,
):
    """Create a callback that filters external joint torques into robot joint order."""

    def callback(msg: JointState):
        if len(msg.effort) == 0:
            return

        tau = np.zeros(7)
        if len(msg.name) == 7:
            name_to_effort = dict(zip(msg.name, msg.effort))
            for i, joint_name in enumerate(joint_names):
                tau[i] = float(name_to_effort.get(joint_name, 0.0))
        else:
            n = min(7, len(msg.effort))
            tau[:n] = np.array(msg.effort[:n], dtype=float)

        now = time.monotonic()
        with lock:
            if torque_state.stamp is None:
                torque_state.value = tau
            else:
                torque_state.value = (1.0 - alpha) * torque_state.value + alpha * tau
            torque_state.stamp = now

    return callback


def wait_for_torque(torque_state: TorqueState, lock: threading.Lock, timeout: float):
    start = time.monotonic()
    while True:
        with lock:
            if torque_state.stamp is not None:
                return
        if time.monotonic() - start > timeout:
            raise TimeoutError(
                "No external joint torque messages received. Check "
                f"{DEFAULT_TORQUE_TOPIC} or pass --torque-topic."
            )
        time.sleep(0.02)


def wait_for_joints(robot, timeout: float):
    start = time.monotonic()
    while robot._current_joint is None:
        if time.monotonic() - start > timeout:
            raise TimeoutError("No joint_states received.")
        time.sleep(0.02)


def read_torque(torque_state: TorqueState, lock: threading.Lock) -> np.ndarray:
    with lock:
        return torque_state.value.copy()


def run(args):
    from crisp_py.robot import make_robot

    mass = parse_vector(args.mass, [8.0, 8.0, 8.0, 6.0, 3.0, 2.0, 1.5], "mass")
    damping = parse_vector(args.damping, [14.0, 14.0, 14.0, 10.0, 6.0, 4.0, 3.0], "damping")
    stiffness = parse_vector(args.stiffness, [10.0, 10.0, 10.0, 8.0, 5.0, 3.0, 2.0], "stiffness")
    threshold = parse_vector(
        args.torque_threshold, [0.4, 0.4, 0.4, 0.35, 0.25, 0.2, 0.15], "torque-threshold")
    max_offset = parse_vector(
        args.max_offset, [0.30, 0.30, 0.30, 0.25, 0.22, 0.20, 0.20], "max-offset")
    max_velocity = parse_vector(
        args.max_velocity, [0.35, 0.35, 0.35, 0.35, 0.45, 0.50, 0.60], "max-velocity")

    robot = make_robot("fr3")
    joint_names = robot.config.joint_names

    torque_state = TorqueState(value=np.zeros(7))
    torque_lock = threading.Lock()
    robot.node.create_subscription(
        JointState,
        args.torque_topic,
        make_torque_callback(torque_state, torque_lock, joint_names, args.torque_filter_alpha),
        qos_profile_sensor_data,
    )

    print("Waiting for joint states and external joint torques...")
    wait_for_joints(robot, args.timeout)
    wait_for_torque(torque_state, torque_lock, args.timeout)

    center = robot.joint_values.astype(float)
    robot.set_target_joint(center.copy())

    print(f"Switching to {CONTROLLER}...")
    robot.controller_switcher_client.switch_controller(CONTROLLER)
    state = {c.name: c.state for c in robot.controller_switcher_client.get_controller_list()}
    if state.get(CONTROLLER) != "active":
        robot.shutdown()
        raise SystemExit(f"{CONTROLLER} did not become active (state={state.get(CONTROLLER)}).")

    print(f"Biasing external torque for {args.bias_time:.1f} s. Do not touch the robot...")
    bias_samples = []
    bias_start = time.monotonic()
    while time.monotonic() - bias_start < args.bias_time:
        robot.set_target_joint(center.copy())
        bias_samples.append(read_torque(torque_state, torque_lock))
        time.sleep(1.0 / args.rate)
    tau_bias = np.mean(bias_samples, axis=0) if bias_samples else np.zeros(7)
    print("Torque bias [Nm]:", np.array2string(tau_bias, precision=3, suppress_small=True))

    print("Starting joint admittance. Press Ctrl-C to stop.")
    print("Admittance M:", np.array2string(mass, precision=3, suppress_small=True))
    print("Admittance D:", np.array2string(damping, precision=3, suppress_small=True))
    print("Admittance K:", np.array2string(stiffness, precision=3, suppress_small=True))

    q_offset = np.zeros(7)
    dq_offset = np.zeros(7)
    dt_nominal = 1.0 / args.rate
    last = time.monotonic()
    last_print = last

    try:
        while rclpy.ok():
            now = time.monotonic()
            dt = np.clip(now - last, 0.0, 2.0 * dt_nominal)
            last = now

            tau_raw = read_torque(torque_state, torque_lock)
            tau_ext = args.sign * apply_deadband(tau_raw - tau_bias, threshold)

            qdd = (tau_ext - damping * dq_offset - stiffness * q_offset) / mass
            qdd = np.clip(qdd, -args.max_acceleration, args.max_acceleration)
            dq_offset = np.clip(dq_offset + qdd * dt, -max_velocity, max_velocity)
            q_offset = q_offset + dq_offset * dt

            admittance_target = clamp_target(center, q_offset, max_offset)
            q_offset = admittance_target - center

            target = clamp_target(center, q_offset, max_offset)
            robot.set_target_joint(target.copy())

            if now - last_print > args.print_every:
                last_print = now
                print(
                    "tau_ext_norm={:.2f} Nm  max_offset={:.3f} rad  q7_offset={:.3f} rad".format(
                        float(np.linalg.norm(tau_ext)),
                        float(np.max(np.abs(target - center))),
                        float(target[6] - center[6]),
                    )
                )

            time.sleep(max(0.0, dt_nominal - (time.monotonic() - now)))

    except KeyboardInterrupt:
        print("\nStopping admittance.")
    finally:
        robot.set_target_joint(center.copy())
        time.sleep(0.2)
        robot.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--torque-topic", default=DEFAULT_TORQUE_TOPIC,
                        help="sensor_msgs/JointState topic with external joint torques in effort.")
    parser.add_argument("--rate", type=float, default=100.0,
                        help="Target publishing and admittance integration rate [Hz].")
    parser.add_argument("--bias-time", type=float, default=1.0,
                        help="Seconds used to estimate external torque bias at startup.")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="Timeout while waiting for joint/torque messages.")
    parser.add_argument("--torque-filter-alpha", type=float, default=0.15,
                        help="Low-pass coefficient for external torque, in (0, 1].")
    parser.add_argument("--sign", type=float, default=-1.0,
                        help="Torque sign used by admittance; use 1 if the direction is reversed.")
    parser.add_argument("--mass", type=float, nargs="+",
                        help="Virtual joint inertia. One value or seven values.")
    parser.add_argument("--damping", type=float, nargs="+",
                        help="Virtual admittance damping. One value or seven values.")
    parser.add_argument("--stiffness", type=float, nargs="+",
                        help="Virtual return stiffness around startup pose. One value or seven values.")
    parser.add_argument("--torque-threshold", type=float, nargs="+",
                        help="External torque deadband [Nm]. One value or seven values.")
    parser.add_argument("--max-offset", type=float, nargs="+",
                        help="Maximum target offset from startup pose [rad]. One value or seven values.")
    parser.add_argument("--max-velocity", type=float, nargs="+",
                        help="Maximum virtual target velocity [rad/s]. One value or seven values.")
    parser.add_argument("--max-acceleration", type=float, default=1.0,
                        help="Maximum virtual target acceleration [rad/s^2].")
    parser.add_argument("--print-every", type=float, default=0.5,
                        help="Status print interval [s].")
    args = parser.parse_args()

    if not 0.0 < args.torque_filter_alpha <= 1.0:
        raise ValueError("--torque-filter-alpha must be in (0, 1].")
    run(args)


if __name__ == "__main__":
    main()
