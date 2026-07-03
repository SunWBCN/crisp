"""Tune the crisp joint-space PD (joint_impedance_controller) on the FR3.

Uses the PD controller already implemented in crisp_controllers
(crisp_controllers/CartesianController). With the task gains zeroed and
nullspace.projector_type: none (see config/controllers.yaml :: joint_impedance_controller),
its control law per joint is

    tau_j = stiffness * weight_j * (q_ref_j - q_j) + damping * weight_j * (dq_ref_j - dq_j)

so the tunable PD knobs are:
    nullspace.stiffness                 -- global P gain
    nullspace.damping                   -- global D gain (-1 => auto 2*sqrt(stiffness))
    nullspace.weights.fr3_jointN.value  -- per-joint scale (effective P = stiffness*weight)

The controller is commanded over /target_joint (robot.set_target_joint). This script
streams a gentle sine on ONE joint (others held), centered on the CURRENT pose (no
homing move), plots commanded-vs-actual live, and lets you tune the gains per run.

Examples
--------
Offline sanity check of the planned trajectory + plot (no robot needed):
    python src/my_controller/scripts/tune_joint_pd.py --joint 6 --preview

On hardware (after `pixi run -e jazzy-crisp franka robot_ip:=<ip>`):
    python src/my_controller/scripts/tune_joint_pd.py --joint 6
    python src/my_controller/scripts/tune_joint_pd.py --joint 6 --stiffness 20 --damping -1 --weight 30
"""

import argparse
import time
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# --- FR3 limits (Franka Research 3 datasheet; conservative, hardcoded) --------
FR3_VMAX = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])  # [rad/s]
FR3_Q_LOWER = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159])
FR3_Q_UPPER = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])

CONTROLLER = "joint_impedance_controller"

SAMPLE_RATE = 100.0  # [Hz] setpoint streaming + recording loop
PLOT_EVERY = 5       # refresh the live plot every N samples
AMP_CEILING = 0.5    # [rad] hard cap on sine amplitude, keeps motion modest
POS_MARGIN = 0.05    # [rad] keep this far from position limits
SETTLE_TIME = 1.5    # [s] keep recording after the motion ends

RESULTS_DIR = Path(__file__).resolve().parent.parent / "tuning_results"


def _wait_for_joints(robot, timeout: float = 10.0):
    """Wait until joint states arrive (this task is pure joint-space; no pose needed)."""
    t0 = time.monotonic()
    while robot._current_joint is None:
        if time.monotonic() - t0 > timeout:
            raise TimeoutError(
                "No joint states on 'joint_states'. Is joint_state_broadcaster active?")
        time.sleep(0.05)


def compute_amplitude(joint_idx: int, center: np.ndarray, freq: float, speed_frac: float) -> float:
    """Amplitude so peak sine velocity ~= speed_frac * vmax, clamped to safe range."""
    amp_speed = speed_frac * FR3_VMAX[joint_idx] / (2.0 * np.pi * freq)
    margin = min(center[joint_idx] - FR3_Q_LOWER[joint_idx],
                 FR3_Q_UPPER[joint_idx] - center[joint_idx]) - POS_MARGIN
    return float(min(amp_speed, AMP_CEILING, max(margin, 0.0)))


def build_reference(center: np.ndarray, joint_idx: int, amp: float, freq: float, duration: float):
    """Sampled sine reference for the tested joint; others held at center."""
    n = max(int(duration * SAMPLE_RATE), 2)
    t = np.linspace(0.0, duration, n)
    w = 2.0 * np.pi * freq
    q = np.tile(center, (n, 1))
    q[:, joint_idx] = center[joint_idx] + amp * np.sin(w * t)
    return t, q


def make_plot(joint_num: int, t: np.ndarray, q_ref_j: np.ndarray):
    """Create the live figure; returns (fig, line handles, axes)."""
    fig, (ax_pos, ax_err) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)

    (l_cmd,) = ax_pos.plot(t, q_ref_j, "C1--", label="commanded")
    (l_act,) = ax_pos.plot([], [], "C0-", label="actual")
    ax_pos.set_ylabel("position [rad]")
    ax_pos.set_title(f"Joint {joint_num} tracking")
    ax_pos.legend(loc="upper right")
    ax_pos.grid(True)

    (l_err,) = ax_err.plot([], [], "C3-", label="error (cmd - actual)")
    ax_err.set_ylabel("error [rad]")
    ax_err.set_xlabel("time [s]")
    ax_err.axhline(0.0, color="k", lw=0.5)
    ax_err.legend(loc="upper right")
    ax_err.grid(True)

    ax_pos.set_xlim(t[0], t[-1])
    fig.tight_layout()
    return fig, (l_cmd, l_act, l_err), (ax_pos, ax_err)


def run_preview(args, center: np.ndarray, joint_idx: int):
    """Offline: just plot the planned reference, no robot connection."""
    amp = compute_amplitude(joint_idx, center, args.freq, args.speed_frac)
    duration = args.cycles / args.freq
    peak_vel = amp * 2.0 * np.pi * args.freq
    print(f"[preview] joint {args.joint}: amplitude={amp:.3f} rad, "
          f"peak velocity={peak_vel:.3f} rad/s "
          f"({peak_vel / FR3_VMAX[joint_idx] * 100:.0f}% of vmax={FR3_VMAX[joint_idx]:.2f}), "
          f"duration={duration:.1f} s")
    t, q_ref = build_reference(center, joint_idx, amp, args.freq, duration)
    fig, lines, _ = make_plot(args.joint, t, q_ref[:, joint_idx])
    lines[1].set_data(t, q_ref[:, joint_idx])  # actual == commanded in preview
    lines[2].set_data(t, np.zeros_like(t))
    plt.show()


def _fmt(x):
    return "default" if x is None else f"{x:g}"


def run_hardware(args, joint_idx: int):
    """Connect, switch to the crisp PD controller, stream the sine, record + plot."""
    from crisp_py.control.parameters_client import ParametersClient
    from crisp_py.robot import make_robot

    robot = make_robot("fr3")
    joint_names = robot.config.joint_names
    joint_name = joint_names[joint_idx]

    print("Waiting for joint states...")
    _wait_for_joints(robot)

    # Center the sine on the CURRENT joint configuration -> no homing / large moves.
    center = robot.joint_values.astype(float)
    robot.set_target_joint(center.copy())  # hold current pose as we switch

    print(f"Switching to {CONTROLLER}...")
    robot.controller_switcher_client.switch_controller(CONTROLLER)

    # crisp_py's switcher uses BEST_EFFORT strictness -> a failed activation does NOT
    # raise. Verify explicitly; if it didn't activate, the FR3 isn't ready for torque.
    state = {c.name: c.state for c in robot.controller_switcher_client.get_controller_list()}
    if state.get(CONTROLLER) != "active":
        robot.shutdown()
        raise SystemExit(
            f"{CONTROLLER} did not become active (state={state.get(CONTROLLER)}).\n"
            "The FR3 likely isn't ready for torque control. In Franka Desk: clear errors, "
            "UNLOCK the joints, ACTIVATE FCI, then relaunch and retry.")

    # Tune the crisp nullspace PD live.
    pc = ParametersClient(robot.node, target_node=CONTROLLER)
    pc.wait_until_ready()
    updates = []
    if args.stiffness is not None:
        updates.append(("nullspace.stiffness", float(args.stiffness)))
    if args.damping is not None:
        updates.append(("nullspace.damping", float(args.damping)))
    if args.weight is not None:
        updates.append((f"nullspace.weights.{joint_name}.value", float(args.weight)))
    if updates:
        try:
            pc.set_parameters(updates)
            print(f"Set PD params: {updates}")
        except Exception as exc:  # noqa: BLE001
            print(f"WARNING: could not set params live ({exc}).\n"
                  f"         Edit the joint_impedance_controller block in "
                  f"config/controllers.yaml and relaunch.")

    # Read back effective gains for labeling.
    try:
        K, D, W = pc.get_parameters(
            ["nullspace.stiffness", "nullspace.damping",
             f"nullspace.weights.{joint_name}.value"])
    except Exception:  # noqa: BLE001
        K = D = W = None

    amp = compute_amplitude(joint_idx, center, args.freq, args.speed_frac)
    duration = args.cycles / args.freq
    w = 2.0 * np.pi * args.freq
    peak_vel = amp * w
    print(f"Joint {args.joint}: amplitude={amp:.3f} rad, peak velocity={peak_vel:.3f} rad/s "
          f"({peak_vel / FR3_VMAX[joint_idx] * 100:.0f}% of vmax), duration={duration:.1f} s")
    eff_p = None if (K is None or W is None) else K * W
    eff_d = None if (D is None or W is None or D < 0) else D * W
    print(f"PD: stiffness={_fmt(K)} damping={_fmt(D)} weight={_fmt(W)}  "
          f"-> effective P~{_fmt(eff_p)} D~{_fmt(eff_d)}{' (auto)' if (D is not None and D < 0) else ''}")

    t_ref, q_ref = build_reference(center, joint_idx, amp, args.freq, duration)

    # Live plot.
    plt.ion()
    fig, (l_cmd, l_act, l_err), _ = make_plot(args.joint, t_ref, q_ref[:, joint_idx])
    fig.show()

    print("Streaming sine to /target_joint ...")
    ts, q_act, q_cmd = [], [], []
    target = center.copy()
    t0 = time.monotonic()
    dt = 1.0 / SAMPLE_RATE
    end = duration + SETTLE_TIME
    i = 0
    while True:
        now = time.monotonic() - t0
        if now > end:
            break
        tau_t = min(now, duration)
        cmd = center[joint_idx] + amp * np.sin(w * tau_t)
        target[joint_idx] = cmd
        robot.set_target_joint(target.copy())

        ts.append(now)
        q_cmd.append(cmd)
        q_act.append(float(robot.joint_values[joint_idx]))

        if i % PLOT_EVERY == 0:
            l_act.set_data(ts, q_act)
            l_cmd.set_data(ts, q_cmd)
            l_err.set_data(ts, np.array(q_cmd) - np.array(q_act))
            for ax in fig.axes:
                ax.relim()
                ax.autoscale_view(scalex=False)
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
        i += 1
        time.sleep(max(0.0, dt - (time.monotonic() - t0 - now)))

    robot.set_target_joint(center.copy())  # settle back at center

    # Stats.
    err = np.array(q_cmd) - np.array(q_act)
    rms = float(np.sqrt(np.mean(err**2)))
    mx = float(np.max(np.abs(err)))
    print(f"Tracking error  RMS={rms*1e3:.2f} mrad  max={mx*1e3:.2f} mrad  "
          f"(stiffness={_fmt(K)}, damping={_fmt(D)}, weight={_fmt(W)})")

    # Final redraw + save.
    l_act.set_data(ts, q_act)
    l_cmd.set_data(ts, q_cmd)
    l_err.set_data(ts, err)
    fig.axes[0].set_title(
        f"Joint {args.joint}  K={_fmt(K)} D={_fmt(D)} w={_fmt(W)}  "
        f"RMS={rms*1e3:.1f} mrad, max={mx*1e3:.1f} mrad")
    for ax in fig.axes:
        ax.relim()
        ax.autoscale_view(scalex=False)
    fig.canvas.draw_idle()

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = RESULTS_DIR / f"joint{args.joint}_K{_fmt(K)}_D{_fmt(D)}_w{_fmt(W)}_{stamp}.png"
    fig.savefig(out, dpi=120)
    print(f"Saved plot to {out}")

    plt.ioff()
    print("Close the plot window to exit.")
    plt.show()
    robot.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--joint", type=int, default=6, choices=range(1, 8),
                        help="Joint to excite (1-7); others held. Default 6.")
    parser.add_argument("--freq", type=float, default=0.2, help="Sine frequency [Hz]. Default 0.2.")
    parser.add_argument("--cycles", type=float, default=3.0,
                        help="Number of sine cycles (run length = cycles/freq). Default 3.")
    parser.add_argument("--speed-frac", type=float, default=0.2,
                        help="Peak velocity as fraction of joint max speed. Default 0.2.")
    parser.add_argument("--stiffness", type=float, default=None,
                        help="nullspace.stiffness (global P gain).")
    parser.add_argument("--damping", type=float, default=None,
                        help="nullspace.damping (global D gain; -1 => auto 2*sqrt(stiffness)).")
    parser.add_argument("--weight", type=float, default=None,
                        help="nullspace.weights.<joint>.value (per-joint scale for the tested joint).")
    parser.add_argument("--preview", action="store_true",
                        help="Offline: plot the planned reference only, no robot.")
    args = parser.parse_args()

    joint_idx = args.joint - 1
    if args.preview:
        # Default home pose only used as a plausible center for the offline plot.
        center = np.array([0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4])
        run_preview(args, center, joint_idx)
    else:
        run_hardware(args, joint_idx)


if __name__ == "__main__":
    main()
