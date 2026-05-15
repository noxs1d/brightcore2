#!/usr/bin/env python3
"""Optuna tuner for parameterized state-machine control_lane_param on AutoRace 2020.

Launches autorace_sim_with_param.launch headless once per episode, overlays a
trial YAML onto control_lane tuned_file, rewards progress + lap finish detection
(red finish log line).

Typical usage::
    cd ~/catkin_ws && source devel/setup.bash
    export TURTLEBOT3_MODEL=burger
    roscore  # terminal 1

    rosrun turtlebot3_autorace_driving tune_control_lane.py \\
        --trials 150 --episode-timeout 75 \\
        --sim-launch "$(rospack find turtlebot3_autorace_driving)/launch/autorace_sim_with_param.launch" \\
        --storage sqlite:///$HOME/cl_study.db \\
        --study-name control_lane_tune

Requires rospy, optuna, pyyaml, rospkg, and turtlebot3_autorace_* packages sourced.
"""

from __future__ import annotations

import argparse
import math
import os
import random
import signal
import subprocess
import sys
import tempfile
import threading
import time
from typing import Any, Dict

import yaml

try:
    import optuna
except ImportError:
    print("ERROR: pip install optuna pyyaml rospkg",
          file=sys.stderr)
    raise

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
from std_msgs.msg import Float64


# Optuna dotted keys -> YAML tree (nested under control_lane node's private ns)
SEARCH_SPACE = {
    "gains.straight.kp":           (0.0020, 0.0060, "float"),
    "gains.straight.kd":           (0.0040, 0.0120, "float"),
    "gains.curve.kp":              (0.0035, 0.0090, "float"),
    "gains.curve.kd":              (0.0070, 0.0180, "float"),
    "gains.circle.kp":             (0.0050, 0.0110, "float"),
    "gains.circle.kd":             (0.0090, 0.0220, "float"),
    "max_speed.straight":          (0.15,   0.30,   "float"),
    "max_speed.curve":             (0.08,   0.16,   "float"),
    "max_speed.circle":            (0.04,   0.10,   "float"),
    "thresholds.straight_err":     (35.0,   75.0,   "float"),
    "thresholds.curve_err":        (80.0,   140.0,  "float"),
    "thresholds.circle_var":       (1500.0, 5000.0, "float"),
    "thresholds.circle_scr":       (0.15,   0.35,   "float"),
    "hsv.yellow.s_low":            (60,     110,    "int"),
    "hsv.yellow.v_low":            (60,     110,    "int"),
}


def _set_dotted(target: Dict[str, Any], dotted_key: str, value: Any) -> None:
    parts = dotted_key.split(".")
    cur = target
    for part in parts[:-1]:
        cur = cur.setdefault(part, {})
    cur[parts[-1]] = value


def sample_overlay(trial: "optuna.trial.Trial") -> Dict[str, Any]:
    overlay: Dict[str, Any] = {}
    for key, (low, high, kind) in SEARCH_SPACE.items():
        if kind == "float":
            val = trial.suggest_float(key, low, high)
        else:
            val = trial.suggest_int(key, int(low), int(high))
        _set_dotted(overlay, key, val)

    gains = overlay.get("gains", {})
    for state in ("straight", "curve", "circle"):
        g = gains.get(state, {})
        kp = float(g["kp"])
        kd = float(g["kd"])
        ki_def = kp * (0.0001 / 0.0035) if state == "straight" else (
            kp * (0.0002 / 0.0055) if state == "curve" else kp * (0.0004 / 0.0070)
        )
        _set_dotted(overlay, f"gains.{state}.ki", float(round(ki_def, 6)))

    return overlay


def write_yaml(path: str, data: Dict[str, Any]) -> None:
    with open(path, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)


class _FinishLogMatcher(threading.Thread):
    """control_lane_param finishes with log line containing RED ZONE — FINISHED!"""

    def __init__(self, pattern: str = "RED ZONE") -> None:
        super().__init__(daemon=True)
        self.pattern = pattern
        self.finished = threading.Event()
        self._sub = None

    def _cb(self, msg: Log) -> None:
        if self.pattern in msg.msg:
            self.finished.set()

    def run(self) -> None:
        try:
            self._sub = rospy.Subscriber("/rosout_agg", Log, self._cb, queue_size=50)
            while not rospy.is_shutdown() and not self.finished.is_set():
                rospy.rostime.wallsleep(0.05)
        except Exception:
            pass


class EpisodeRunner:
    """One full-stack roslaunch (Gazebo + camera + detect + control_lane_param)."""

    STARTUP_PAUSE = 15.0

    def __init__(self, sim_launch: str, overlay_path: str, timeout: float) -> None:
        self.sim_launch = sim_launch
        self.overlay_path = overlay_path
        self.timeout = timeout

        self._procs: list = []
        self._odom_sub = None
        self._lane_sub = None
        self._cmd_sub = None
        self._log_matcher = None

        self.start_pos = None
        self.last_pos = None
        self.progress = 0.0
        self.off_track_time = 0.0
        self.last_lane_time = None
        self.episode_time = 0.0
        self.finished = False
        self.crashed = False
        self.lost_samples = 0

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        if self.start_pos is None:
            self.start_pos = (p.x, p.y)
        if self.last_pos is not None:
            dx = p.x - self.last_pos[0]
            dy = p.y - self.last_pos[1]
            self.progress += math.hypot(dx, dy)
        self.last_pos = (p.x, p.y)

    def _lane_cb(self, msg: Float64) -> None:
        """Proxy off-track heuristic (detect publishes lane-centre-ish float)."""
        now = time.monotonic()
        err = abs(float(msg.data) - 500.0)
        if self.last_lane_time is not None and err > 80.0:
            self.off_track_time += now - self.last_lane_time
        self.last_lane_time = now

    def _cmd_cb(self, msg: Twist) -> None:
        if abs(msg.linear.x) < 1e-3 and abs(msg.angular.z) < 1e-3:
            self.lost_samples += 1

    def _spawn(self, cmd: list) -> subprocess.Popen:
        env = os.environ.copy()
        env.setdefault("TURTLEBOT3_MODEL", "burger")
        proc = subprocess.Popen(
            cmd,
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        self._procs.append(proc)
        return proc

    def _kill(self) -> None:
        for proc in self._procs:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + 8.0
        for proc in self._procs:
            while proc.poll() is None and time.monotonic() < deadline:
                time.sleep(0.05)
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self._procs.clear()

    def run(self) -> Dict[str, Any]:
        launch_abs = os.path.abspath(os.path.expanduser(self.sim_launch))
        gazebo_cmd = [
            "roslaunch",
            launch_abs,
            "gui:=false",
            "tuned_file:=" + self.overlay_path,
        ]

        try:
            self._spawn(gazebo_cmd)
            time.sleep(self.STARTUP_PAUSE)

            self._odom_sub = rospy.Subscriber("/odom", Odometry, self._odom_cb,
                                              queue_size=50)
            self._lane_sub = rospy.Subscriber(
                "/detect/lane",
                Float64,
                self._lane_cb,
                queue_size=50,
            )
            self._cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self._cmd_cb,
                                             queue_size=50)

            self._log_matcher = _FinishLogMatcher()
            self._log_matcher.start()

            t0 = time.monotonic()
            while time.monotonic() - t0 < self.timeout:
                if self._log_matcher.finished.is_set():
                    self.finished = True
                    break
                if self.lost_samples > 350:
                    self.crashed = True
                    break
                rospy.rostime.wallsleep(0.1)
            self.episode_time = time.monotonic() - t0
        finally:
            for sub in (self._odom_sub, self._lane_sub, self._cmd_sub):
                if sub is not None:
                    try:
                        sub.unregister()
                    except Exception:
                        pass
            self._kill()

        return {
            "finished": self.finished,
            "progress": self.progress,
            "episode_time": self.episode_time,
            "off_track_time": self.off_track_time,
            "crashed": self.crashed,
        }


def compute_reward(metrics: Dict[str, Any]) -> float:
    progress = float(metrics.get("progress", 0.0))
    off_track = float(metrics.get("off_track_time", 0.0))
    episode_time = float(metrics.get("episode_time", 0.0))
    finished = 1.0 if metrics.get("finished") else 0.0
    crashed = 1.0 if metrics.get("crashed") else 0.0
    return (
        10.0 * finished
        + progress
        - 0.05 * episode_time
        - 5.0 * off_track
        - 5.0 * crashed
    )


def make_objective(args: argparse.Namespace):
    def objective(trial: optuna.trial.Trial) -> float:
        overlay = sample_overlay(trial)
        with tempfile.NamedTemporaryFile(
                mode="w", suffix=".yaml", prefix="cl_trial_", delete=False,
        ) as f:
            overlay_path = f.name
        try:
            write_yaml(overlay_path, overlay)
            runner = EpisodeRunner(
                sim_launch=args.sim_launch,
                overlay_path=os.path.abspath(overlay_path),
                timeout=args.episode_timeout,
            )
            metrics = runner.run()
        finally:
            try:
                os.unlink(overlay_path)
            except OSError:
                pass

        reward = compute_reward(metrics)
        rospy.loginfo(
            "trial=%d reward=%.2f finished=%s progress=%.2fm time=%.1fs off_track=%.2fs",
            trial.number, reward, metrics["finished"], metrics["progress"],
            metrics["episode_time"], metrics["off_track_time"])
        for k, v in metrics.items():
            trial.set_user_attr(k, v)
        return reward

    return objective


def save_best(study: optuna.study.Study, output_path: str) -> None:
    best = study.best_trial
    overlay: Dict[str, Any] = {}
    for key in SEARCH_SPACE:
        if key in best.params:
            _set_dotted(overlay, key, best.params[key])
    for state in ("straight", "curve", "circle"):
        kp = overlay.get("gains", {}).get(state, {}).get("kp")
        kd = overlay.get("gains", {}).get(state, {}).get("kd")
        if kp is None:
            continue
        kp = float(kp)
        factor = {"straight": 0.0001 / 0.0035,
                  "curve": 0.0002 / 0.0055,
                  "circle": 0.0004 / 0.0070}[state]
        _set_dotted(overlay, f"gains.{state}.ki", float(round(kp * factor, 6)))
    write_yaml(output_path, overlay)
    print(f"\nBest reward: {best.value:.3f}")
    print(f"Best params saved to: {output_path}")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--trials", type=int, default=150)
    p.add_argument("--episode-timeout", type=float, default=75.0)
    p.add_argument(
        "--sim-launch",
        required=True,
        help="ABSOLUTE path to autorace_sim_with_param.launch (use rospack find ...)",
    )
    p.add_argument(
        "--output",
        default=None,
        help="Write best overlay here (default: package param/tuned_control_lane.yaml)",
    )
    p.add_argument("--study-name", default="control_lane_tune")
    p.add_argument("--storage", default=None)
    p.add_argument("--seed", type=int, default=None)
    return p.parse_args()


def resolve_output(args: argparse.Namespace) -> None:
    if args.output is not None:
        return
    import rospkg
    rp = rospkg.RosPack()
    pkg = rp.get_path("turtlebot3_autorace_driving")
    args.output = os.path.join(pkg, "param", "tuned_control_lane.yaml")


def main() -> None:
    args = parse_args()
    resolve_output(args)

    if args.seed is not None:
        random.seed(args.seed)

    launch_path = os.path.abspath(os.path.expanduser(args.sim_launch))
    if not os.path.isfile(launch_path):
        print(f"--sim-launch must be absolute path to a .launch file, got:\n {args.sim_launch}",
              file=sys.stderr)
        sys.exit(1)
    args.sim_launch = launch_path

    rospy.init_node("control_lane_tuner", anonymous=True, disable_signals=True)

    sampler = optuna.samplers.TPESampler(seed=args.seed)
    study = optuna.create_study(
        study_name=args.study_name,
        storage=args.storage,
        direction="maximize",
        sampler=sampler,
        load_if_exists=True,
    )

    try:
        study.optimize(make_objective(args), n_trials=args.trials)
    except KeyboardInterrupt:
        rospy.logwarn("Interrupted — saving best-so-far...")
    finally:
        if study.trials and study.best_trial is not None:
            save_best(study, os.path.abspath(args.output))


if __name__ == "__main__":
    main()
