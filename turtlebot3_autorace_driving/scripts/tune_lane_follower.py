#!/usr/bin/env python3
"""Optuna-driven auto-tuning of the scanline lane follower.

Runs many headless Gazebo episodes of the autorace track, varies a handful
of high-impact parameters per trial, scores each episode and saves the best
parameters as a YAML overlay that can be loaded by the launch file via the
``tuned_file`` argument.

Designed to be lightweight: only depends on rospy + optuna + pyyaml +
subprocess. No new ROS messages, no compiled code.

Typical usage from a ROS-sourced shell::

    cd ~/catkin_ws && source devel/setup.bash
    export TURTLEBOT3_MODEL=burger
    rosrun turtlebot3_autorace_driving tune_lane_follower.py \
        --trials 80 --episode-timeout 60 \
        --gazebo-launch ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_autorace_2020.launch \
        --output $(rospack find turtlebot3_autorace_driving)/param/tuned_lane_follower.yaml

The script logs progress to stdout and writes intermediate study state to a
SQLite file so the search can be resumed after a Ctrl+C.
"""

from __future__ import annotations

import argparse
import math
import os
import random
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from contextlib import contextmanager
from typing import Any, Dict, Optional

import yaml

try:
    import optuna
except ImportError as exc:  # pragma: no cover - executed only when missing dep
    print("ERROR: optuna is required. Install with `pip install optuna pyyaml rospkg`",
          file=sys.stderr)
    raise

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
from std_msgs.msg import Float32


# ---------------------------------------------------------------------------
# Parameter space
# ---------------------------------------------------------------------------

# Each entry: yaml dotted path -> (low, high, type)
SEARCH_SPACE = {
    "pid.Kp":                       (0.005, 0.015, "float"),
    "pid.Kd":                       (0.001, 0.010, "float"),
    "speed.base_linear":            (0.05,  0.10,  "float"),
    "speed.max_angular":            (1.0,   1.7,   "float"),
    "control.single_line_turn_gain": (1.0,   2.0,   "float"),
    "control.lost_decay":           (0.6,   0.95,  "float"),
    "detection.max_lost_frames":    (4,     12,    "int"),
    "detection.max_center_jump_px": (80,    200,   "int"),
}

# Domain randomization: tiny coordinated jitter applied to each trial so that
# parameters that survive across small visual perturbations transfer better
# to the real robot. Each entry produces an *additive* delta to the base
# value drawn uniformly from [-amp, amp].
DOMAIN_RANDOMIZATION = {
    "yellow_lane.v_low":         15,
    "yellow_lane.s_low":         15,
    "perspective.src_top_left":  ("y", 0.05),
    "perspective.src_top_right": ("y", 0.05),
}


# ---------------------------------------------------------------------------
# Helpers for nested dict access
# ---------------------------------------------------------------------------

def _set_dotted(target: Dict[str, Any], dotted_key: str, value: Any) -> None:
    parts = dotted_key.split(".")
    cur = target
    for part in parts[:-1]:
        cur = cur.setdefault(part, {})
    cur[parts[-1]] = value


def _get_dotted(target: Dict[str, Any], dotted_key: str, default: Any = None) -> Any:
    parts = dotted_key.split(".")
    cur = target
    for part in parts:
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    return cur


def _load_base_params(base_yaml: str) -> Dict[str, Any]:
    with open(base_yaml, "r") as f:
        return yaml.safe_load(f) or {}


# ---------------------------------------------------------------------------
# Trial-level YAML generation
# ---------------------------------------------------------------------------

def sample_overlay(trial: "optuna.trial.Trial", base_params: Dict[str, Any]) -> Dict[str, Any]:
    overlay: Dict[str, Any] = {}
    for key, (low, high, kind) in SEARCH_SPACE.items():
        if kind == "float":
            value = trial.suggest_float(key, low, high)
        else:
            value = trial.suggest_int(key, int(low), int(high))
        _set_dotted(overlay, key, value)

    # Domain randomization (does not occupy Optuna's parameter space - it is
    # noise within a trial, sampled fresh on top of the suggested parameters).
    for key, spec in DOMAIN_RANDOMIZATION.items():
        if isinstance(spec, tuple):
            axis, amp = spec
            base = _get_dotted(base_params, key)
            if isinstance(base, list) and len(base) == 2:
                jittered = list(base)
                if axis == "x":
                    jittered[0] = float(base[0]) + random.uniform(-amp, amp)
                else:
                    jittered[1] = float(base[1]) + random.uniform(-amp, amp)
                _set_dotted(overlay, key, jittered)
        else:
            amp = float(spec)
            base = _get_dotted(base_params, key, 0)
            value = float(base) + random.uniform(-amp, amp)
            _set_dotted(overlay, key, max(0.0, value))

    return overlay


def write_yaml(path: str, data: Dict[str, Any]) -> None:
    with open(path, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)


# ---------------------------------------------------------------------------
# Roslaunch episode runner
# ---------------------------------------------------------------------------

class _LogMatcher(threading.Thread):
    """Tail ``/rosout_agg`` for the finish-marker log line."""

    def __init__(self, pattern: str = "Finish marker detected"):
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
    """Run one Gazebo + lane_follower episode and collect metrics."""

    def __init__(
        self,
        gazebo_launch: str,
        lf_launch: str,
        overlay_path: str,
        timeout: float,
        camera_topic: str,
        camera_compressed_topic: str,
    ) -> None:
        self.gazebo_launch = gazebo_launch
        self.lf_launch = lf_launch
        self.overlay_path = overlay_path
        self.timeout = timeout
        self.camera_topic = camera_topic
        self.camera_compressed_topic = camera_compressed_topic

        self._procs = []
        self._odom_sub = None
        self._err_sub = None
        self._cmd_sub = None
        self._log_matcher = None

        # Metrics
        self.start_pos = None
        self.last_pos = None
        self.progress = 0.0
        self.off_track_time = 0.0
        self.last_err_time = None
        self.episode_time = 0.0
        self.finished = False
        self.crashed = False
        self.error_samples = 0
        self.lost_samples = 0

    # -- subscribers -------------------------------------------------------

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        if self.start_pos is None:
            self.start_pos = (p.x, p.y)
        if self.last_pos is not None:
            dx = p.x - self.last_pos[0]
            dy = p.y - self.last_pos[1]
            self.progress += math.hypot(dx, dy)
        self.last_pos = (p.x, p.y)

    def _err_cb(self, msg: Float32) -> None:
        now = time.monotonic()
        if self.last_err_time is not None and abs(msg.data) > 80.0:
            self.off_track_time += now - self.last_err_time
        self.last_err_time = now
        self.error_samples += 1

    def _cmd_cb(self, msg: Twist) -> None:
        # Track frames where the robot is sitting still and steering hard -
        # a sign of being stuck.
        if abs(msg.linear.x) < 1e-3 and abs(msg.angular.z) < 1e-3:
            self.lost_samples += 1

    # -- process management ------------------------------------------------

    def _spawn(self, cmd: list) -> subprocess.Popen:
        env = os.environ.copy()
        env.setdefault("TURTLEBOT3_MODEL", "burger")
        proc = subprocess.Popen(
            cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
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
        deadline = time.monotonic() + 5.0
        for proc in self._procs:
            while proc.poll() is None and time.monotonic() < deadline:
                time.sleep(0.05)
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self._procs.clear()

    # -- run ---------------------------------------------------------------

    def run(self) -> Dict[str, Any]:
        gazebo_cmd = [
            "roslaunch", self.gazebo_launch, "gui:=false",
        ]
        lf_cmd = [
            "roslaunch",
            "turtlebot3_autorace_driving",
            "turtlebot3_autorace_scanline_lane_following.launch",
            "mode:=action",
            f"camera_topic:={self.camera_topic}",
            f"camera_compressed_topic:={self.camera_compressed_topic}",
            f"tuned_file:={self.overlay_path}",
        ]
        try:
            self._spawn(gazebo_cmd)
            # Give gazebo a moment to spin up before the follower attaches.
            time.sleep(8.0)
            self._spawn(lf_cmd)

            self._odom_sub = rospy.Subscriber("/odom", Odometry, self._odom_cb, queue_size=50)
            self._err_sub = rospy.Subscriber("/lane_error", Float32, self._err_cb, queue_size=50)
            self._cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self._cmd_cb, queue_size=50)

            self._log_matcher = _LogMatcher()
            self._log_matcher.start()

            t0 = time.monotonic()
            while time.monotonic() - t0 < self.timeout:
                if self._log_matcher.finished.is_set():
                    self.finished = True
                    break
                # Crash heuristic: if the cmd_vel has been zero for a long
                # stretch (10s) without the finish flag, bail out early.
                if self.lost_samples > 200:
                    self.crashed = True
                    break
                rospy.rostime.wallsleep(0.1)
            self.episode_time = time.monotonic() - t0
        finally:
            for sub in (self._odom_sub, self._err_sub, self._cmd_sub):
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


# ---------------------------------------------------------------------------
# Reward
# ---------------------------------------------------------------------------

def compute_reward(metrics: Dict[str, Any]) -> float:
    progress = float(metrics.get("progress", 0.0))
    off_track = float(metrics.get("off_track_time", 0.0))
    episode_time = float(metrics.get("episode_time", 0.0))
    finished = 1.0 if metrics.get("finished") else 0.0
    crashed = 1.0 if metrics.get("crashed") else 0.0
    reward = 10.0 * finished + progress - 0.05 * episode_time - 5.0 * off_track - 5.0 * crashed
    return reward


# ---------------------------------------------------------------------------
# Optuna glue
# ---------------------------------------------------------------------------

def make_objective(args, base_params: Dict[str, Any]):
    def objective(trial: "optuna.trial.Trial") -> float:
        overlay = sample_overlay(trial, base_params)
        with tempfile.NamedTemporaryFile(
                "w", suffix=".yaml", prefix="lf_trial_", delete=False) as f:
            overlay_path = f.name
        try:
            write_yaml(overlay_path, overlay)
            runner = EpisodeRunner(
                gazebo_launch=args.gazebo_launch,
                lf_launch=args.lf_launch,
                overlay_path=overlay_path,
                timeout=args.episode_timeout,
                camera_topic=args.camera_topic,
                camera_compressed_topic=args.camera_compressed_topic,
            )
            metrics = runner.run()
        finally:
            try:
                os.remove(overlay_path)
            except OSError:
                pass
        reward = compute_reward(metrics)
        rospy.loginfo(
            "trial=%d reward=%.2f finished=%s progress=%.2fm time=%.1fs off_track=%.2fs",
            trial.number, reward, metrics["finished"], metrics["progress"],
            metrics["episode_time"], metrics["off_track_time"])
        # Record auxiliary metrics
        for k, v in metrics.items():
            trial.set_user_attr(k, v)
        return reward
    return objective


def save_best(study: "optuna.study.Study", output_path: str) -> None:
    best = study.best_trial
    overlay: Dict[str, Any] = {}
    for key in SEARCH_SPACE:
        if key in best.params:
            _set_dotted(overlay, key, best.params[key])
    write_yaml(output_path, overlay)
    print(f"\nBest reward: {best.value:.3f}")
    print(f"Best params saved to: {output_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=80)
    parser.add_argument("--episode-timeout", type=float, default=60.0,
                        help="seconds to allow per episode")
    parser.add_argument("--gazebo-launch", required=True,
                        help="full path to turtlebot3_autorace_2020.launch (resolves rospack ambiguity)")
    parser.add_argument("--lf-launch", default="turtlebot3_autorace_scanline_lane_following.launch",
                        help="lane follower launch file (resolved via rospack)")
    parser.add_argument("--base-params",
                        default=None,
                        help="path to base lane_follower_scanline.yaml (defaults to package param/)")
    parser.add_argument("--output",
                        default=None,
                        help="path to write best tuned overlay YAML (defaults to package param/tuned_lane_follower.yaml)")
    parser.add_argument("--study-name", default="lane_follower_tune")
    parser.add_argument("--storage", default=None,
                        help="optional optuna storage URL, e.g. sqlite:///lf_study.db (enables resume)")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--camera-topic", default="/camera/image",
                        help="Gazebo camera raw topic (e.g. /camera/image for autorace world)")
    parser.add_argument("--camera-compressed-topic", default="/camera/image/compressed")
    return parser.parse_args()


def resolve_default_paths(args: argparse.Namespace) -> None:
    if args.base_params is None or args.output is None:
        import rospkg
        rospack = rospkg.RosPack()
        try:
            pkg_path = rospack.get_path("turtlebot3_autorace_driving")
        except rospkg.ResourceNotFound:
            print("turtlebot3_autorace_driving package not found in ROS_PACKAGE_PATH",
                  file=sys.stderr)
            sys.exit(1)
        if args.base_params is None:
            args.base_params = os.path.join(pkg_path, "param", "lane_follower_scanline.yaml")
        if args.output is None:
            args.output = os.path.join(pkg_path, "param", "tuned_lane_follower.yaml")


def main() -> None:
    args = parse_args()
    resolve_default_paths(args)

    if args.seed is not None:
        random.seed(args.seed)

    if not os.path.exists(args.gazebo_launch):
        print(f"gazebo launch file not found: {args.gazebo_launch}", file=sys.stderr)
        sys.exit(1)
    if not os.path.exists(args.base_params):
        print(f"base params not found: {args.base_params}", file=sys.stderr)
        sys.exit(1)

    base_params = _load_base_params(args.base_params)

    rospy.init_node("lane_follower_tuner", anonymous=True, disable_signals=True)

    sampler = optuna.samplers.TPESampler(seed=args.seed)
    study = optuna.create_study(
        study_name=args.study_name,
        storage=args.storage,
        direction="maximize",
        sampler=sampler,
        load_if_exists=True,
    )

    try:
        study.optimize(make_objective(args, base_params), n_trials=args.trials)
    except KeyboardInterrupt:
        rospy.logwarn("Interrupted by user. Saving best-so-far ...")
    finally:
        if len(study.trials) > 0 and study.best_trial is not None:
            save_best(study, args.output)


if __name__ == "__main__":
    main()
