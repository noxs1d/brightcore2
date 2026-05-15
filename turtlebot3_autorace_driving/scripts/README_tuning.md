# Auto-tuning the scanline lane follower

This package ships with `scripts/tune_lane_follower.py`, an Optuna-based
black-box optimizer that runs many headless Gazebo episodes of the AutoRace
2020 track and saves the best-scoring parameter overlay as
`param/tuned_lane_follower.yaml`. The tuned file is loaded automatically
when you pass `tuned_file:=...` to the lane-follower launch.

The white lane is fully disabled in the runtime, so no parameter related
to it is tuned. Random domain perturbations are applied on top of the
parameter suggestions to make the result transfer to the real robot.

## 1. One-time prerequisites

```bash
pip install --user optuna pyyaml rospkg
sudo apt install ros-noetic-turtlebot3-simulations  # if not already installed
```

If `roslaunch turtlebot3_gazebo turtlebot3_autorace_2020.launch` complains
about ambiguity between `/opt/ros/...` and `~/catkin_ws/src/...`, pass the
full path (the script wants the full path anyway).

Build and source the workspace once after the script was added:

```bash
cd ~/catkin_ws && catkin_make
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
```

## 2. Launching the auto-tune (sim only)

### 2.a. Recommended: BrightCore custom track

The package now ships a Gazebo world built from `CD-digital map.png`. The
robot spawns inside the red bracket (bottom-right of the texture) facing
the bottom corridor, which matches the real-world starting position.

```bash
GAZEBO_LAUNCH=$(rospack find turtlebot3_autorace_driving)/launch/brightcore_track.launch

rosrun turtlebot3_autorace_driving tune_lane_follower.py \
    --trials 80 \
    --episode-timeout 60 \
    --gazebo-launch "$GAZEBO_LAUNCH" \
    --storage sqlite:///$HOME/lf_study.db \
    --study-name lane_follower_tune
```

To preview the world (no tuning, just look at it):

```bash
roslaunch turtlebot3_autorace_driving brightcore_track.launch gui:=true
```

To preview the world AND let the lane follower drive on it:

```bash
roslaunch turtlebot3_autorace_driving brightcore_track.launch \
    gui:=true run_lane_follower:=true rear_camera:=false
```

### 2.b. Alternative: stock AutoRace 2020 track

```bash
GAZEBO_LAUNCH=~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_autorace_2020.launch

rosrun turtlebot3_autorace_driving tune_lane_follower.py \
    --trials 80 \
    --episode-timeout 60 \
    --gazebo-launch "$GAZEBO_LAUNCH" \
    --storage sqlite:///$HOME/lf_study.db \
    --study-name lane_follower_tune
```

Notes:

- `--trials 80` typically takes 1.5-2.5 hours on a desktop CPU; reduce to
  30 for a quick smoke run.
- `--storage sqlite:///...` lets you resume the study after `Ctrl+C` by
  re-running the same command.
- The script writes the best parameters to
  `param/tuned_lane_follower.yaml` of the
  `turtlebot3_autorace_driving` package and updates that file every time
  the optimization finishes a better trial.

## 3. Verifying the tuned file

```bash
cat $(rospack find turtlebot3_autorace_driving)/param/tuned_lane_follower.yaml
```

Expected: a small YAML containing dotted-section subset like

```yaml
pid:
  Kp: 0.0083
  Kd: 0.0042
speed:
  base_linear: 0.072
  max_angular: 1.45
control:
  single_line_turn_gain: 1.62
  lost_decay: 0.78
detection:
  max_lost_frames: 7
  max_center_jump_px: 132
```

## 4. Running the tuned lane follower (sim or real)

In the simulator on the BrightCore track:

```bash
roslaunch turtlebot3_autorace_driving brightcore_track.launch \
    gui:=true run_lane_follower:=true rear_camera:=false \
    tuned_file:=$(rospack find turtlebot3_autorace_driving)/param/tuned_lane_follower.yaml
```

In any sim where you already have Gazebo running (front-mounted camera on
`/camera/image`):

```bash
roslaunch turtlebot3_autorace_driving turtlebot3_autorace_scanline_lane_following.launch \
    mode:=action \
    camera_topic:=/camera/image \
    camera_compressed_topic:=/camera/image/compressed \
    rear_camera:=false \
    tuned_file:=$(rospack find turtlebot3_autorace_driving)/param/tuned_lane_follower.yaml
```

On the real robot (camera bringup on the bot, lane follower on the PC):

```bash
roslaunch turtlebot3_autorace_driving turtlebot3_autorace_scanline_lane_following.launch \
    mode:=action \
    rear_camera:=true \
    tuned_file:=$(rospack find turtlebot3_autorace_driving)/param/tuned_lane_follower.yaml
```

## 5. Sim2Real calibration checklist

Real lighting, paint and perspective differ from the simulator. Before
trusting `tuned.yaml`, take 5 minutes to calibrate the yellow HSV mask and
the bird-eye projection on the actual robot:

1. Start the follower in calibration mode (it publishes debug images but
   does NOT drive):

   ```bash
   roslaunch turtlebot3_autorace_driving turtlebot3_autorace_scanline_lane_following.launch \
       mode:=calibration
   ```

2. Open rqt and watch `/lane/yellow_mask`. Edit
   `param/lane_follower_scanline.yaml` -> `yellow_lane.v_low` and
   `yellow_lane.s_low` until the mask is bright on the lane and dark
   everywhere else. The white mask is not used (and not published).

3. Watch `/lane/perspective_overlay`. The yellow trapezoid should sit on
   the ground and span both lane edges. Adjust
   `perspective.src_top_*` and `perspective.src_bottom_*` until the
   trapezoid hugs the lane. Save the file and re-launch.

4. Validate the finish marker by holding/printing the red shape and
   watching `/lane/red_mask`. The mask should light up cleanly. If not,
   reduce `red_marker.s_low` and `red_marker.v_low` until it does.

5. Run a full lap in `mode:=action`. If the robot under-turns in a
   specific section, the most useful per-section knobs to tweak by hand:

   - Hairpin (point 2 on the virtual map): bump
     `control.single_line_turn_gain` to 1.7-1.9.
   - S-loop around the centre island (points 5-6):
     `detection.max_center_jump_px` to 160-200 and
     `control.lost_decay` to 0.85.
   - Long straight before the finish: leave defaults.

## 6. What gets tuned vs. what is fixed

Tuned by Optuna (per trial):

- `pid.Kp`, `pid.Kd`
- `speed.base_linear`, `speed.max_angular`
- `control.single_line_turn_gain`, `control.lost_decay`
- `detection.max_lost_frames`, `detection.max_center_jump_px`

Domain-randomized on top of each trial (sim2real margin):

- `yellow_lane.v_low`, `yellow_lane.s_low`
- y-coordinate of `perspective.src_top_left` / `src_top_right`

Fixed (no tuning, intentionally):

- The lane color mode is always `yellow_only`. The white mask is not
  built, not published, and not consulted.
- Finish marker thresholds (calibrate visually if the red mark fails
  to be detected in real lighting).
- Camera intrinsic calibration (already shipped in
  `calibration/camera_calibration.yaml`).

## 7. State-machine `control_lane_param` + Optuna (AutoRace 2020)

This path uses the **forked** state-machine controller (`nodes/control_lane_param`)
with ROIs on bird-eye (`/camera/image_projected_compensated`), same stack as
`~/catkin_ws/src/turtlebot3_autorace_2020/launch_autorace.launch`, but with
tunable gains in `param/control_lane.yaml` and `tuned_file` overlay.

**Smoke test (manual, before tuning):** from a sourced workspace that has
`turtlebot3_gazebo`, `turtlebot3_autorace_camera`, `turtlebot3_autorace_detect`:

```bash
roslaunch turtlebot3_autorace_driving autorace_sim_with_param.launch gui:=true
```

Expect the burger to complete the AutoRace 2020 lap (like the original
`control_lane` chain).

**3-hour tuning (headless, roscore in another terminal):**

```bash
export TURTLEBOT3_MODEL=burger
SIM="$(rospack find turtlebot3_autorace_driving)/launch/autorace_sim_with_param.launch"

rosrun turtlebot3_autorace_driving tune_control_lane.py \
    --trials 150 --episode-timeout 75 \
    --sim-launch "$SIM" \
    --storage sqlite:///$HOME/cl_study.db \
    --study-name control_lane_tune
```

Best overlay is written to `param/tuned_control_lane.yaml`. Load it in sim:

```bash
roslaunch turtlebot3_autorace_driving autorace_sim_with_param.launch \
    tuned_file:=$(rospack find turtlebot3_autorace_driving)/param/tuned_control_lane.yaml
```

**Real robot (rear camera):** camera bridge + intrinsic + extrinsic + detect as in
`AGENTS.md`, then:

```bash
roslaunch turtlebot3_autorace_driving autorace_real.launch \
    tuned_file:=$(rospack find turtlebot3_autorace_driving)/param/tuned_control_lane.yaml
```

`autorace_real.launch` publishes commands to `/cmd_vel_raw` and `cmd_vel_invert`
mirrors signs onto `/cmd_vel` for TurtleBot hardware that expects inverted drive.
