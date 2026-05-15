# Camera Configuration & Lane Detection Calibration Guide

Complete setup guide for the TurtleBot3 AutoRace 2020 camera pipeline — from raw Pi camera frames to a fully calibrated lane detector. Follow the stages in order; each one depends on the previous.

---

## 1. Architecture overview

The lane detector consumes a **bird's-eye-view** image that has been rectified, projected, and brightness-compensated. The full pipeline:

```
raspicam_node (on Pi)
        │  /raspicam_node/image/compressed
        ▼
autorace2020_raspicam_bridge       (relay: rename topics)
        │  /camera/rgb/image_raw/compressed
        ▼
intrinsic_camera_calibration       (lens distortion correction)
        │  /camera/image_rect_color/compressed
        ▼
image_compensation                 (brightness/contrast normalization)
        │  /camera/image_compensated/compressed
        ▼
image_projection                   (homography → bird's-eye view)
        │  /camera/image_projected/compressed
        ▼
image_compensation_projection      (post-projection brightness)
        │  /camera/image_projected_compensated/compressed
        ▼
detect_lane                        (HSV mask + sliding window)
        │  /detect/image_lane/compressed
        │  /detect/image_white_lane_marker/compressed
        │  /detect/image_yellow_lane_marker/compressed
        │  /detect/lane (Float64 — lane center pixel)
        ▼
control_lane                       (PID → /cmd_vel)
```

Each stage has a `mode:=calibration` flag that exposes parameters via **dynamic_reconfigure** and shows visual feedback. After tuning, the values must be copied into the corresponding YAML file in `param/` or `calibration/`.

---

## 2. ROS network setup

You need **one** `roscore`, with both PC and Pi pointing at it. Typical setup runs roscore on the PC.

### 2.1 Find IP addresses

On each machine:
```bash
hostname -I
```

Note both addresses (e.g. PC `192.168.0.10`, Pi `192.168.0.20`).

### 2.2 Configure environment variables

Add to `~/.bashrc` on **PC**:
```bash
export ROS_MASTER_URI=http://192.168.0.10:11311
export ROS_HOSTNAME=192.168.0.10
```

Add to `~/.bashrc` on **Pi**:
```bash
export ROS_MASTER_URI=http://192.168.0.10:11311
export ROS_HOSTNAME=192.168.0.20
```

Then `source ~/.bashrc` on both, or open new terminals.

### 2.3 Verify connectivity

From PC:
```bash
ping 192.168.0.20      # should respond
ssh pi@192.168.0.20    # should let you in
```

From Pi (after starting roscore on PC):
```bash
rostopic list          # should show /rosout — proves it sees the master
```

If `rostopic list` hangs on the Pi, your `ROS_MASTER_URI` / `ROS_HOSTNAME` is wrong or a firewall is blocking port 11311.

---

## 3. What runs on the Robot (Pi)

SSH into the Pi from your PC, then run each command in a separate terminal (or `tmux` / `screen` session).

### Terminal P1 — Robot bringup

Starts motors, IMU, LDS, TF tree:
```bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

### Terminal P2 — Camera node

The standard TurtleBot3 AutoRace 2020 uses `raspicam_node`:
```bash
roslaunch turtlebot3_autorace_camera turtlebot3_autorace_camera_pi.launch
```

If that launch file does not exist on your Pi, fall back to:
```bash
roslaunch raspicam_node camerav2_410x308_30fps.launch
```

This publishes `/raspicam_node/image/compressed`. Verify from the PC:
```bash
rostopic hz /raspicam_node/image/compressed
```

Expected: ~30 Hz.

---

## 4. What runs on the PC

Each block runs in its own terminal. Always `source ~/catkin_ws/devel/setup.bash` first.

### Terminal 1 — roscore
```bash
roscore
```

### Terminal 2 — Topic bridge

The Pi publishes `/raspicam_node/image/...` but this codebase expects `/camera/rgb/image_raw/...`. The bridge relays the names:
```bash
roslaunch autorace_lane autorace2020_raspicam_bridge.launch
```

Verify:
```bash
rostopic hz /camera/rgb/image_raw/compressed
```

### Terminal 3 — Intrinsic rectification

Removes lens distortion. Requires `camera_info` with the camera matrix and distortion coefficients (produced once by `camera_calibration`'s `cameracalibrator.py` checkerboard procedure):
```bash
roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch
```

Publishes `/camera/image_rect_color`. Verify:
```bash
rostopic hz /camera/image_rect_color/compressed
```

If you have **never** done intrinsic calibration on this exact camera, do it first (see §6).

### Terminal 4 — Extrinsic projection + compensation

Reprojects the front-camera view into a top-down (bird's-eye) view and normalizes brightness:
```bash
roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch
```

In **action mode** (default) this publishes `/camera/image_projected_compensated`. In **calibration mode** it also exposes ROI parameters via dynamic_reconfigure:
```bash
roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=calibration
```

Verify:
```bash
rostopic hz /camera/image_projected_compensated/compressed
```

### Terminal 5 — Lane detection

```bash
roslaunch turtlebot3_autorace_detect detect_lane.launch mode:=calibration
```

Publishes:
- `/detect/image_white_lane_marker/compressed` — white-line HSV mask
- `/detect/image_yellow_lane_marker/compressed` — yellow-line HSV mask
- `/detect/image_lane/compressed` — final overlay with detected polynomial
- `/detect/lane` — lane center pixel (consumed by control_lane)

### Terminal 6 — rqt (visualization + tuning)
```bash
rqt
```

Then add the plugins below in §7.

---

## 5. Verification chain

After each launch, run the corresponding `rostopic hz`. If any line shows 0 Hz, **stop and fix that stage before continuing**.

| Stage | Topic to check | Expected |
|---|---|---|
| Pi camera | `/raspicam_node/image/compressed` | ~30 Hz |
| Bridge | `/camera/rgb/image_raw/compressed` | ~30 Hz |
| Intrinsic | `/camera/image_rect_color/compressed` | ~30 Hz |
| Extrinsic | `/camera/image_projected_compensated/compressed` | ~30 Hz |
| Detect | `/detect/image_white_lane_marker/compressed` | ~30 Hz |

Useful diagnostic commands:
```bash
rosnode list                                          # what's running
rostopic info /camera/image_projected_compensated     # who publishes / subscribes
rqt_graph                                             # visual node/topic graph
rosnode info /detect_lane                             # detect_lane's pubs & subs
```

If a topic has `Publishers: None` but `Subscribers: /detect_lane`, the topic only "exists" because someone is listening — nobody is actually publishing. That points you at the missing upstream launch.

---

## 6. Intrinsic calibration (one-time per camera)

Skip this section if your camera already has a valid `camera_info` published.

### 6.1 Print a checkerboard

Use the included pattern at [turtlebot3_autorace_camera/data/checkerboard.pdf](turtlebot3_autorace_camera/data/checkerboard.pdf) (or any 8×6 inner-corner 25 mm board). Mount it flat on a rigid surface.

### 6.2 Run the calibrator

With the Pi camera streaming:
```bash
rosrun camera_calibration cameracalibrator.py \
    --size 8x6 --square 0.025 \
    image:=/camera/rgb/image_raw camera:=/camera/rgb
```

Move the checkerboard until **X/Y/Size/Skew** bars are all green, then click **CALIBRATE** → **COMMIT**.

This writes a `camera.yaml` into `~/.ros/camera_info/`. Move it to the Pi's `raspicam_node` config path so future runs pick it up automatically:
```bash
scp ~/.ros/camera_info/head_camera.yaml pi@<pi-ip>:~/.ros/camera_info/
```

---

## 7. Extrinsic calibration (bird's-eye-view ROI)

Launch in calibration mode:
```bash
roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=calibration
```

### 7.1 Open rqt plugins

In rqt:
- **Plugins → Visualization → Image View** → open **two** panels
  - Panel A: `/camera/image_extrinsic_calib/compressed` (shows ROI trapezoid overlay)
  - Panel B: `/camera/image_projected_compensated/compressed` (shows the warped result)
- **Plugins → Configuration → Dynamic Reconfigure** → select `/image_projection`

### 7.2 Tune the ROI

Four sliders define the trapezoid that gets warped to a rectangle:

| Param | Meaning |
|---|---|
| `top_x` | Half-width of the trapezoid's **top** edge (further from camera) |
| `top_y` | Vertical offset of the top edge from image center |
| `bottom_x` | Half-width of the trapezoid's **bottom** edge (closer to camera) |
| `bottom_y` | Vertical offset of the bottom edge from image center |

Goal: place the robot on a known straight lane segment. Adjust sliders until:
- The blue trapezoid in Panel A covers exactly the **lane region** in front of the robot.
- In Panel B, the lane lines appear **straight and parallel** (vertical), not bowed or skewed.

Current defaults from [turtlebot3_autorace_camera/calibration/extrinsic_calibration/projection.yaml](turtlebot3_autorace_camera/calibration/extrinsic_calibration/projection.yaml):
```yaml
top_x: 72
top_y: 4
bottom_x: 115
bottom_y: 120
```

### 7.3 Save values

Copy the final slider values into [projection.yaml](turtlebot3_autorace_camera/calibration/extrinsic_calibration/projection.yaml). Dynamic reconfigure does **not** persist — values are lost on relaunch.

### 7.4 Brightness compensation (optional)

[compensation.yaml](turtlebot3_autorace_camera/calibration/extrinsic_calibration/compensation.yaml) has `clip_hist_percent` (default `1.0`). Increase if the bird's-eye image is too dark, decrease if washed out. Usually the default is fine.

---

## 8. Lane HSV calibration (the rqt step you asked about)

Launch:
```bash
roslaunch turtlebot3_autorace_detect detect_lane.launch mode:=calibration
```

### 8.1 Open rqt plugins

In rqt:
- **Dynamic Reconfigure** → select `/detect_lane`
- **Image View** × 3 panels:
  - Panel A: `/detect/image_white_lane_marker` (white mask)
  - Panel B: `/detect/image_yellow_lane_marker` (yellow mask)
  - Panel C: `/detect/image_lane` (final detection)

**Tip:** in rqt's Image View, enter the **base topic name without `/compressed`** — image_transport auto-selects the compressed stream. Typing the full `/compressed` path may show blank.

### 8.2 The 12 sliders

Six per color (white, yellow):

| Slider | Range | Meaning |
|---|---|---|
| `hue_l` | 0–179 | Lower hue bound |
| `hue_h` | 0–179 | Upper hue bound |
| `saturation_l` | 0–255 | Lower saturation bound |
| `saturation_h` | 0–255 | Upper saturation bound |
| `lightness_l` | 0–255 | Lower lightness bound (a.k.a. value) |
| `lightness_h` | 0–255 | Upper lightness bound |

**Goal:** the mask panel should show the lane line as **solid white pixels** with as little background noise as possible.

### 8.3 Tuning recipe

**White line** (white = any hue, low saturation, high lightness):
1. Set `hue_l = 0`, `hue_h = 179` (accept all hues).
2. Set `saturation_l = 0`. Lower `saturation_h` until shiny/glare areas drop out (~50–100).
3. Raise `lightness_l` until the floor/background disappears but the line stays (~100–180).
4. Keep `lightness_h = 255`.

**Yellow line** (yellow has a specific hue):
1. `hue_l ≈ 10`, `hue_h ≈ 35` (OpenCV HSV yellow band).
2. Raise `saturation_l` to ~70–120 to reject pale/grey surfaces.
3. `saturation_h = 255`.
4. Raise `lightness_l` to ~80–120 to reject shadows.
5. `lightness_h = 255`.

Move the robot left/right by a few cm and re-check — the mask should stay stable under small viewpoint changes. Walk around the track if possible to expose the calibration to different lighting.

Current defaults from [turtlebot3_autorace_detect/param/lane/lane.yaml](turtlebot3_autorace_detect/param/lane/lane.yaml):
```yaml
white:  { hue_l: 0,  hue_h: 179, saturation_l: 0,  saturation_h: 70,  lightness_l: 105, lightness_h: 255 }
yellow: { hue_l: 10, hue_h: 127, saturation_l: 70, saturation_h: 255, lightness_l: 95,  lightness_h: 255 }
```

### 8.4 Save values

Copy slider values into [lane.yaml](turtlebot3_autorace_detect/param/lane/lane.yaml) and restart `detect_lane`.

---

## 9. Switching to action mode

Once everything is calibrated, relaunch without the calibration flag — the nodes will load YAML values and run silently:

```bash
roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch
roslaunch turtlebot3_autorace_detect detect_lane.launch
roslaunch turtlebot3_autorace_driving turtlebot3_autorace_control_lane.launch
```

The driver consumes `/detect/lane` and publishes `/cmd_vel`. Robot starts following the lane.

---

## 10. Troubleshooting

### Symptom: rqt Image View shows grey gradient

**Cause:** detect_lane is running but receiving no upstream frames. The "grey gradient" is the empty default image.

**Fix:** run §5 verification chain top-to-bottom. The first 0 Hz topic tells you which stage isn't running.

### Symptom: `rostopic info /X` shows `Publishers: None`

The topic name only exists because a subscriber is listening. Nothing publishes it — start the missing upstream launch.

### Symptom: Bird's-eye view is black or distorted

- ROI sliders are wrong → re-do §7.
- Intrinsic calibration is missing → check `rostopic echo /camera/camera_info` returns sane non-zero K matrix.

### Symptom: White mask catches yellow line (or vice versa)

Saturation thresholds overlap. Raise yellow's `saturation_l` and lower white's `saturation_h`. White lines should have near-zero saturation; yellow lines high saturation.

### Symptom: Detection flickers between frames

Lighting is borderline for current thresholds. Widen `lightness_l` window slightly, or improve track lighting. Check `/detect/white_line_reliability` and `/detect/yellow_line_reliability` — values <80 mean detection is unstable.

### Symptom: ROS topics flow on PC but not Pi (or vice versa)

Network issue. On both machines:
```bash
echo $ROS_MASTER_URI
echo $ROS_HOSTNAME
```

Both must point at the **same** master. `ROS_HOSTNAME` must be the **local** machine's IP. Both must resolve via ping.

### Symptom: `[X.launch] is neither a launch file in package [Y]…`

The launch file doesn't exist in that package. List what's actually there:
```bash
ls $(rospack find <package_name>)/launch/
```

---

## 11. File reference

| Purpose | File |
|---|---|
| Topic bridge (Pi → autorace names) | [autorace_lane/launch/autorace2020_raspicam_bridge.launch](../autorace_lane1/autorace_lane/launch/autorace2020_raspicam_bridge.launch) |
| Intrinsic rectification | [turtlebot3_autorace_camera/launch/intrinsic_camera_calibration.launch](turtlebot3_autorace_camera/launch/intrinsic_camera_calibration.launch) |
| Extrinsic projection | [turtlebot3_autorace_camera/launch/extrinsic_camera_calibration.launch](turtlebot3_autorace_camera/launch/extrinsic_camera_calibration.launch) |
| Projection ROI values | [turtlebot3_autorace_camera/calibration/extrinsic_calibration/projection.yaml](turtlebot3_autorace_camera/calibration/extrinsic_calibration/projection.yaml) |
| Brightness compensation values | [turtlebot3_autorace_camera/calibration/extrinsic_calibration/compensation.yaml](turtlebot3_autorace_camera/calibration/extrinsic_calibration/compensation.yaml) |
| Lane detection launch | [turtlebot3_autorace_detect/launch/detect_lane.launch](turtlebot3_autorace_detect/launch/detect_lane.launch) |
| Lane HSV thresholds | [turtlebot3_autorace_detect/param/lane/lane.yaml](turtlebot3_autorace_detect/param/lane/lane.yaml) |
| Lane detector source | [turtlebot3_autorace_detect/nodes/detect_lane](turtlebot3_autorace_detect/nodes/detect_lane) |
| Driving controller | [turtlebot3_autorace_driving/nodes/control_lane](turtlebot3_autorace_driving/nodes/control_lane) |

---

## 12. Quick checklist

- [ ] Pi and PC see each other (`ping`, `ssh`, matching `ROS_MASTER_URI`)
- [ ] `roscore` running on PC
- [ ] Pi: `turtlebot3_robot.launch` + camera node up
- [ ] Topic bridge running on PC → `/camera/rgb/image_raw/compressed` at ~30 Hz
- [ ] Intrinsic calibration done & launch running → `/camera/image_rect_color/compressed` at ~30 Hz
- [ ] Extrinsic ROI tuned & saved → `/camera/image_projected_compensated/compressed` at ~30 Hz
- [ ] HSV thresholds tuned & saved → `/detect/image_*_marker/compressed` at ~30 Hz
- [ ] Lane detection overlay looks correct on `/detect/image_lane`
- [ ] Switch all launches to `mode:=action` for driving
