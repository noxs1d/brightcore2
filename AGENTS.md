# Project Session Notes

Always use Russian when helping with this workspace unless the user asks otherwise.

## ROS Network

The robot is normally reachable at:

```bash
export ROBOT_IP=10.250.166.36
export PC_IP=10.250.166.52
export ROS_MASTER_URI=http://10.250.166.36:11311
export ROS_HOSTNAME=10.250.166.52
export TURTLEBOT3_MODEL=burger
```

On every PC terminal for this project, start with:

```bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.250.166.36:11311
export ROS_HOSTNAME=10.250.166.52
export TURTLEBOT3_MODEL=burger
```

On every robot SSH terminal, start with:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/tb3_run/robot_env.sh
export TURTLEBOT3_MODEL=burger
```

If the PC IP changes, run `hostname -I` on the PC and replace `ROS_HOSTNAME`.

## Robot Bringup Without Lidar

The lidar is not used. Do not use `turtlebot3_robot.launch` for driving, because it tries to start the lidar.

Use only the core for wheel control:

```bash
roslaunch turtlebot3_bringup turtlebot3_core.launch
```

The robot can drive only when `/cmd_vel` has a subscriber from `/turtlebot3_core`:

```bash
rostopic info /cmd_vel
```

Expected:

```text
Publishers:
 * /control_lane

Subscribers:
 * /turtlebot3_core
```

## Camera And Lane Pipeline

Robot camera:

```bash
roslaunch raspicam_node camerav2_410x308_30fps.launch
```

PC bridge:

```bash
roslaunch autorace_lane autorace2020_raspicam_bridge.launch
```

If `roslaunch autorace_lane autorace2020_raspicam_bridge.launch` fails, check:

```bash
rospack find autorace_lane
find ~/catkin_ws/src -name autorace2020_raspicam_bridge.launch -print
```

The bridge launch should relay:

```text
/raspicam_node/image/compressed -> /camera/rgb/image_raw/compressed
/raspicam_node/camera_info      -> /camera/rgb/camera_info
```

PC intrinsic:

```bash
roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch
```

PC republisher needed by this current pipeline:

```bash
rosrun image_transport republish raw compressed \
  in:=/camera/image \
  out:=/camera/image_rect_color
```

PC projection/action:

```bash
roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=action
```

PC lane detection/action:

```bash
roslaunch turtlebot3_autorace_detect detect_lane.launch mode:=action
```

PC lane control:

```bash
roslaunch turtlebot3_autorace_driving turtlebot3_autorace_control_lane.launch \
  center_ref:=525.0 kp:=0.0018 kd:=0.0050
```

## Preferred New Lane Follower

A newer scanline-based follower has been added from the `autorace_lane` algorithm. Prefer this for driving instead of the old `detect_lane + control_lane` chain.

It subscribes directly to:

```text
/raspicam_node/image/compressed
```

and publishes directly to:

```text
/cmd_vel
```

So do not run `detect_lane`, `control_lane`, `extrinsic_camera_calibration.launch`, or the camera republishers at the same time unless debugging.

Run it on the PC with:

```bash
roslaunch turtlebot3_autorace_driving turtlebot3_autorace_scanline_lane_following.launch mode:=action
```

For debugging masks/images:

```bash
roslaunch turtlebot3_autorace_driving turtlebot3_autorace_scanline_lane_following.launch mode:=calibration
```

Useful debug topics:

```text
/lane/yellow_mask
/lane/bird_eye
/lane/debug_image
/lane_error
```

Emergency stop:

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}'
```

## Calibration Files

Projection calibration:

```text
turtlebot3_autorace_camera/calibration/extrinsic_calibration/projection.yaml
```

Lane HSV calibration:

```text
turtlebot3_autorace_detect/param/lane/lane.yaml
```

Do not use `rosrun dynamic_reconfigure dynparam dump ...` directly on these files; it can write Python-object YAML that `roslaunch` cannot load. Save values manually in the normal ROS YAML structure.
