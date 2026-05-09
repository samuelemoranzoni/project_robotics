# Safe Lane Selection with RoboMaster EP

Students: Samuele Moranzoni, Ferdinando Giordano.

This project implements an autonomous-driving inspired RoboMaster EP simulation in CoppeliaSim and ROS 2. The current milestone focuses on the first three intermediate steps of the project:

1. Ego-lane identification: given a multi-lane road, determine which lane the ego vehicle is currently occupying.
2. Orientation understanding: estimate whether the robot is aligned with the road direction using the lane geometry.
3. Camera-based object detection: detect red and green cars through the RoboMaster camera.



## Repository Layout

```text
project/
+-- README.md
+-- scenes/
|   +-- safe_lane_selection_scene.ttt
|   +-- safe_lane_selection_scene_with_red_and_green_cars.ttt
|   +-- README.md
+-- src/
    +-- safe_lane_selection/
        +-- config/
        |   +-- safe_lane_selection.yaml
        +-- launch/
        |   +-- rm_camera.launch.xml
        |   +-- safe_lane_controller.launch.py
        |   +-- safe_lane_full.launch.xml
        +-- safe_lane_selection/
        |   +-- perception.py
        |   +-- safe_lane_node.py
        +-- scripts/
            +-- add_camera_obstacle_test.lua
            +-- add_world_pose_publisher.lua
            +-- create_safe_lane_scene.lua
```

Main package:

```text
src/safe_lane_selection
```

The older `src/autonomous_guardian_drive` package is left in the workspace as previous experimental work, but the current project is `safe_lane_selection`.

## Current Features

Implemented and tested:

- Multi-lane road scene in CoppeliaSim.
- RoboMaster EP camera streaming through `robomaster_ros`.
- Ego-lane identification using the simulated world pose and visual lane detection.
- Orientation estimate from lane boundaries.
- Debug image overlay with lane, target, pose, command and object information.
- Detection of red and green cars from the camera image.
- ROS 2 status publication as JSON.
- Slow lane-following controller for intermediate testing.

Not fully validated yet:

- Automatic safe lane switching.
- Obstacle braking.
- CNN-based perception.
- ArUco-based object detection in Python. It is disabled because the current macOS/Pixi OpenCV build can segfault when using `cv2.aruco`.

## How It Works

The ROS node `safe_lane_node.py` subscribes to the RoboMaster camera and pose topics, runs perception, publishes debug/status topics, and sends velocity commands.

Lane perception in `perception.py`:

- Segments white/yellow lane paint in HSV.
- Uses Hough lines to extract lane boundaries.
- Clusters boundaries and estimates visible lane centers.
- Uses the simulated `/rm0/world_pose` topic to stabilize the physical ego-lane index.

Orientation:

- Estimates the lane boundary angle in the camera frame.
- Applies a fixed calibration offset configured in `safe_lane_selection.yaml`.
- Publishes both raw and corrected orientation values in the status topic.

Object detection:

- Detects only red and green cars.
- Masks white/yellow lane paint to reduce false positives.
- Ignores large side-border blobs and very high image regions.
- Publishes detected objects with lane assignment, confidence and risk score.

Obstacle braking is disabled for the current milestone:

```yaml
enable_obstacle_braking: false
```

This means objects are detected and displayed, but the robot does not stop because of them.

## Scene Setup

Open CoppeliaSim from the RoboMaster Pixi workspace:

```bash
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster
pixi shell
source install/setup.zsh
pixi run coppelia
```

Open the scene:

```text
/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project/scenes/safe_lane_selection_scene_with_red_and_green_cars.ttt
```

Before pressing Play:

- Enable Real-time mode in the CoppeliaSim Simulation menu.
- Make sure the RoboMaster object is named `/rm0`.
- Make sure the scene contains the world pose publisher script.


## Updating the Scene with Red and Green Cars

The easiest way to update the CoppeliaSim scene is through the Commander.

In CoppeliaSim:

```text
Modules > Developer tools > Commander > Commander
```

Run:

```lua
dofile('/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project/src/safe_lane_selection/scripts/add_camera_obstacle_test.lua')
```

This script:

- Does not move or rotate `/rm0`.
- Removes the previous gray/blue test objects.
- Adds one red car and one green car.
- Saves the scene automatically to:

```text
/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project/scenes/safe_lane_selection_scene.ttt
```

## Build

Use one terminal for building and launching ROS:

```bash
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster
pixi shell
source install/setup.zsh

cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/project
colcon build --symlink-install --packages-select safe_lane_selection
source install/setup.zsh
```

After every Python, YAML or launch-file change, rebuild and source again:

```bash
colcon build --symlink-install --packages-select safe_lane_selection
source install/setup.zsh
```

## Launch

Terminal 1: CoppeliaSim

```bash
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster
pixi shell
source install/setup.zsh
pixi run coppelia
```

Then open the scene, enable Real-time mode, and press Play.

Terminal 2: ROS 2 launch

```bash
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster
pixi shell
source install/setup.zsh

cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/project
source install/setup.zsh
export ROS_LOCALHOST_ONLY=1

ros2 launch safe_lane_selection safe_lane_full.launch.xml name:=rm0
```

Optional lateral velocity:

```bash
ros2 launch safe_lane_selection safe_lane_full.launch.xml name:=rm0 use_lateral_velocity:=true
```

Optional future safe-lane logic:

```bash
ros2 launch safe_lane_selection safe_lane_full.launch.xml name:=rm0 enable_safe_lane_selection:=true enable_obstacle_braking:=true
```

## Debug and Verification

Terminal 3: diagnostics

```bash
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster
pixi shell
source install/setup.zsh

cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/project
source install/setup.zsh
export ROS_LOCALHOST_ONLY=1
```

Check that the controller node is alive:

```bash
ros2 node list | grep safe_lane
```

Check camera stream:

```bash
ros2 topic hz /rm0/camera/image_color
```

Check velocity commands:

```bash
ros2 topic echo /rm0/cmd_vel --once
```

Check status:

```bash
ros2 topic echo /rm0/safe_lane/status --once
```

Open the debug image:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select:

```text
/rm0/safe_lane/debug_image
```

Useful output topics:

```text
/rm0/safe_lane/debug_image
/rm0/safe_lane/status
/rm0/safe_lane/ego_lane
/rm0/safe_lane/selected_lane
/rm0/cmd_vel
```

Useful input topics:

```text
/rm0/camera/image_color
/rm0/odom
/rm0/world_pose
```

## Configuration

Main configuration file:

```text
src/safe_lane_selection/config/safe_lane_selection.yaml
```

Important parameters:

```yaml
enable_object_detection: true
enable_aruco: false
enable_obstacle_braking: false
enable_safe_lane_selection: false
use_localization_ego_lane: true
use_world_lane_control: true
base_speed: 0.05
image_center_x: 0.60
orientation_offset_deg: 45.0
```

Lane numbering is from right to left:

```text
lane 1: rightmost
lane 2
lane 3
lane 4: leftmost
```

## Troubleshooting

### The robot does not move

First check if the controller node is alive:

```bash
ros2 node list | grep safe_lane
```

If nothing appears, restart the ROS launch. The launch process can stay alive even if `safe_lane_node` crashed.

```bash
pkill -f "ros2 launch safe_lane_selection" || true
pkill -f "safe_lane_node" || true
pkill -f "robomaster_driver" || true
ros2 daemon stop
```

Then rebuild and relaunch:

```bash
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/project
colcon build --symlink-install --packages-select safe_lane_selection
source install/setup.zsh
export ROS_LOCALHOST_ONLY=1
ros2 launch safe_lane_selection safe_lane_full.launch.xml name:=rm0
```

### Topics appear in `ros2 topic list`, but `ros2 topic echo` hangs

Use localhost discovery and reset the ROS daemon:

```bash
export ROS_LOCALHOST_ONLY=1
ros2 daemon stop
```

Make sure this variable is exported in every ROS terminal.

### `safe_lane_node` crashes with exit code -11

This was observed when `cv2.aruco` was enabled on the current macOS/Pixi OpenCV build.

Keep this disabled:

```yaml
enable_aruco: false
```

Then rebuild:

```bash
colcon build --symlink-install --packages-select safe_lane_selection
source install/setup.zsh
```

### CoppeliaSim reports `bind: Address already in use`

There is probably an old RoboMaster/CoppeliaSim process still running.

```bash
pkill -f "ros2 launch safe_lane_selection" || true
pkill -f "robomaster_driver" || true
pkill -f "coppeliaSim" || true
ros2 daemon stop
```

Then start CoppeliaSim again and press Play only once.

### RoboMaster connection drops after a few seconds

Enable Real-time mode in CoppeliaSim before pressing Play. Without real-time simulation, the simulated RoboMaster heartbeat can time out.

### The robot is rotated incorrectly or falls

Do not regenerate the full scene while the manually fixed `/rm0` pose is good. Prefer using:

```lua
dofile('/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project/src/safe_lane_selection/scripts/add_camera_obstacle_test.lua')
```

This updates the test cars without moving `/rm0`.

### Object detection shows false cars on the lane lines

The detector is designed for red/green cars only and masks white/yellow lane paint. If false positives appear after code changes:

```bash
colcon build --symlink-install --packages-select safe_lane_selection
source install/setup.zsh
```

Then restart the launch.

### No objects are detected

Check:

```bash
ros2 topic echo /rm0/safe_lane/status --once
```

The status should include:

```json
"object_detection_enabled": true
```

Also make sure the cars are red and/or green, not blue or gray.

## Publishing This Project to GitHub

This repository should include source code, launch/config files and scene files. It should not include generated folders such as `build/`, `install/` and `log/`.

Initialize Git:

```bash
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/project
git init
git branch -M main
git remote add origin https://github.com/samuelemoranzoni/project_robotics.git
```

Add files:

```bash
git add .
git status
```

Commit:

```bash
git commit -m "Add safe lane selection robotics project"
```

Push:

```bash
git push -u origin main
```

If the GitHub repository already contains files and Git refuses the push, pull first:

```bash
git pull --rebase origin main --allow-unrelated-histories
git push -u origin main
```

For later updates:

```bash
git add .
git commit -m "Update safe lane selection"
git push
```
