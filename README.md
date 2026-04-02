# BCR Arm

https://github.com/user-attachments/assets/3a08bc22-2d3c-4b57-9e81-967f29c440c9

Gazebo and ROS 2 simulation for a 7-DOF robotic arm, with MoveIt support and a browser-based control UI.

## Browser UI

The working browser UI lives in [`browser_ui/`](browser_ui) and runs as a Vite app in front of the ROS bridge backend.

GitHub cannot run the interactive app directly inside the README, but this repository now documents the real startup path for the UI that exists in the repo.

### Quick Start

Start the full browser stack:

```bash
./scripts/run_browser_full.sh
```

Then open:

```text
http://127.0.0.1:5176
```

That script starts:

- Gazebo software stack
- ROS browser bridge backend on `127.0.0.1:8080`
- Vite browser UI on `127.0.0.1:5176`

Run it detached if you want the services to keep running after you close the terminal:

```bash
./scripts/run_browser_full.sh --detach
```

Detached mode writes logs to:

- `/tmp/bcr_browser_gazebo.log`
- `/tmp/bcr_browser_bridge.log`
- `/tmp/bcr_browser_ui.log`

### Browser UI Only

If Gazebo is already running and you only want the browser control layer:

```bash
./scripts/run_browser_app.sh
```

Detached:

```bash
./scripts/run_browser_app.sh --detach
```

Or run the pieces separately:

```bash
./scripts/run_browser_bridge.sh --host 127.0.0.1 --port 8080
./scripts/run_browser_vite.sh --host 127.0.0.1 --port 5176 --api-url http://127.0.0.1:8080
```

### No-Backend Demo Mode (GitHub Pages-Friendly)

You can run the browser UI without a ROS bridge backend and still show the command/sensor flow.

Set:

```bash
cd browser_ui
VITE_DEMO_NO_BACKEND=true npm run dev
```

Or for a production/static build:

```bash
cd browser_ui
VITE_DEMO_NO_BACKEND=true npm run build
```

In this mode, the Status panel explicitly reports that backend calls are suppressed and sensor data is simulated.

### Publish `browser_ui` On GitHub Pages

This repository includes a GitHub Actions workflow at `.github/workflows/pages.yml` that:

- Builds `browser_ui/` with `VITE_DEMO_NO_BACKEND=true`
- Sets the correct Vite base path for user/org pages vs project pages
- Deploys `browser_ui/dist` to GitHub Pages

To enable:

1. Push changes to `main` (or run the workflow manually from the Actions tab).
2. In GitHub: `Settings -> Pages -> Build and deployment -> Source = GitHub Actions`.
3. Wait for the `Deploy Browser UI To Pages` workflow to finish.

URL:

- User/org site repo (`<user>.github.io`): `https://<user>.github.io/`
- Project repo (`bcr_arm`): `https://<user>.github.io/bcr_arm/`

### What The UI Includes

- 3D browser viewer for the arm and gantry cell
- Preset poses
- Per-joint manual controls
- Direct joint command entry
- Motion sequence buttons
- Scene item placement/removal
- Live status polling through the bridge API

## About

This repository currently supports:

1. [ROS 2 Humble + Gazebo Fortress (Ubuntu 22.04)](#ros-2-humble--gazebo-fortress-ubuntu-2204)
2. [ROS 2 Jazzy + Gazebo Harmonic (Ubuntu 24.04)](#ros-2-jazzy--gazebo-harmonic-ubuntu-2404)
3. [NVIDIA Isaac Sim](#nvidia-isaac-sim)

## ROS 2 Humble + Gazebo Fortress (Ubuntu 22.04)

### Dependencies

Ensure you have ROS 2 Humble and Gazebo Fortress installed.

```bash
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y gz-fortress
```

Install remaining dependencies with `rosdep` from the root of your workspace:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Source Build

```bash
colcon build --symlink-install
source install/setup.bash
```

### Binary Install

```bash
sudo apt-get install ros-humble-bcr-arm
```

### Launch Files

Gazebo + ROS 2 Control + MoveIt 2:

```bash
ros2 launch bcr_arm_moveit_config bcr_arm_moveit_gazebo.launch.py
```

Gazebo + ROS 2 Control:

```bash
ros2 launch bcr_arm_gazebo bcr_arm.gazebo.launch.py
```

Notes:

- Uses ROS 2 mock controllers for the arm.
- Supports `world_path:=<path_to_world>`.

## ROS 2 Jazzy + Gazebo Harmonic (Ubuntu 24.04)

### Dependencies

Ensure you have ROS 2 Jazzy and Gazebo Harmonic installed.

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop
sudo apt install -y gz-harmonic
```

Build `topic_based_ros2_control` from source if you need MoveIt with Isaac Sim:

```bash
git clone https://github.com/PickNikRobotics/topic_based_ros2_control.git
cd topic_based_ros2_control
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --event-handlers log-
source install/setup.bash
```

Install remaining dependencies with `rosdep`:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the project:

```bash
colcon build --symlink-install
source install/setup.bash
```

### Launch Files

Gazebo + ROS 2 Control + MoveIt 2:

```bash
ros2 launch bcr_arm_moveit_config bcr_arm_moveit_gazebo.launch.py
```

Gazebo + ROS 2 Control:

```bash
ros2 launch bcr_arm_gazebo bcr_arm.gazebo.launch.py
```

Notes:

- Uses ROS 2 mock controllers for the arm.
- Supports `world_path:=<path_to_world>`.

## NVIDIA Isaac Sim

### Dependencies

Ensure NVIDIA Isaac Sim is installed.

- Download Isaac Sim from [NVIDIA](https://developer.nvidia.com/isaac-sim)
- Launch Isaac Sim
- Load [`isaacsim/bcr_arm_scene.usd`](isaacsim/bcr_arm_scene.usd)

### Launch

Run the robot in Isaac Sim, then start MoveIt:

```bash
ros2 launch bcr_arm_moveit_config isaac_demo.launch.py
```

## Controlling The Arm

Once the simulation with controllers is running, you can send commands through the CLI:

```bash
ros2 run bcr_arm_gazebo control_arm_cli.py
```

## Images

![Gazebo BCR Arm simulation with MoveIt 2](images/gz_img1.png)

![Isaac Sim BCR Arm simulation with MoveIt 2](images/isaac_img2.png)

![Jazzy BCR Arm simulation](images/jazzy_img3.png)
