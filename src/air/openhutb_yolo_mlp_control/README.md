# OpenHUTB YOLO + MLP ROS 2 package

This package replaces direct Python-to-Python data flow with ROS 2 messages.
Only `airsim_bridge` calls the AirSim API.

## Node graph

```text
OpenHUTB / AirSim
      ^   |
      |   +--> /openhutb/camera/rgb (sensor_msgs/Image) --> yolo_detector
      |   +--> /openhutb/odom       (nav_msgs/Odometry) --> mlp_controller
      |
      +------ /openhutb/cmd_vel     (geometry_msgs/Twist) <-- mlp_controller

trajectory_target
      +------ /openhutb/target      (geometry_msgs/PointStamped) --> mlp_controller

yolo_detector
      +------ /openhutb/detections  (vision_msgs/Detection2DArray)
      +------ /openhutb/yolo/image  (sensor_msgs/Image)
```

## Build

Use the Python environment that belongs to your ROS 2 installation. Do not assume
an arbitrary Conda environment can import `rclpy`; the Python ABI must match ROS 2.
Install the non-ROS Python dependencies into that compatible environment:

```bash
python -m pip install -r requirements.txt
```

Install ROS message dependencies using your ROS 2 distribution package manager,
including `vision_msgs`.

From the ROS 2 workspace root:

```bash
colcon build --packages-select openhutb_yolo_mlp_control --symlink-install
```

Then source the workspace (`install/setup.bash` on Linux or `install\\setup.bat` on Windows).

## Perception

Start OpenHUTB in AIR mode first, then run the following command on the same
machine. If ROS 2 runs in WSL2/Linux and OpenHUTB runs on Windows, replace
`airsim_ip` with the Windows host address.

```bash
ros2 launch openhutb_yolo_mlp_control perception.launch.py \
  airsim_ip:=127.0.0.1 \
  model_path:=/absolute/path/to/yolo11n.pt \
  show_window:=true
```

For a headless Linux/WSL2 session, use `show_window:=false`. The YOLO model
`yolo11n.pt` is not included in the repository; download it separately and
pass its local path with `model_path`.

Inspect topics:

```bash
ros2 topic list
ros2 topic echo /openhutb/detections
```

## MLP trajectory control

Generate data and train the model first:

```bash
ros2 run openhutb_yolo_mlp_control generate_training_data
ros2 run openhutb_yolo_mlp_control train_mlp
```

Run a trajectory:

```bash
ros2 launch openhutb_yolo_mlp_control trajectory_control.launch.py \
  airsim_ip:=127.0.0.1 \
  auto_takeoff:=false \
  model_path:=/absolute/path/to/mlp_controller.pth \
  figure8_points:=160
```

The controller now consumes `nav_msgs/Odometry` and `geometry_msgs/PointStamped`,
and publishes `geometry_msgs/Twist`; it does not call AirSim directly.

## Coordinate convention

To preserve compatibility with the already-trained controller, `/openhutb/odom`,
`/openhutb/target`, and `/openhutb/cmd_vel` use AirSim NED coordinates. The odometry
frame is explicitly named `airsim_ned`.

## OpenHUTB AirSim PythonClient vendoring

Do **not** install the legacy PyPI `airsim` / `msgpack-rpc-python` stack. The
package contains the compatible AirSim client under `vendor/`. On Windows,
the vendor refresh helper can be run with:

```powershell
cd D:\openhutb_ros_ws\src\openhutb_yolo_mlp_control
powershell -ExecutionPolicy Bypass -File .\tools\vendor_openhutb_pythonclient.ps1 `
  -OpenHutbRoot "D:\无人机\hutb_windows_v2.10.0"
```

This creates:

```text
openhutb_yolo_mlp_control/vendor/airsim/
```

and rewrites only the legacy `import msgpackrpc` statements to use the local
`msgpackrpc_compat.py` client.  The rest of the AirSim PythonClient is copied
from the user's OpenHUTB installation.

The ROS environment only needs compatible MessagePack/NumPy for the bridge:

```powershell
cd C:\ros2_jazzy
pixi shell
pixi add numpy msgpack
```

On Ubuntu 22.04 use ROS 2 Humble; on Ubuntu 24.04 use ROS 2 Jazzy. Then build
the workspace and run perception:

```powershell
cd D:\openhutb_ros_ws
colcon build --packages-select openhutb_yolo_mlp_control --symlink-install
.\install\local_setup.ps1
ros2 launch openhutb_yolo_mlp_control perception.launch.py
```

For WSL2 with OpenHUTB running on Windows:

```bash
WINDOWS_HOST=$(ip route show default | awk '/default/ {print $3}')
ros2 launch openhutb_yolo_mlp_control perception.launch.py \
  airsim_ip:="$WINDOWS_HOST" \
  airsim_port:=41451 \
  model_path:="$HOME/models/yolo11n.pt" \
  show_window:=true
```
