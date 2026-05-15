# waypoint-drone-control

Waypoint and curve-following drone control simulation using ROS 2, MAVROS, PX4 SITL, Gazebo, and QGroundControl.

This project controls a PX4 Iris drone in Gazebo using waypoint-based and curve-following flight scripts through ROS 2 and MAVROS.

---

## Project Overview

This project implements multiple autonomous drone flight patterns in a PX4/Gazebo simulation environment.

The drone receives velocity commands from Python ROS 2 nodes.  
Each node subscribes to the drone's local position and publishes velocity commands to MAVROS.  
MAVROS sends those commands to PX4 in OFFBOARD mode, and PX4 controls the Iris drone inside Gazebo.

---

## Tech Stack

- Python
- ROS 2 Humble
- MAVROS
- PX4 SITL
- Gazebo
- QGroundControl
- geometry_msgs/TwistStamped
- geometry_msgs/PoseStamped

---

## Control Pipeline

```text
Python Waypoint / Curve Node
      ↓
/mavros/local_position/pose
      ↓
Position Feedback
      ↓
P-Control Velocity Command
      ↓
/mavros/setpoint_velocity/cmd_vel
      ↓
MAVROS
      ↓
PX4 OFFBOARD Mode
      ↓
Gazebo Iris Drone
```

---

## Flight Scripts

| File | Description |
|---|---|
| `square_waypoint_drone.py` | Follows square-shaped waypoints and returns to the origin. |
| `circle_waypoint_drone.py` | Generates many waypoints on a circle and follows them sequentially. |
| `true_circle_waypoint_drone.py` | Performs continuous circular flight using tangential velocity and radial correction. |
| `curve_follower_drone.py` | Follows a parametric curve using position feedback control. |

---

## How to Run

The overall execution order is:

1. Start PX4 SITL with Gazebo
2. Start QGroundControl
3. Start MAVROS
4. Set MAVROS velocity coordinate frame
5. Run one Python flight script
6. Switch PX4 to OFFBOARD mode
7. Arm the drone
8. Check local position feedback

---

## Terminal 1: Start Gazebo Iris Drone

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

This command starts PX4 SITL and launches the Gazebo Iris drone simulation.

---

## Terminal 2: Start QGroundControl

```bash
./QGroundControl-x86_64.AppImage
```

If the command does not work, check the actual path of the QGroundControl AppImage file.

Example:

```bash
cd ~/Downloads
./QGroundControl-x86_64.AppImage
```

QGroundControl is used to monitor PX4 status, flight mode, arming state, and vehicle behavior.

---

## Terminal 3: Start MAVROS

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557
```

MAVROS connects ROS 2 and PX4 through UDP communication.

---

## Terminal 4: Set Coordinate Frame

```bash
source /opt/ros/humble/setup.bash
ros2 param set /mavros/setpoint_velocity mav_frame LOCAL_NED
```

This sets the MAVROS velocity command frame to `LOCAL_NED`.

In `LOCAL_NED` mode, velocity commands are interpreted based on the local world coordinate frame.

---

## Terminal 5: Run Python Waypoint Script

Move to the waypoint drone project directory.

```bash
cd ~/Desktop/study/waypoint_drone
```

Run only one script at a time.

### Square Waypoint Flight

```bash
python3 square_waypoint_drone.py
```

### Circle Waypoint Flight

```bash
python3 circle_waypoint_drone.py
```

### True Circle Flight

```bash
python3 true_circle_waypoint_drone.py
```

### Curve Follower Flight

```bash
python3 curve_follower_drone.py
```

---

## Terminal 6: Set PX4 OFFBOARD Mode and Arm

```bash
source /opt/ros/humble/setup.bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

OFFBOARD mode allows PX4 to receive external control commands from ROS 2.

The Python waypoint script should be running before switching to OFFBOARD mode.

---

## Terminal 7: Check Drone Position

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /mavros/local_position/pose
```

This command prints the current local position and orientation of the drone.

---

## Full Execution Summary

The commands below summarize the execution flow. Some commands must be executed in separate terminals.

```bash
# 1. Start Gazebo Iris Drone
cd ~/PX4-Autopilot
make px4_sitl gazebo

# 2. Start QGroundControl
./QGroundControl-x86_64.AppImage

# 3. Start MAVROS
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557

# 4. Set coordinate frame
source /opt/ros/humble/setup.bash
ros2 param set /mavros/setpoint_velocity mav_frame LOCAL_NED

# 5. Run one Python flight script
cd ~/Desktop/study/waypoint_drone

python3 square_waypoint_drone.py
# or
python3 circle_waypoint_drone.py
# or
python3 true_circle_waypoint_drone.py
# or
python3 curve_follower_drone.py

# 6. Set PX4 OFFBOARD mode
source /opt/ros/humble/setup.bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"

# 7. Arm the drone
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# 8. Check local position
ros2 topic echo /mavros/local_position/pose
```

---

## Notes

- PX4 and Gazebo must be running before starting MAVROS.
- QGroundControl can be used to monitor PX4 status and flight mode.
- Only one Python flight script should be executed at a time.
- The Python waypoint script should be running before switching to OFFBOARD mode.
- The drone will not move unless PX4 is in OFFBOARD mode and armed.
- The coordinate frame is set to `LOCAL_NED`, so movement commands are based on the local coordinate frame.
- The Python scripts use `/mavros/local_position/pose` as position feedback.
- The Python scripts publish velocity commands to `/mavros/setpoint_velocity/cmd_vel`.

---

## Future Improvements

- Add automatic takeoff and landing sequence
- Add waypoint visualization
- Add CSV-based waypoint loading
- Add A* path planning integration
- Add obstacle avoidance
- Add 3D path tracking
- Improve trajectory smoothing
