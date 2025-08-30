# Franka Applications: ROS2-based Drawing & Pick-and-Place

This repository contains ROS2 packages for controlling the Franka Research 3 (FR3) robot arm for drawing (raster/vector), pick-and-place, and gripper proxy operations. It includes MoveIt2 integration, drawing nodes, and utility nodes for logging and calibration.

---

## 📦 Packages Overview

- **fr3_generic_drawing**: Main drawing logic (raster and vector).
- **fr3_pick_and_place**: Pick and place demo.
- **franka_gripper_proxy**: Gripper command proxy for MoveIt2.
- **fr3_moveit_config**: MoveIt2 configuration for FR3.
- **fr3_pencil_logger**: Logs the pencil tip position.
- **fr3_test**, **ground_truth_publisher**, **position_calibration**: Utilities and tests.

---

## 🖊️ Drawing Instructions

### Franka Research 3 Control Guide with MoveIt2 + Gripper

> **Important:**  
> - Make sure FCI is **ON** and joints are **UNLOCKED** before operation.  
> - Access the robot web interface at [https://172.16.0.2/desk/](https://172.16.0.2/desk/) to unlock joints and activate FCI.

---

#### 1. 🧰 Open a Terminal

Press <kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>T</kbd> to open a new terminal window.

---

#### 2. 🚀 Launch the FR3 Move Group with MoveIt2

```sh
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
