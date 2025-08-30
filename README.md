# Franka Applications: ROS2-based Robot Drawing & Pick-and-Place

This repository contains ROS2 packages for controlling the Franka Research 3 (FR3) robot arm for drawing (raster/vector), pick-and-place, and gripper proxy operations. It includes MoveIt2 integration, drawing nodes, and utility nodes for logging and calibration.

---

![Star_Drawing](./demos/Drawing/star.gif)
![Pick_Place](./demos/Pick-And-Place/pick_place_side.gif)

## 💻 System Requirements

- **Operating System:** Ubuntu 22.04 LTS (recommended)
- **ROS 2:** Humble (or compatible ROS 2 distribution)
- **Hardware:** Franka Research 3 (FR3) robot arm with FCI enabled
- **RAM:** 16 GB (recommended)
- **Disk Space:** 5 GB free
- **Network:** Direct Ethernet connection to the FR3 robot controller (default IP: 172.16.0.2)
- **Other:**
  - Python 3.10+
  - Colcon build system
  - RViz2
  - Visual Studio Code (recommended for editing)
  - Access to [https://172.16.0.2/desk/](https://172.16.0.2/desk/) for robot control

---

## Packages Overview

- **fr3_generic_drawing**: Main drawing logic (raster and vector).
- **fr3_pick_and_place**: Pick and place logic.
- **franka_gripper_proxy**: Gripper command proxy for MoveIt2.
- **fr3_moveit_config**: MoveIt2 configuration for FR3.
- **fr3_pencil_logger**: Logs the pencil tip position.
- **fr3_test**, **ground_truth_publisher**, **position_calibration**: Utilities and tests.

---

## Drawing Instructions

### Franka Research 3 Control Guide with MoveIt2 + Gripper

> **Important:**  
> - Make sure FCI is **ON** and joints are **UNLOCKED** before operation.  
> - Access the robot web interface at [https://172.16.0.2/desk/](https://172.16.0.2/desk/) to unlock joints and activate FCI.

---

#### 1. Open a Terminal

Press <kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>T</kbd> to open a new terminal window.

---

#### 2. Launch the FR3 Move Group with MoveIt2

```sh
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
```
- This launches the FR3 robot model, RViz2 for visualization, and the MoveIt2 motion planning server.
- Wait until RViz fully loads and displays the robot.

---

#### 3. ➕ Open a Second Terminal Tab

Open a new terminal tab.

---

#### 4. Open the Gripper

```sh
ros2 action send_goal -f /fr3_gripper/move franka_msgs/action/Move "{width: 0.07, speed: 0.05}"
```
- This opens the gripper to 7 cm at 0.05 m/s.

---

#### 5. Close the Gripper (e.g. for gripping a pen)

Try one of the following, depending on your pen/object:

```sh
ros2 action send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.028, speed: 0.03, force: 36.0, epsilon: {inner: 0.002, outer: 0.002}}"
ros2 action send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.033, speed: 0.03, force: 35.0, epsilon: {inner: 0.002, outer: 0.002}}"
ros2 action send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 70.0, epsilon: {inner: 0.1, outer: 0.1}}"
```
- Use a width close to your pen's diameter for a secure grip.

---

#### 6. ❗ If Gripper Command Fails (Aborted)

Calibrate the gripper:

```sh
ros2 action send_goal /fr3_gripper/homing franka_msgs/action/Homing {}
```
- Then retry the open/close commands.

---

#### 7. Moving the Arm in RViz

- In RViz, check the **Motion Planning** panel (left sidebar).
- Use the interactive marker at the gripper tip to move the arm.
- Click **Plan and Execute** to move the real robot.
- For quick positioning, set the Goal State to "ready" and click **Plan & Execute**.

---

#### 9. 🖼️ Launching the Drawing

Choose one:

- **To draw raster (JPEG/PNG):**
  ```sh
  ros2 launch fr3_generic_drawing draw_image.launch.py
  ```
- **To draw vector (SVG):**
  ```sh
  ros2 launch fr3_generic_drawing draw_image_vector.launch.py
  ```

---

#### 10. Changing Drawing Parameters such as the image file to draw, the height of the drawing plane, orientation of the pen, etc.

- **To change image, scale, pen height, or gripper orientation, etc:**
  - Go to the `config` folder on the desktop.
  - Open `config.yaml` and adjust parameters, for example:
    - `image_center_y`: Horizontal position of the image center (in cm, relative to the arm base).
    - `vertical_gripper`: Set to `true` for a vertical pen, `false` for an angled pen.
    - Other parameters may be modified as needed as needed.

- **After changing `config.yaml`, relaunch the drawing node as in step 9.**

---

#### 11. 🛑 To Stop the Movement

Press <kbd>Ctrl</kbd> + <kbd>C</kbd> in the drawing terminal.

---

#### 12. Robot Restart & FCI

- If the arm is restarted, go to [https://172.16.0.2/desk/](https://172.16.0.2/desk/).
- Unlock the joint brakes and activate FCI before running any ROS2 commands.

---

#### 13. Selftest (Take pencil out first!)

- In the web interface, deactivate FCI and lock joints.
- In settings, start selftest: acknowledge and execute.
- Once done, unlock joints and activate FCI for drawing mode.

---

> **Tip:**  
> For any configuration changes in `config.yaml`, always relaunch the drawing node.

---

## 🤖 Pick and Place Instructions

### 1. Launch the Gripper Proxy Node

In a terminal, run:

```sh
ros2 run franka_gripper_proxy proxy_node
```

This node acts as a proxy for gripper actions during pick and place operations.

---

### 2. Configure MoveIt2 to Use the Proxy

1. Navigate to the MoveIt2 configuration folder, for example:

    ```sh
    cd /home/qpaig/franka_ros2_ws/install/franka_fr3_moveit_config/share/franka_fr3_moveit_config/config
    ```

2. Open the file `fr3_controllers.yaml` in a text editor.

3. Change the value of `action_ns` to `moveit_gripper_proxy`:

    ```yaml
    # ...existing code...
    action_ns: moveit_gripper_proxy
    # ...existing code...
    ```

4. Save and close the file.

---

### 3. Launch the Pick and Place Demo

In a terminal, run:

```sh
ros2 launch fr3_pick_and_place pick_and_place.launch.py
```

This will execute a hardcoded pick and place sequence using the FR3 robot.

---

### 4. General ROS2 Launch Instructions

After sourcing your install folder, you can launch any ROS2 project with:

```sh
ros2 launch [project_name] [launch_file_name]
```
or
```sh
ros2 launch [project_name] [node_file_name]
```

Replace `[project_name]` and `[launch_file_name]` with the appropriate names for your package and launch file.

---

## 🛠️ Building the Project

From the workspace root:

```sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Then source the install folder:
```sh
source install/setup.bash
```

---

## 📄 License

See individual package folders for license information.

---
