# Franka Applications: ROS2-based Drawing & Pick-and-Place

This repository contains ROS2 packages for controlling the Franka Research 3 (FR3) robot arm for drawing, pick-and-place, and gripper proxy operations. It includes MoveIt2 integration, drawing nodes (raster and vector), and utility nodes for logging and calibration.

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

This guide walks you through launching the MoveIt2 interface for the FR3 arm, initializing the gripper, and sending control commands through the terminal and RViz.

---

### 1. 🧰 Open a Terminal

Press <kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>T</kbd> to open a new terminal window.

---

### 2. 🚀 Launch the FR3 Move Group with MoveIt2

```sh
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
```

This launches:
- The FR3 robot model
- RViz2 for visualization
- MoveIt2 motion planning server

Wait until RViz fully loads and displays the robot.

---

### 3. ➕ Open a Second Terminal Tab

Click the <kbd>+</kbd> icon in the terminal window to open a new tab.

---

### 4. 🔗 Launch the Gripper Node

```sh
ros2 launch franka_gripper gripper.launch.py robot_ip:=172.16.0.2 namespace:=fr3_gripper
```

- Connects to the physical gripper.
- Sets up the `/fr3_gripper` namespace.

---

### 5. ➕ Open a Third Terminal Tab

Open another terminal tab.

---

### 6. 👐 Open the Gripper

```sh
ros2 action send_goal -f /fr3_gripper/move franka_msgs/action/Move "{width: 0.07, speed: 0.05}"
```

This opens the gripper to 7 cm at 0.05 m/s.

---

### 7. 🤏 Close the Gripper (e.g. for gripping a pen)

```sh
ros2 action send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.040, speed: 0.03, force: 31.0, epsilon: {inner: 0.002, outer: 0.002}}"
```

Or for a tighter grip:

```sh
ros2 action send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.009, speed: 0.03, force: 31.0, epsilon: {inner: 0.002, outer: 0.002}}"
```

---

### 8. ❗ If Gripper Command Fails (Aborted)

Calibrate the gripper:

```sh
ros2 action send_goal /fr3_gripper/homing franka_msgs/action/Homing {}
```

Then retry the open/close commands.

---

### 9. Moving the Arm in RViz

- In RViz, check the **Motion Planning** panel.
- Use the interactive marker at the gripper tip to move the arm.
- Click **Plan and Execute** to move the real robot.

---

### 10. ➕ Open a Fourth Terminal Tab

Ignore any `franka_robot_state_broadcaster` errors unless they persist.

---

### 11. 🖼️ Launching the Drawing

In the fourth terminal:

```sh
cd /home/qpaig/my_ros2_ws/src
source fr3_fetch_robot_description.sh
```

Then choose one:

- **To draw raster (JPEG/PNG):**
  ```sh
  ros2 launch fr3_generic_drawing draw_image.launch.py
  ```
- **To draw vector (SVG):**
  ```sh
  ros2 launch fr3_generic_drawing draw_image_vector.launch.py
  ```

---

### 12. ✏️ Changing Pen Height or Scale

- **Edit Pen Height or Scale:**
  - Open Visual Studio Code:
    ```sh
    cd /home/qpaig/my_ros2_ws/src/fr3_generic_drawing
    code .
    ```
  - For raster: Edit `const double DRAWING_HEIGHT = ...;` and `const double CONVERSION_FACTOR = ...;` in `draw_image.cpp`.
  - For vector: Edit corresponding variables in `draw_image_vector.cpp`.

- **Rebuild:**
  ```sh
  cd /home/qpaig/my_ros2_ws
  rm -rf build/ log/ install/
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

- **Relaunch drawing as in step 11.**

---

### 13. 🛑 To Stop the Movement

Press <kbd>Ctrl</kbd> + <kbd>C</kbd> in the fourth terminal.

---

### 14. Note on Robot Restart

- If the arm is restarted, go to a browser and type `172.16.0.2`.
- Unlock the joint brakes and activate FCI.

---

### 15. Selftest

- In the web interface, lock joints, acknowledge, and execute.
- When the blue light is ready, unlock joints and check FCI for drawing mode.

---

### 16. To Change Drawings

- On the desktop, use the `copy_from_here` and `paste_here` folders.
- Copy your image to `paste_here` and rename it to `image.png`.
- Follow step 11 to launch the drawing.
- Adjust scaling if needed for different image dimensions.

---

## 📝 Additional Notes

- All launch files are in launch.
- Configuration files for the robot and drawing parameters are in raster_config.
- For pick-and-place, see `fr3_pick_and_place`.

---

## 🛠️ Building the Project

From the workspace root:

```sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## 📄 License

See individual package folders for license information.

---

## 📚 References

- [MoveIt2 Documentation](https://moveit.ros.org/)
- [Franka Emika Documentation](https://frankaemika.github.io/docs/)

---
```
You can further edit and format this file as needed for your project.You can further edit and format this file as needed for your project.

jaranmio: ADD INSTRUCTIONS OF RPICKJA ND PALCE BASE DON THIS: "- To launch the proxy for the pick and place, run: "ros2 run franka_gripper_proxy proxy_node." To tell franka ros2 to use that proxy, navigate to: "/home/qpaig/franka_ros2_ws/install/franka_fr3_moveit_config/share/franka_fr3_moveit_config/config" and modify the file: "fr3_controllers.yaml." Change the config value of 'action_ns' to 'moveit_gripper_proxy'
- To launch the pick and place (hardcoded), run: "ros2 launch fr3_pick_and_place pick_and_place.launch.py"
- Generally, to launch a ROS2 project (after having sourced the install folder) run: "ros2 launch [project name] [launch file name]" or "ros2 launch [project name] [node file nmae]""

GitHub Copilot: ```md
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

1. Navigate to the MoveIt2 configuration folder:

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
```
Add this section to your README.md under the drawing instructions.

jaranmio: generate full final README.md file

GitHub Copilot: ```md
# Franka Drawing & Pick-and-Place ROS2 Workspace

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

This guide walks you through launching the MoveIt2 interface for the FR3 arm, initializing the gripper, and sending control commands through the terminal and RViz.

---

### 1. 🧰 Open a Terminal

Press <kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>T</kbd> to open a new terminal window.

---

### 2. 🚀 Launch the FR3 Move Group with MoveIt2

```sh
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
```

This launches:
- The FR3 robot model
- RViz2 for visualization
- MoveIt2 motion planning server

Wait until RViz fully loads and displays the robot.

---

### 3. ➕ Open a Second Terminal Tab

Click the <kbd>+</kbd> icon in the terminal window to open a new tab.

---

### 4. 🔗 Launch the Gripper Node

```sh
ros2 launch franka_gripper gripper.launch.py robot_ip:=172.16.0.2 namespace:=fr3_gripper
```

- Connects to the physical gripper.
- Sets up the `/fr3_gripper` namespace.

---

### 5. ➕ Open a Third Terminal Tab

Open another terminal tab.

---

### 6. 👐 Open the Gripper

```sh
ros2 action send_goal -f /fr3_gripper/move franka_msgs/action/Move "{width: 0.07, speed: 0.05}"
```

This opens the gripper to 7 cm at 0.05 m/s.

---

### 7. 🤏 Close the Gripper (e.g. for gripping a pen)

```sh
ros2 action send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.040, speed: 0.03, force: 31.0, epsilon: {inner: 0.002, outer: 0.002}}"
```

Or for a tighter grip:

```sh
ros2 action send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.009, speed: 0.03, force: 31.0, epsilon: {inner: 0.002, outer: 0.002}}"
```

---

### 8. ❗ If Gripper Command Fails (Aborted)

Calibrate the gripper:

```sh
ros2 action send_goal /fr3_gripper/homing franka_msgs/action/Homing {}
```

Then retry the open/close commands.

---

### 9. Moving the Arm in RViz

- In RViz, check the **Motion Planning** panel.
- Use the interactive marker at the gripper tip to move the arm.
- Click **Plan and Execute** to move the real robot.

---

### 10. ➕ Open a Fourth Terminal Tab

Ignore any `franka_robot_state_broadcaster` errors unless they persist.

---

### 11. 🖼️ Launching the Drawing

In the fourth terminal:

```sh
cd /home/qpaig/my_ros2_ws/src
source fr3_fetch_robot_description.sh
```

Then choose one:

- **To draw raster (JPEG/PNG):**
  ```sh
  ros2 launch fr3_generic_drawing draw_image.launch.py
  ```
- **To draw vector (SVG):**
  ```sh
  ros2 launch fr3_generic_drawing draw_image_vector.launch.py
  ```

---

### 12. ✏️ Changing Pen Height or Scale

- **Edit Pen Height or Scale:**
  - Open Visual Studio Code:
    ```sh
    cd /home/qpaig/my_ros2_ws/src/fr3_generic_drawing
    code .
    ```
  - For raster: Edit `const double DRAWING_HEIGHT = ...;` and `const double CONVERSION_FACTOR = ...;` in `draw_image.cpp`.
  - For vector: Edit corresponding variables in `draw_image_vector.cpp`.

- **Rebuild:**
  ```sh
  cd /home/qpaig/my_ros2_ws
  rm -rf build/ log/ install/
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

- **Relaunch drawing as in step 11.**

---

### 13. 🛑 To Stop the Movement

Press <kbd>Ctrl</kbd> + <kbd>C</kbd> in the fourth terminal.

---

### 14. Note on Robot Restart

- If the arm is restarted, go to a browser and type `172.16.0.2`.
- Unlock the joint brakes and activate FCI.

---

### 15. Selftest

- In the web interface, lock joints, acknowledge, and execute.
- When the blue light is ready, unlock joints and check FCI for drawing mode.

---

### 16. To Change Drawings

- On the desktop, use the `copy_from_here` and `paste_here` folders.
- Copy your image to `paste_here` and rename it to `image.png`.
- Follow step 11 to launch the drawing.
- Adjust scaling if needed for different image dimensions.

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

1. Navigate to the MoveIt2 configuration folder:

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

---

## 📄 License

See individual package folders for license information.

---

## 📚 References

- [MoveIt2 Documentation](https://moveit.ros.org/)
- [Franka Emika Documentation](https://frankaemika.github.io/docs/)

---
```


jaranmio: update the drawing instructions based on this: "#
FCI on and joints open for operation
Franka Research 3 Control Guide with MoveIt2 + Gripper
This guide walks you through launching the MoveIt2 interface for the Franka Research 3 (FR3) arm, initializing the gripper, and sending control commands through the terminal and RViz.
1. 🧰 Open a Terminal
Press Ctrl + Alt + T to open a new terminal window.
2. 🚀 Launch the FR3 Move Group with MoveIt2
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
This command launches:
- The FR3 robot model# horizontal position of the center of the image (left / right), in cm, with respect to the arm's base
# increasing this values moves the center of the image towards the left of the paper
# along length
image_center_y: 0
- RViz2 for visualization
- MoveIt2 motion planning serverros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2

 Wait until RViz fully loads and displays the robot in the scene.
3. ➕ Open a Second Terminal Tab
Click the + icon in the terminal window to open a new tab.
4.  Open the Gripper
ros2 action send_goal -f /fr3_gripper/move franka_msgs/action/Move "{width: 0.07, speed: 0.05}"
previous command: ros2 action send_goal -f /fr3_gripper/franka_gripper/move franka_msgs/action/Move "{width: 0.07, speed: 0.05}"ros2 action send_goal -f /fr3_gripper/move franka_msgs/action/Move "{width: 0.07, speed: 0.05}"ros2 launch fr3_generic_drawing drawros2 launch fr3_generic_drawing draw_image.launch.py_image.launch.py
ros2 action send_goal -f /fr3_gripper/move franka_msgs/action/Move "{width: 0.07, speed: 0.05}"ros2 launch fr3_generic_drawing draw_image.launch.pyThis opens the gripper to 8 cm at 0.1 m/s.
5. 🤏 Close the Gripper (e.g. for gripping a pen)
ros2 action send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 70.0, epsilon: {inner: 0.1, outer: 0.1}}"
ros2 action send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 36.0, epsilon: {inner: 0.1, outer: 0.1}}"
ros2 action send_goal -f /fr3_gripper/grasp franka_msgs/ac # test linesScreen Shot 2025-06-23 at 4.44tion/Grasp "{width: 0.028, speed: 0.03, force: 36.0, epsilon: {inner: 0.002, outer: 0.002}}"ros2 action send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 36.0, epsilon: {inner: 0.1, outer: 0.1}}"
ros2 action send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.033, speed: 0.03, force: 35.0, epsilon: {inner: 0.002, outer: 0.002}}"

ros2 action send_gros2 action send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.20.00, speed: 0.03, force: 70.0, epsilon: {inner: 0.1, outer: 0.1}}"oal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.009, speed: 0.03, force: 25.0, epsilon: {inner: 0.002, outer: 0.002}}".
ros2 launch fr3_generic_drawing draw_image.launch.py
previous command: ros2 action send_goal -f /fr3_gripper/franka_gripper/grasp franka_msgs/action/Grasp "{width: 0.009, speed: 0.03, forceros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2: 23.0, epsilon: {inner: 0.002, outer: 0.002}}"
Use this to grip an object ~0.9 cm wide.
6. ❗ If Gripper Command Fails (Aborted)
ros2 action send_goal /fr3_gripper/homing franka_msgs/action/Homing {}
prev command: “ros2 action send_goal /fr3_gripper/franka_gripper/homing franka_msgs/action/Homing {}”ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
If you see an error, the gripper may need calibration.
Then retry tros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2he open and close commands.
7. Moving the Arm
ros2 action send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 36.0, epsilon: {inner: 0.1, outer: 0.1}}" in RViz
Back in the RViz window from Step 2:
1. On the left panel, scroll and check the ros2 action send_goal -fros2 action ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 36.0, epsilon: {inner: 0.1, outer: 0.1}}" /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 36.0, epsilon: {inner: 0.1, outer: 0.1}}"box for Motion Planning.ros2 action ros2 action send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 36.0, epsilon: {inner: 0.1, outer: 0.1}}"send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 36.0, epsilon: {inner: 0.1, outer: 0.1}}"
2. An interact what():  libfranka: Move command aborted: motion aborted by reflex! ["joint_velocity_violation"]ive marker (arrow) appears at the gripper tip.
3. Click and drag the arrow slightly toheight move the arm.
4. Click Plan and Execute in the Motion Planning panel to execute the motion on the real robot. 2
 2This is useful for inserting or detaching objectros2 action send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 36.0, epsilon: {inner: 0.1, outer: 0.1}}"s.
8 ➕ Oros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2pen a Third Terminal Tab
Click the + agaros2 launch fr3_generic_drawing draw_image.launch.pyin to open one more terminal
11. Launching the drawing 
Enter the following commands, all at opython3 quick_start_cross_platform.pynce in the fourth terminal: 
cd /home/qpaig/my_ros2_ws/srros2 launch fr3_generic_drawing draw_image_vector.launch.pyc
source fr3_fetch_robot_description.sh

	
To draw Machine (raster / jpeg), file in visual studio code draw_image.cpp:
ros2 launch fr3_generic_drawing draw_image.launch.py

	To draw svg (vector), file in visual studio code draw_image_vector.cpp:
ros2 launch fr3_generic_drawing draw_image_vector.launch.py
ros2 launch fr3_generic_drawing draw_image.launch.py

11)
Lift arm via programming manually or go to  launch Rviz and then choose  motion planning, then  below Goal State , choose ready, then Plan & Execute

12. Changing drawing parameters
Go to the config folder in desktop, open the file ‘config.yaml’, and change configuration as desired there.
               python3 quick_start_cross_platform.py
Follow step 11) for launching drawing

13. To Stop the movement:
Press:
Ctrl + C in the third terminal

14
Note: Everytime robotic arm is restarted, go to google. Then DESK: TYPE 172.16.0.2 . Then unlock the joints brakes and activate FCI 

15
Selftest: take pencil out!
Go to  google , desk ,  under N/A top right 
deactivate FCI and lock joints
in setting start selftest : acknowledge and execute
once done and clock runs go to desk , then  N/A unlock joints and activate FCI.
lock joints and acknowledge and execute.
When blue light ready. 

For drawing mode
UNLOCK joints under desk 
check FCI on 


Important’
on google to get to DESK
https://172.16.0.2/desk/

FRANKA WORLD LOG IN 
credentials

16. To change Drawings
Go to the desktop. There are 2 very visible files named ‘copy_from_here’ and ‘paste_here.’ Copy the desired image from the folder named ‘copy_from_here’ and paste in the folder named ‘paste_here.’ After pasting, rename the image you want to draw as exactly: image.png. Then follow normal instructions to launch a drawing (see 11. above). Once the drawing is done, you can delete the image and restart with another one.

Note that because the image files have different dimensions, you may need to adjust the scaling based on whether you observe the drawing of the robot to be too small or large
ros2 action send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 70.0, epsilon: {inner: 0.1, outer: 0.1}}"

16. To change gripper orientation
Open the config folder in desktop. Then open the file ‘config.yaml’. Look for the name ‘vertical_gripper’. Set this to true for a vertical gripper / pen, and to false for an angled gripper/pen
" try to be smart abotu it

GitHub Copilot: ```md
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
```
- This launches the FR3 robot model, RViz2 for visualization, and the MoveIt2 motion planning server.
- Wait until RViz fully loads and displays the robot.

---

#### 3. ➕ Open a Second Terminal Tab

Open a new terminal tab.

---

#### 4. 👐 Open the Gripper

```sh
ros2 action send_goal -f /fr3_gripper/move franka_msgs/action/Move "{width: 0.07, speed: 0.05}"
```
- This opens the gripper to 7 cm at 0.05 m/s.

---

#### 5. 🤏 Close the Gripper (e.g. for gripping a pen)

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

#### 8. ➕ Open a Third Terminal Tab

Open another terminal tab for drawing commands.

---

#### 9. 🖼️ Launching the Drawing

In the third terminal:

```sh
cd /home/qpaig/my_ros2_ws/src
source fr3_fetch_robot_description.sh
```

Then choose one:

- **To draw raster (JPEG/PNG):**
  ```sh
  ros2 launch fr3_generic_drawing draw_image.launch.py
  ```
- **To draw vector (SVG):**
  ```sh
  ros2 launch fr3_generic_drawing draw_image_vector.launch.py
  ```

---

#### 10. ✏️ Changing Drawing Parameters

- **To change drawing position, scale, or gripper orientation:**
  - Go to the `config` folder on the desktop.
  - Open `config.yaml` and adjust parameters:
    - `image_center_y`: Horizontal position of the image center (in cm, relative to the arm base).
    - `vertical_gripper`: Set to `true` for a vertical pen, `false` for an angled pen.
    - Other parameters for scaling and offsets as needed.

- **To change pen height or scale in code:**
  - Open Visual Studio Code:
    ```sh
    cd /home/qpaig/my_ros2_ws/src/fr3_generic_drawing
    code .
    ```
  - Edit `DRAWING_HEIGHT` and `CONVERSION_FACTOR` in `draw_image.cpp` (raster) or `draw_image_vector.cpp` (vector).

- **Rebuild after changes:**
  ```sh
  cd /home/qpaig/my_ros2_ws
  rm -rf build/ log/ install/
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

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

#### 14. To Change Drawings

- On the desktop, use the `copy_from_here` and `paste_here` folders.
- Copy your image to `paste_here` and rename it to `image.png`.
- Follow the drawing launch instructions above.
- Adjust scaling in `config.yaml` or code if the drawing is too small or large.

---

#### 15. To Change Gripper Orientation

- In the `config` folder on the desktop, open `config.yaml`.
- Set `vertical_gripper: true` for a vertical pen, or `false` for an angled pen.

---

> **Tip:**  
> For any configuration changes, always rebuild and relaunch the drawing node.

---
```
Replace your previous drawing instructions section with this updated version for clarity and accuracy.Replace your previous drawing instructions section with this updated version for clarity and accuracy.

jaranmio: generate full final readme.md

GitHub Copilot: ```md
# Franka Drawing & Pick-and-Place ROS2 Workspace

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
```
- This launches the FR3 robot model, RViz2 for visualization, and the MoveIt2 motion planning server.
- Wait until RViz fully loads and displays the robot.

---

#### 3. ➕ Open a Second Terminal Tab

Open a new terminal tab.

---

#### 4. 👐 Open the Gripper

```sh
ros2 action send_goal -f /fr3_gripper/move franka_msgs/action/Move "{width: 0.07, speed: 0.05}"
```
- This opens the gripper to 7 cm at 0.05 m/s.

---

#### 5. 🤏 Close the Gripper (e.g. for gripping a pen)

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

#### 8. ➕ Open a Third Terminal Tab

Open another terminal tab for drawing commands.

---

#### 9. 🖼️ Launching the Drawing

In the third terminal:

```sh
cd /home/qpaig/my_ros2_ws/src
source fr3_fetch_robot_description.sh
```

Then choose one:

- **To draw raster (JPEG/PNG):**
  ```sh
  ros2 launch fr3_generic_drawing draw_image.launch.py
  ```
- **To draw vector (SVG):**
  ```sh
  ros2 launch fr3_generic_drawing draw_image_vector.launch.py
  ```

---

#### 10. ✏️ Changing Drawing Parameters

- **To change drawing position, scale, or gripper orientation:**
  - Go to the `config` folder on the desktop.
  - Open `config.yaml` and adjust parameters:
    - `image_center_y`: Horizontal position of the image center (in cm, relative to the arm base).
    - `vertical_gripper`: Set to `true` for a vertical pen, `false` for an angled pen.
    - Other parameters for scaling and offsets as needed.

- **To change pen height or scale in code:**
  - Open Visual Studio Code:
    ```sh
    cd /home/qpaig/my_ros2_ws/src/fr3_generic_drawing
    code .
    ```
  - Edit `DRAWING_HEIGHT` and `CONVERSION_FACTOR` in `draw_image.cpp` (raster) or `draw_image_vector.cpp` (vector).

- **Rebuild after changes:**
  ```sh
  cd /home/qpaig/my_ros2_ws
  rm -rf build/ log/ install/
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

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

#### 14. To Change Drawings

- On the desktop, use the `copy_from_here` and `paste_here` folders.
- Copy your image to `paste_here` and rename it to `image.png`.
- Follow the drawing launch instructions above.
- Adjust scaling in `config.yaml` or code if the drawing is too small or large.

---

#### 15. To Change Gripper Orientation

- In the `config` folder on the desktop, open `config.yaml`.
- Set `vertical_gripper: true` for a vertical pen, or `false` for an angled pen.

---

> **Tip:**  
> For any configuration changes, always rebuild and relaunch the drawing node.

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

1. Navigate to the MoveIt2 configuration folder:

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

---

## 📄 License

See individual package folders for license information.

---

## 📚 References

- [MoveIt2 Documentation](https://moveit.ros.org/)
- [Franka Emika Documentation](https://frankaemika.github.io/docs/)

---


jaranmio: you don't undersytand kme. Make that into a downloadable readme.md file. ALSO the user shouldn't have nistructions to modify code, the pen height can also be changed in the config.yaml file

GitHub Copilot: ```md
# Franka Drawing & Pick-and-Place ROS2 Workspace

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
```
- This launches the FR3 robot model, RViz2 for visualization, and the MoveIt2 motion planning server.
- Wait until RViz fully loads and displays the robot.

---

#### 3. ➕ Open a Second Terminal Tab

Open a new terminal tab.

---

#### 4. 👐 Open the Gripper

```sh
ros2 action send_goal -f /fr3_gripper/move franka_msgs/action/Move "{width: 0.07, speed: 0.05}"
```
- This opens the gripper to 7 cm at 0.05 m/s.

---

#### 5. 🤏 Close the Gripper (e.g. for gripping a pen)

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

#### 8. ➕ Open a Third Terminal Tab

Open another terminal tab for drawing commands.

---

#### 9. 🖼️ Launching the Drawing

In the third terminal:

```sh
cd /home/qpaig/my_ros2_ws/src
source fr3_fetch_robot_description.sh
```

Then choose one:

- **To draw raster (JPEG/PNG):**
  ```sh
  ros2 launch fr3_generic_drawing draw_image.launch.py
  ```
- **To draw vector (SVG):**
  ```sh
  ros2 launch fr3_generic_drawing draw_image_vector.launch.py
  ```

---

#### 10. ✏️ Changing Drawing Parameters

- **To change drawing position, scale, pen height, or gripper orientation:**
  - Go to the `config` folder on the desktop.
  - Open `config.yaml` and adjust parameters:
    - `image_center_y`: Horizontal position of the image center (in cm, relative to the arm base).
    - `vertical_gripper`: Set to `true` for a vertical pen, `false` for an angled pen.
    - `drawing_height`: Set the pen height (in meters).
    - Other parameters for scaling and offsets as needed.

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

#### 14. To Change Drawings

- On the desktop, use the `copy_from_here` and `paste_here` folders.
- Copy your image to `paste_here` and rename it to `image.png`.
- Follow the drawing launch instructions above.
- Adjust scaling in `config.yaml` if the drawing is too small or large.

---

#### 15. To Change Gripper Orientation

- In the `config` folder on the desktop, open `config.yaml`.
- Set `vertical_gripper: true` for a vertical pen, or `false` for an angled pen.

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

1. Navigate to the MoveIt2 configuration folder:

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

---

## 📄 License

See individual package folders for license information.

---

## 📚 References

- [MoveIt2 Documentation](https://moveit.ros.org/)
- [Franka Emika Documentation](https://frankaemika.github.io/docs/)

---
```

**To download:**  
1. Copy the above content into a file named README.md in your project root.
2. You can now download or share this file as needed.


jaranmio: but it won't cipy the style if i just copy that. i want atlkeadt something i can directly copya nd paste without lsoing style, rmemeber that .md file shave special tags like '##'

GitHub Copilot: Absolutely! The content I provided above **is** in Markdown (`.md`) format, which uses special tags like `#`, `##`, `-`, and code blocks (triple backticks) for formatting. When you copy and paste it into a file named README.md, GitHub and other Markdown viewers will automatically render the headings, lists, and code blocks with the correct style.

**Instructions:**

1. **Copy** all the text in the code block below (including the `#`, `##`, and triple backticks for code).
2. **Paste** it into a new file named README.md in your project folder.
3. **Save** the file.
4. **View** it on GitHub or in a Markdown viewer/editor (like VS Code) to see the styled output.

---

```md
# Franka Drawing & Pick-and-Place ROS2 Workspace

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
```
- This launches the FR3 robot model, RViz2 for visualization, and the MoveIt2 motion planning server.
- Wait until RViz fully loads and displays the robot.

---

#### 3. ➕ Open a Second Terminal Tab

Open a new terminal tab.

---

#### 4. 👐 Open the Gripper

```sh
ros2 action send_goal -f /fr3_gripper/move franka_msgs/action/Move "{width: 0.07, speed: 0.05}"
```
- This opens the gripper to 7 cm at 0.05 m/s.

---

#### 5. 🤏 Close the Gripper (e.g. for gripping a pen)

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

#### 8. ➕ Open a Third Terminal Tab

Open another terminal tab for drawing commands.

---

#### 9. 🖼️ Launching the Drawing

In the third terminal:

```sh
cd /home/qpaig/my_ros2_ws/src
source fr3_fetch_robot_description.sh
```

Then choose one:

- **To draw raster (JPEG/PNG):**
  ```sh
  ros2 launch fr3_generic_drawing draw_image.launch.py
  ```
- **To draw vector (SVG):**
  ```sh
  ros2 launch fr3_generic_drawing draw_image_vector.launch.py
  ```

---

#### 10. ✏️ Changing Drawing Parameters

- **To change drawing position, scale, pen height, or gripper orientation:**
  - Go to the `config` folder on the desktop.
  - Open `config.yaml` and adjust parameters:
    - `image_center_y`: Horizontal position of the image center (in cm, relative to the arm base).
    - `vertical_gripper`: Set to `true` for a vertical pen, `false` for an angled pen.
    - `drawing_height`: Set the pen height (in meters).
    - Other parameters for scaling and offsets as needed.

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

#### 14. To Change Drawings

- On the desktop, use the `copy_from_here` and `paste_here` folders.
- Copy your image to `paste_here` and rename it to `image.png`.
- Follow the drawing launch instructions above.
- Adjust scaling in `config.yaml` if the drawing is too small or large.

---

#### 15. To Change Gripper Orientation

- In the `config` folder on the desktop, open `config.yaml`.
- Set `vertical_gripper: true` for a vertical pen, or `false` for an angled pen.

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

1. Navigate to the MoveIt2 configuration folder:

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

---

## 📄 License

See individual package folders for license information.

---

## 📚 References

- [MoveIt2 Documentation](https://moveit.ros.org/)
- [Franka Emika Documentation](https://frankaemika.github.io/docs/)

---
```

---
