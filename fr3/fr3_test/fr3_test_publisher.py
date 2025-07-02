import rclpy
from sensor_msgs.msg import JointState
import numpy as np
import time

def main():
    rclpy.init()
    node = rclpy.create_node("test_rosbridge")

    pub = node.create_publisher(JointState, "/joint_command", 10)
    joint_state = JointState()

    joint_state.name = [
        "fr3_joint1",
        "fr3_joint2",
        "fr3_joint3",
        "fr3_joint4",
        "fr3_joint5",
        "fr3_joint6",
        "fr3_joint7",
        "fr3_finger_joint1",
        "fr3_finger_joint2"
    ]

    num_joints = len(joint_state.name)

    joint_state.position = np.array([0.0] * num_joints)
    default_joints = [0.0, -1.16, -0.0, -2.3, -0.0, 1.6, 1.1, 0.4, 0.4]

    max_joints = np.array(default_joints) + 0.5
    min_joints = np.array(default_joints) - 0.5

    time_start = time.time()
    rate = node.create_rate(20)
    while rclpy.ok():
        joint_state.position = np.sin(time.time() - time_start) * (max_joints - min_joints) * 0.5 + default_joints
        pub.publish(joint_state)
        rate.sleep()

if __name__ == '__main__':
    main()