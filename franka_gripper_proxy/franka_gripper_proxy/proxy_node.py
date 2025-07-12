import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from control_msgs.action import GripperCommand
from franka_msgs.action import Grasp

class GripperCommandProxy(Node):
    def __init__(self):
        super().__init__('gripper_command_proxy')
        self._server = ActionServer(
            self,
            GripperCommand,
            '/fr3_gripper/moveit_gripper_proxy',
            self.execute_callback
        )
        self._grasp_client = ActionClient(self, Grasp, '/fr3_gripper/grasp')
        self.get_logger().info('GripperCommand proxy started.')

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received GripperCommand goal: pos={goal_handle.request.command.position:.3f}, effort={goal_handle.request.command.max_effort:.1f}')
        grasp_goal = Grasp.Goal()
        grasp_goal.width = goal_handle.request.command.position
        grasp_goal.force = goal_handle.request.command.max_effort
        grasp_goal.speed = 0.05
        grasp_goal.epsilon.inner = 0.01
        grasp_goal.epsilon.outer = 0.01

        if not self._grasp_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Franka /grasp action not available.')
            goal_handle.abort()
            return GripperCommand.Result()

        send_goal_future = self._grasp_client.send_goal_async(grasp_goal)
        await send_goal_future
        await send_goal_future.result().get_result_async()

        goal_handle.succeed()
        return GripperCommand.Result()

def main(args=None):
    rclpy.init(args=args)
    node = GripperCommandProxy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
