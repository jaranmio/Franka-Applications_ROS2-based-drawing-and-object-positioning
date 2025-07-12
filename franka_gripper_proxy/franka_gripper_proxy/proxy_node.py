import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from control_msgs.action import GripperCommand
from franka_msgs.action import Grasp
from rcl_interfaces.msg import ParameterEvent
from rclpy.parameter import Parameter


class GripperCommandProxy(Node):
    def __init__(self):
        super().__init__('gripper_command_proxy')

        # Declare the 'simulate' parameter — TRUE means forward to original GripperCommand
        # same default value as in pick place node launch
        self.declare_parameter('simulate', True)

        self._parameter_event_sub = self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self._on_parameter_event,
            10
        )

        # Accepts MoveIt's GripperCommand action
        self._server = ActionServer(
            self,
            GripperCommand,
            '/fr3_gripper/moveit_gripper_proxy',
            self.execute_callback
        )

        # Grasp action for real execution (Franka-specific)
        self._grasp_client = ActionClient(self, Grasp, '/fr3_gripper/grasp')

        # Standard GripperCommand action (for simulation / planning)
        self._sim_gripper_client = ActionClient(self, GripperCommand, '/fr3_gripper/gripper_action')

        self.get_logger().info('GripperCommand proxy started.')

    def _on_parameter_event(self, event):
        for changed_param in event.changed_parameters:
            if changed_param.name == 'simulate' and event.node == '/pick_place_node':
                value = changed_param.value.bool_value
                self.get_logger().info(f"Mirroring 'simulate' param: {value}")
                self.set_parameters([rclpy.parameter.Parameter(
                    'simulate',
                    rclpy.Parameter.Type.BOOL,
                    value
                )])

    async def execute_callback(self, goal_handle):
        position = goal_handle.request.command.position
        effort = goal_handle.request.command.max_effort
        self.get_logger().info(f'Received GripperCommand goal: pos={position:.3f}, effort={effort:.1f}')

        simulate = self.get_parameter('simulate').get_parameter_value().bool_value

        if simulate:
            # Simulate = true → Forward original GripperCommand
            self.get_logger().info('Simulate=True → Forwarding to /fr3_gripper/gripper_action')

            if not self._sim_gripper_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error('Simulated gripper action server not available.')
                goal_handle.abort()
                return GripperCommand.Result()

            sim_goal = GripperCommand.Goal()
            sim_goal.command.position = position
            sim_goal.command.max_effort = effort

            sim_send_future = self._sim_gripper_client.send_goal_async(sim_goal)
            sim_goal_handle = await sim_send_future

            if not sim_goal_handle.accepted:
                self.get_logger().error('Simulated gripper action goal rejected.')
                goal_handle.abort()
                return GripperCommand.Result()

            sim_result_future = sim_goal_handle.get_result_async()
            await sim_result_future

            self.get_logger().info('Simulated gripper action completed.')
            goal_handle.succeed()
            return GripperCommand.Result()

        else:
            # Simulate = false → Send Franka Grasp action
            self.get_logger().info('Simulate=False → Forwarding to /fr3_gripper/grasp')

            if not self._grasp_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error('Franka /grasp action not available.')
                goal_handle.abort()
                return GripperCommand.Result()

            grasp_goal = Grasp.Goal()
            grasp_goal.width = position
            grasp_goal.force = effort
            grasp_goal.speed = 0.05
            grasp_goal.epsilon.inner = 0.01
            grasp_goal.epsilon.outer = 0.01

            send_goal_future = self._grasp_client.send_goal_async(grasp_goal)
            grasp_goal_handle = await send_goal_future

            if not grasp_goal_handle.accepted:
                self.get_logger().error('Grasp goal rejected.')
                goal_handle.abort()
                return GripperCommand.Result()

            result_future = grasp_goal_handle.get_result_async()
            await result_future

            self.get_logger().info('Franka grasp action completed.')
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
