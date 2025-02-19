#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import time

from example_msgs.action import ExampleAction

class ExampleActionServer(Node):

    def __init__(self):
        super().__init__('example_action_server')
        self._action_server = ActionServer(
            self,
            ExampleAction,
            'example_action_server',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Simulated processing time
        time.sleep(5)

        # Check if the goal was canceled
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled.')
            return ExampleAction.Result()

        # Process the goal and prepare the result
        result = ExampleAction.Result()
        result.server_output = goal_handle.request.server_input + 2
        self.get_logger().info(f'Succeeded. Added two to input. Returning {result.server_output}')

        goal_handle.succeed()  # Correct usage without passing 'result'
        return result  # Return the result directly

def main(args=None):
    rclpy.init(args=args)
    example_action_server = ExampleActionServer()

    try:
        rclpy.spin(example_action_server)
    except KeyboardInterrupt:
        pass
    finally:
        example_action_server.destroy_node()  # Correct method to destroy the node
        rclpy.shutdown()

if __name__ == '__main__':
    main()
