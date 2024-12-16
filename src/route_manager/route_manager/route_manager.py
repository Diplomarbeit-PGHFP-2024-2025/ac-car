import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from custom_action_interfaces.action import Fibonacci

from .map import Map


class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__("fibonacci_action_server")
        self._action_server = ActionServer(
            self, Fibonacci, "fibonacci", self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")
        result = Fibonacci.Result()
        return result


def main(args=None):
    ac_map = Map()
    ac_map.read_file()
    print("obstacles", ac_map.get_obstacles())
    print("station", ac_map.get_stations())

    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == "__main__":
    main()
