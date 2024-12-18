import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from custom_action_interfaces.action import Fibonacci

from .map import MapData, Map, Point


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
    ac_map_data = MapData()
    ac_map_data.read_file()
    print("obstacles", ac_map_data.get_obstacles())
    print("station", ac_map_data.get_stations())

    ac_map = Map(ac_map_data)
    path = ac_map.get_path((1, 0), Point(5, 0), Point(18, 18))
    path = ac_map.simplify_path(path)

    if path is not None:
        print(ac_map.display_path(path))
    else:
        print("no path found...")

    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == "__main__":
    main()
