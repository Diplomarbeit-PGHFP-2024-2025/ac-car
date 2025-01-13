import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from custom_action_interfaces.action import Path

from .map import MapData, Map, Point


class PathActionServer(Node):
    def __init__(self):
        super().__init__("path_action_server")
        self._action_server = ActionServer(
            self, Path, "path", self.execute_callback
        )

    def execute_callback(self, goal_handle):
        print(goal_handle.request.target)

        self.get_logger().info("Executing goal...")

        goal_handle.succeed()

        result = Path.Result()

        print("return")

        return result


def main(args=None):
    ac_map_data = MapData()
    ac_map_data.read_file()
    print("obstacles", ac_map_data.get_obstacles())
    print("station", ac_map_data.get_stations())

    ac_map = Map(ac_map_data)
    path = ac_map.get_path((1, 0), Point(15, 15), 0)

    if path is not None:
        path = ac_map.simplify_path(path)
        print(ac_map.display_path(path))
    else:
        print(ac_map)
        print("no path found...")

    rclpy.init(args=args)

    path_action_server = PathActionServer()
    rclpy.spin(path_action_server)


if __name__ == "__main__":
    main()
