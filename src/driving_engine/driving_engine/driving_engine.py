import math

from .algorithms import drive_to
from custom_action_interfaces.action import DriveTo
from custom_action_interfaces.action import Path

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node


class DrivingEngine(Node):
    def __init__(self):
        super().__init__("drive_to_action_client")
        self._path_action_client = ActionClient(self, Path, "path")
        self._action_server = ActionServer(
            self, DriveTo, "drive_to", self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info("got drive request")

        target_station = goal_handle.request.target_station
        self.fetch_path(15, 15, math.pi, target_station)

        # todo - somehow wait till done driving...

        goal_handle.succeed()
        result = DriveTo.Result()

        result.status_code = 404

        return result

    def fetch_path(self, x: int, y: int, car_rotation: float, target_station: int):
        goal_msg = Path.Goal()

        goal_msg.x = x
        goal_msg.y = y
        goal_msg.rotation = car_rotation
        goal_msg.target = target_station

        self._path_action_client.wait_for_server()

        self._send_goal_future = self._path_action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result: {0}".format(result.path))

        path = result.path

        print(path)
        drive_to((0, 0), (0, 1), (0, 10), (1, 1))

        # TODO - your code here


def main(args=None):
    rclpy.init(args=args)

    action_client = DrivingEngine()

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
