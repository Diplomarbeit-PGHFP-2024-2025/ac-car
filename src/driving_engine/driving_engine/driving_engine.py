import asyncio
import math
from typing import List

from .algorithms import drive_to
from custom_action_interfaces.action import DriveTo
from custom_action_interfaces.srv import GetPath
from custom_action_interfaces.msg import Location

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node


class DrivingEngine(Node):
    def __init__(self):
        super().__init__("drive_to_action_client")

        self._get_path_client = self.create_client(GetPath, 'get_path')
        self._action_server = ActionServer(
            self, DriveTo, "drive_to", self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info("got drive request")

        target_station = goal_handle.request.target_station
        path = await self.fetch_path(15, 15, math.pi, target_station)

        print(path)
        drive_to((0, 0), (0, 1), (0, 10), (1, 1))

        # todo - somehow wait till done driving...

        goal_handle.succeed()
        result = DriveTo.Result()

        result.status_code = 404

        return result

    async def fetch_path(self, x: int, y: int, car_rotation: float, target_station: int) -> List[Location]:
        get_path_req = GetPath.Request()

        get_path_req.x = x
        get_path_req.y = y
        get_path_req.rotation = car_rotation
        get_path_req.target = target_station

        while not self._get_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GetPath service not available, waiting again...')

        print("call")
        response = await self._get_path_client.call_async(get_path_req)
        print(response)
        path = response.path

        return path


def main(args=None):
    rclpy.init(args=args)

    action_client = DrivingEngine()

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
