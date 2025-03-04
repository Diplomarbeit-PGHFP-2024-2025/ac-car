import asyncio
import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action import ActionServer

from custom_action_interfaces.action import DriveTo
from custom_action_interfaces.action import DriveToStation
from custom_action_interfaces.srv import GetPath
from custom_action_interfaces.msg import Location
from rclpy.executors import MultiThreadedExecutor

from uagents import Context

from .sort_stations import initialize_car_properties, set_car_properties
from .fetchAgent import agent
from .communication import fetch_stations, register_at_station

minimal_publisher = None


@agent.on_event("startup")
async def introduce_agent(ctx: Context):
    global minimal_publisher
    ctx.logger.info(f"Agent: {agent.name} ({agent.address})")
    initialize_car_properties(ctx)

    rclpy.init()

    minimal_publisher = MinimalPublisher(ctx)


@agent.on_interval(period=2.0)  # Runs every 2 seconds
async def say_hello(ctx: Context):
    global minimal_publisher

    if rclpy.ok():
        rclpy.spin_once(minimal_publisher, timeout_sec=1.0)  # Process ROS2 messages once


def singleton(cls):
    instance = {}

    def get_instance(*args, **kwargs):
        if cls not in instance:
            instance[cls] = cls(*args, **kwargs)
        return instance[cls]

    return get_instance


@singleton
class MinimalPublisher(Node):
    def __init__(self, ctx):
        super().__init__("minimal_publisher")
        self._action_client = ActionClient(self, DriveTo, "drive_to")
        self._path_client = self.create_client(GetPath, "get_path")

        self.ctx = ctx

        # todo - get value from somewhere...
        self.current_location = (10, 0)
        self.angle = math.pi

        self._action_server = ActionServer(
            self, DriveToStation, "drive_to_station", self.execute_drive_to
        )
        self.get_logger().info("started action server...")

    async def fetch_path(
            self,
            car_x: int,
            car_y: int,
            car_rotation: float,
            target_station_x: float,
            target_station_y: float,
    ) -> List[Location]:
        get_path_req = GetPath.Request()

        get_path_req.x = car_x
        get_path_req.y = car_y
        get_path_req.rotation = car_rotation
        get_path_req.target_x = target_station_x
        get_path_req.target_y = target_station_y

        while not self._path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("GetPath service not available, waiting again...")

        print("call")
        response = await self._path_client.call_async(get_path_req)
        print(response)
        path = response.path

        return path

    async def execute_drive_to(self, goal_handle):
        self.get_logger().info("Executing goal...")
        _target_soc = goal_handle.request.target_soc

        green_energy = goal_handle.request.green_energy
        cost_per_kwh = goal_handle.request.cost_per_kwh
        charging_wattage = goal_handle.request.charging_wattage
        max_km = goal_handle.request.max_km

        set_car_properties(
            self.ctx, green_energy, cost_per_kwh, charging_wattage, max_km
        )

        station_id, station_property, time_frame = await fetch_stations(
            self.ctx, self.current_location, 1000
        )

        print(station_property)

        if station_id == "NO STATION":
            print("NO STATION!")
            goal_handle.succeed()

            result = DriveToStation.Result()
            result.status = "failed"
            return result

        await register_at_station(self.ctx, station_id, time_frame)

        self.drove_to_station_future = asyncio.Future()
        self.drive_to_station(
            station_property.geo_point[0], station_property.geo_point[1]
        )

        drove_to_result = await self.drove_to_station_future
        self.get_logger().info(f"drove_to_result: {drove_to_result}")

        # todo - call start charging

        goal_handle.succeed()

        result = DriveToStation.Result()
        result.status = "done"
        return result

    def drive_to_station(self, target_station_x: float, target_station_y: float):
        goal_msg = DriveTo.Goal()

        goal_msg.target_station_x = target_station_x
        goal_msg.target_station_y = target_station_y

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.drive_to_station_goal_callback)

    def drive_to_station_goal_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.drive_to_station_result_callback)

    def drive_to_station_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result: {}".format(result.status_code))

        self.drove_to_station_future.set_result(result)


def main():
    agent.run()


if __name__ == "__main__":
    main()
