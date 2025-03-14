import asyncio
import os
from typing import List

import rclpy
from dotenv import load_dotenv
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action import ActionServer

from custom_action_interfaces.action import DriveTo
from custom_action_interfaces.action import DriveToStation
from custom_action_interfaces.srv import GetPath
from custom_action_interfaces.msg import Location

from uagents import Context

from .sort_stations import initialize_car_properties, set_car_properties
from .fetchAgent import agent
from .communication import fetch_stations, register_at_station

load_dotenv()


@agent.on_event("startup")
async def introduce_agent(ctx: Context):
    ctx.logger.info(f"Agent: {agent.name} ({agent.address})")
    initialize_car_properties(ctx)

    rclpy.init()


@agent.on_interval(period=2.0)  # Runs every 2 seconds
async def say_hello(ctx: Context):
    minimal_publisher = MinimalPublisher(ctx)

    # check if ctx.storage.get("finished_waiting") is true so we don't spin ros while we are fetching stations via fetchAI since this causes problems with optimal_station_future
    if rclpy.ok() and ctx.storage.get("finished_waiting"):
        rclpy.spin_once(
            minimal_publisher, timeout_sec=1.0
        )  # Process ROS2 messages once


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
        self.current_location = (float(os.getenv("CAR_X")), float(os.getenv("CAR_Y")))
        self.angle = float(os.getenv("CAR_ANGLE"))

        print(self.current_location)
        print(self.angle)

        self._action_server = ActionServer(
            self, DriveToStation, "drive_to_station", self.execute_drive_to
        )
        self.get_logger().info("started action server...")

    async def fetch_path(
        self,
        car_x: float,
        car_y: float,
        car_rotation: float,
        target_station_x: float,
        target_station_y: float,
    ) -> List[Location]:
        get_path_req = GetPath.Request()

        get_path_req.x = float(car_x)
        get_path_req.y = float(car_y)
        get_path_req.rotation = float(car_rotation)
        get_path_req.target_x = float(target_station_x)
        get_path_req.target_y = float(target_station_y)

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
            station_property.geo_point[0],
            station_property.geo_point[1],
            self.current_location[0],
            self.current_location[1],
            self.angle,
        )

        # todo - crash because of... i dont know why :(
        drove_to_result = await self.drove_to_station_future
        self.get_logger().info(f"drove_to_result: {drove_to_result}")

        # todo - call start charging

        goal_handle.succeed()

        result = DriveToStation.Result()
        result.status = "done"
        return result

    def drive_to_station(
        self,
        target_station_x: float,
        target_station_y: float,
        car_x: float,
        car_y: float,
        car_angle: float,
    ):
        goal_msg = DriveTo.Goal()

        goal_msg.target_station_x = float(target_station_x)
        goal_msg.target_station_y = float(target_station_y)
        goal_msg.car_x = float(car_x)
        goal_msg.car_y = float(car_y)
        goal_msg.car_angle = float(car_angle)

        print(goal_msg)

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
