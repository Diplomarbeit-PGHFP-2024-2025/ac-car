import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action import ActionServer

from custom_action_interfaces.action import DriveTo
from custom_action_interfaces.action import DriveToStation

from uagents import Context

from .sort_stations import initialize_car_properties, set_car_properties
from .fetchAgent import agent
from .communication import fetch_stations, register_at_station


@agent.on_event("startup")
async def introduce_agent(ctx: Context):
    ctx.logger.info(f"Agent: {agent.name} ({agent.address})")
    initialize_car_properties(ctx)

    rclpy.init()

    minimal_publisher = MinimalPublisher(ctx)

    rclpy.spin(minimal_publisher)


class MinimalPublisher(Node):
    def __init__(self, ctx: Context):
        super().__init__("minimal_publisher")
        self._action_client = ActionClient(self, DriveTo, "drive_to")

        self.context = ctx

        # todo - get value from somewhere...
        self.current_location = (0, 0)

        self._action_server = ActionServer(
            self,
            DriveToStation,
            'driveToStation',
            self.execute_drive_to)

    async def execute_drive_to(self, goal_handle):
        self.get_logger().info('Executing goal...')
        target_soc = goal_handle.request.target_soc

        green_energy = goal_handle.request.green_energy
        cost_per_kwh = goal_handle.request.cost_per_kwh
        charging_wattage = goal_handle.request.charging_wattage
        max_km = goal_handle.request.max_km

        set_car_properties(self.ctx, green_energy, cost_per_kwh, charging_wattage, max_km)

        station_id, station_property, time_frame = await fetch_stations(self.ctx, self.current_location)

        if not station_id == "NO STATION":
            await register_at_station(self.ctx, station_id, time_frame)

        self.drove_to_station_future = asyncio.Future()
        self.drive_to_station(station_property.geo_point[0], station_property.geo_point[1])

        drove_to_result = await self.drove_to_station_future
        self.get_logger().info(f"drove_to_result: {drove_to_result}")

        #todo - call start charging

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
