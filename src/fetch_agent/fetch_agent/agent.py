import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from custom_action_interfaces.action import Path

from uagents import Context

from .sort_stations import initialize_car_properties
from .fetchAgent import agent
from .communication import fetch_stations


@agent.on_event("startup")
async def introduce_agent(ctx: Context):
    ctx.logger.info(f"Agent: {agent.name} ({agent.address})")
    initialize_car_properties(ctx)
    await fetch_stations(ctx)

    rclpy.init()

    minimal_publisher = MinimalPublisher()
    minimal_publisher.fetch_path(15, 15, math.pi, 0)

    rclpy.spin(minimal_publisher)


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self._action_client = ActionClient(self, Path, "path")

    def fetch_path(self, x: int, y: int, car_rotation: float, target_station: int):
        goal_msg = Path.Goal()

        goal_msg.x = x
        goal_msg.y = y
        goal_msg.rotation = car_rotation
        goal_msg.target = target_station

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

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


def main():
    agent.run()


if __name__ == "__main__":
    main()
