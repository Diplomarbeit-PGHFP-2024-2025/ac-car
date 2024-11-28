import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from custom_action_interfaces.action import Fibonacci

from uagents import Context

from .filter_stations import initialize_car_properties
from .fetchAgent import agent
from .communication import fetch_stations


@agent.on_event("startup")
async def introduce_agent(ctx: Context):
    ctx.logger.info(f"Agent: {agent.name} ({agent.address})")
    initialize_car_properties(ctx)
    await fetch_stations(ctx)


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self._action_client = ActionClient(self, Fibonacci, "fibonacci")
        self.send_goal(5)
        agent.run()

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
