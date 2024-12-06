import rclpy
from rclpy.node import Node

from uagents import Context

from .sort_stations import initialize_car_properties
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
        agent.run()


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
