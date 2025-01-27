import math
from typing import List

from .algorithms import drive_to
from .driving_command import DrivingCommand
from custom_action_interfaces.action import DriveTo
from custom_action_interfaces.srv import GetPath
from custom_action_interfaces.srv import MotionControl
from custom_action_interfaces.msg import Location

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node


class DrivingEngine(Node):
    def __init__(self):
        super().__init__("drive_to_action_client")

        self._get_path_client = self.create_client(GetPath, "get_path")
        self._action_server = ActionServer(
            self, DriveTo, "drive_to", self.execute_callback
        )

    def send_command(
        self, override=True, angle=0.0, arc_length=0.0, velocity=0.8, cmd_id=0
    ):
        msg = MotionControl.Request()
        msg.id = cmd_id
        msg.velocity = int(velocity * 1000)  # velocity m/s -> mm/s
        msg.arc_length = int(arc_length * 1000)  # arcLength m -> mm
        msg.angle = int(angle * 1000)  # angle rad -> mrad
        msg.override = override
        self.get_logger().info(f"Sending command: {msg}")
        self.future = self.client.call_async(msg)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=self.timeout)
        if self.future.done():
            try:
                response = self.future.result()
                self.get_logger().info(f"Response: {response}")
                return response
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
        else:
            self.get_logger().error("Service call failed: timeout")
        return None

    async def execute_callback(self, goal_handle):
        self.get_logger().info("got drive request")

        # TODO should represent real car data
        current_position = (15, 15)
        current_angle = math.pi
        driving_radius = 6

        target_station_x = goal_handle.request.target_station_x
        target_station_y = goal_handle.request.target_station_y

        path = await self.fetch_path(
            current_position[0],
            current_position[1],
            current_angle,
            target_station_x,
            target_station_y,
        )

        last_point = current_position
        updated_path = []
        for point in path[1::]:
            updated_path.append(
                (last_point, (point[0] - last_point[0], point[1] - last_point[1]))
            )
            last_point = point
        updated_path.append((last_point, updated_path[-1][1]))

        current_direction = (math.cos(current_angle), math.sin(current_angle))
        updated_path[0][1] = current_direction

        last_point = updated_path[0]
        final_path = []
        for point in updated_path[1::]:
            new_start, new_next, angle = drive_to(
                last_point[0], last_point[1], point[0], point[1], driving_radius
            )

            straight_distance = distance(last_point, new_start)
            final_path.append(DrivingCommand(straight_distance, None))

            if angle == 0:
                straight_distance = distance(new_start, new_next)
                final_path.append(DrivingCommand(straight_distance, None))
            else:
                angle_distance = (2 * driving_radius * math.pi * angle) / (2 * math.pi)
                final_path.append(DrivingCommand(angle_distance, angle))

        for instruction in final_path:
            if instruction.angle is None:
                instruction.angle = 0

            self.send_command(
                angle=instruction.angle,
                arc_length=instruction.distance,
            )

        goal_handle.succeed()
        result = DriveTo.Result()

        result.status_code = 404

        return result

    async def fetch_path(
        self,
        x: int,
        y: int,
        car_rotation: float,
        target_station_x: float,
        target_station_y: float,
    ) -> List[Location]:
        get_path_req = GetPath.Request()

        get_path_req.x = x
        get_path_req.y = y
        get_path_req.rotation = car_rotation
        get_path_req.target_x = target_station_x
        get_path_req.target_y = target_station_y

        while not self._get_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("GetPath service not available, waiting again...")

        print("call")
        response = await self._get_path_client.call_async(get_path_req)
        print(response)
        path = response.path

        return path


def distance(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def main(args=None):
    rclpy.init(args=args)

    action_client = DrivingEngine()

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
