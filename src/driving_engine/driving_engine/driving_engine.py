import math

from .algorithms import drive_to
from .driving_command import DrivingCommand
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

        driving_radius = 6

        last_point = (0, 0)
        updated_path = []
        for point in path[1::]:
            updated_path.append((last_point, (point[0] - last_point[0], point[1] - last_point[1])))
            last_point = point
        updated_path.append((last_point, updated_path[-1][1]))

        last_point = updated_path[0]
        final_path = []
        for point in updated_path[1::]:
            new_start, new_next, angle = drive_to(last_point[0], last_point[1], point[0], point[1], driving_radius)

            straight_distance = distance(last_point, new_start)
            final_path.append(DrivingCommand(straight_distance, None))

            if angle == 0:
                straight_distance = distance(new_start, new_next)
                final_path.append(DrivingCommand(straight_distance, None))
            else:
                angle_distance = (2 * driving_radius * math.pi * angle) / (2 * math.pi)
                final_path.append(DrivingCommand(angle_distance, angle))

        # TODO perform instructions via ros

def distance(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def main(args=None):
    rclpy.init(args=args)

    action_client = DrivingEngine()

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
