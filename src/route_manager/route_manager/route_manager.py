import math

import rclpy
from rclpy.node import Node

from custom_action_interfaces.srv import GetPath
from custom_action_interfaces.msg import Location

from .map import MapData, Map, Point


class PathActionServer(Node):
    def __init__(self):
        super().__init__("path_action_server")
        self.srv = self.create_service(GetPath, "get_path", self.get_path)

    def get_path(self, request, response):
        x = request.x
        y = request.y
        target_station = request.target
        car_rotation = request.rotation

        self.get_logger().info(
            "stating path calculation goal... x:{} y:{} rotation:{} target_station:{} ".format(
                x, y, car_rotation, target_station
            )
        )
        ac_map_data = MapData()
        ac_map_data.read_file()

        ac_map = Map(ac_map_data)

        path = ac_map.get_path(
            (round(math.cos(car_rotation)), round(math.sin(car_rotation))),
            Point(x, y),
            target_station,
        )

        if path is not None:
            path = ac_map.simplify_path(path)
            print(ac_map.display_path(path))
        else:
            print(ac_map)
            print("no path found...")

        path_points = []
        for path_point in path:
            path_points.append(
                Location(point=[path_point.position.x, path_point.position.y])
            )

        response.path = path_points
        return response


def main(args=None):
    rclpy.init(args=args)

    path_action_server = PathActionServer()
    rclpy.spin(path_action_server)


if __name__ == "__main__":
    main()
