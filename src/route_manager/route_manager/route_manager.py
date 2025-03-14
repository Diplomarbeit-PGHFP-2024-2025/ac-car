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
        x = int(round(request.x))
        y = int(round(request.y))
        target_station_x = request.target_x
        target_station_y = request.target_y
        car_rotation = request.rotation

        self.get_logger().info(
            "stating path calculation goal... x:{} y:{} rotation:{} target_station_x:{}  target_station_y:{}".format(
                x, y, car_rotation, target_station_x, target_station_y
            )
        )
        ac_map_data = MapData()
        ac_map_data.read_file()

        ac_map = Map(ac_map_data)

        station_index = ac_map.get_station_id_from_cord(
            target_station_x, target_station_y
        )

        if station_index == -1:
            print("no station found...")
            response.path = []
            return response

        path = ac_map.get_path(
            (round(math.cos(car_rotation)), round(math.sin(car_rotation))),
            Point(x // ac_map_data.cell_size, y // ac_map_data.cell_size),
            station_index,
        )

        if path is not None:
            path = ac_map.simplify_path(path)
            print(ac_map.display_path(path))
        else:
            print(ac_map)
            print("no path found...")

            response.path = []
            return response

        path_points = []
        for path_point in path:
            path_points.append(
                Location(
                    point=[
                        path_point.position.x * ac_map_data.cell_size,
                        path_point.position.y * ac_map_data.cell_size,
                    ]
                )
            )

        response.path = path_points
        return response


def main(args=None):
    rclpy.init(args=args)

    path_action_server = PathActionServer()
    rclpy.spin(path_action_server)


if __name__ == "__main__":
    main()
