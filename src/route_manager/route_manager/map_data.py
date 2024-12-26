import math
import json

from .point import Point
import os

from ament_index_python.packages import get_package_share_directory


class Obstacle:
    def __init__(self, rotation: float, center: Point, width: int, height: int):
        self.rotation = rotation
        self.center = center
        self.width = width
        self.height = height

    def is_inside(self, point: Point) -> bool:
        # Translate the point to the rectangle's center
        translated_x = point.x - self.center.x
        translated_y = point.y - self.center.y

        # Rotate the point back by the negative rotation angle
        angle_rad = -self.rotation
        rotated_x = translated_x * math.cos(angle_rad) - translated_y * math.sin(
            angle_rad
        )
        rotated_y = translated_x * math.sin(angle_rad) + translated_y * math.cos(
            angle_rad
        )

        # Check if the rotated point is within the bounds of the rectangle
        half_width = self.width / 2
        half_height = self.height / 2

        return (
            -half_width <= rotated_x <= half_width
            and -half_height <= rotated_y <= half_height
        )

    def __repr__(self):
        return (
            f"Obstacle(rotation={self.rotation}, center={self.center}, "
            f" width={self.width}, height={self.height})"
        )


class Station:
    def __init__(self, obstacle: Obstacle, opening: Point):
        self.obstacle = obstacle
        self.opening = opening

    def __repr__(self):
        return f"Station(obstacle={self.obstacle}, opening={self.opening})"


class MapData:
    _obstacles: list[Obstacle]
    _stations: list[Station]
    width: int
    height: int
    cell_size: int

    def read_file(self):
        package_path = get_package_share_directory("route_manager")
        json_file_path = os.path.join(package_path, "map.json")

        with open(json_file_path, "r") as file:
            data = json.load(file)

        self.width = data["width"]
        self.height = data["height"]
        self.cell_size = data["cell_size"]

        self._obstacles = []
        self._stations = []

        for obstacle in data["obstacles"]:
            self._obstacles.append(
                Obstacle(
                    obstacle["rotation"],
                    Point(obstacle["x"], obstacle["y"]),
                    obstacle["width"],
                    obstacle["height"],
                )
            )

        for station in data["stations"]:
            obstacle = Obstacle(
                station["rotation"],
                Point(station["x"], station["y"]),
                station["width"],
                station["height"],
            )
            self._stations.append(
                Station(
                    obstacle, Point(station["opening"]["x"], station["opening"]["y"])
                )
            )

    def get_obstacles(self) -> list[Obstacle]:
        data = []
        data.extend(self._obstacles)

        for station in self._stations:
            data.append(station.obstacle)

        return data

    def get_stations(self) -> list[Station]:
        return self._stations
