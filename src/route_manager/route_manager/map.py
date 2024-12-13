import json
import math
import os
from enum import Enum

from ament_index_python.packages import get_package_share_directory


class Obstacle:
    def __init__(self, rotation: float, x: int, y: int, width: int, height: int):
        self.rotation = rotation
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def is_inside(self, px: int, py: int) -> bool:
        # Translate the point to the rectangle's center
        translated_x = px - self.x
        translated_y = py - self.y

        # Rotate the point back by the negative rotation angle
        angle_rad = -self.rotation
        rotated_x = (translated_x * math.cos(angle_rad) -
                     translated_y * math.sin(angle_rad))
        rotated_y = (translated_x * math.sin(angle_rad) +
                     translated_y * math.cos(angle_rad))

        # Check if the rotated point is within the bounds of the rectangle
        half_width = self.width / 2
        half_height = self.height / 2

        return (-half_width <= rotated_x <= half_width and
                -half_height <= rotated_y <= half_height)

    def __repr__(self):
        return (
            f"Obstacle(rotation={self.rotation}, x={self.x}, "
            f"y={self.y}, width={self.width}, height={self.height})"
        )


class Station:
    def __init__(self, obstacle: Obstacle, opening_x: int, opening_y: int):
        self.obstacle = obstacle
        self.opening_x = opening_x
        self.opening_y = opening_y

    def __repr__(self):
        return f"Station(obstacle={self.obstacle}, opening_x={self.opening_x}, opening_y={self.opening_y})"


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
                    obstacle["x"],
                    obstacle["y"],
                    obstacle["width"],
                    obstacle["height"],
                )
            )

        for station in data["stations"]:
            obstacle = Obstacle(
                station["rotation"],
                station["x"],
                station["y"],
                station["width"],
                station["height"],
            )
            self._stations.append(
                Station(obstacle, station["opening"]["x"], station["opening"]["y"])
            )

    def get_obstacles(self) -> list[Obstacle]:
        data = []
        data.extend(self._obstacles)

        for station in self._stations:
            data.append(station.obstacle)

        return data

    def get_stations(self) -> list[Station]:
        return self._stations


class CellState(Enum):
    EMPTY = 1
    PADDING = 2
    OBSTACLE = 3


class Map:
    _cells: list[list[CellState]]

    def __init__(self, map_data: MapData):
        x_count = int(map_data.width / map_data.cell_size)
        y_count = int(map_data.height / map_data.cell_size)

        self._cells = [[CellState.EMPTY for _ in range(x_count)] for _ in range(y_count)]

        for obstacle in map_data.get_obstacles():
            for x in range(0, x_count):
                for y in range(0, y_count):
                    if obstacle.is_inside(x * map_data.cell_size, y * map_data.cell_size):
                        self._cells[x][y] = CellState.OBSTACLE

        for x in range(0, x_count):
            for y in range(0, y_count):
                if self._cells[x][y] != CellState.EMPTY:
                    continue

                has_neighbor = False

                for x_offset in range(-1, 2):
                    for y_offset in range(-1, 2):
                        x_check = x + x_offset
                        y_check = y + y_offset

                        if x_offset == y_offset and x_offset == 0:
                            continue

                        if x_check < 0 or y_check < 0 or x_check >= x_count or y_check >= y_count:
                            continue

                        if self._cells[x_check][y_check] == CellState.OBSTACLE:
                            has_neighbor = True

                if has_neighbor:
                    self._cells[x][y] = CellState.PADDING

    def __repr__(self):
        result = ""

        width = len(self._cells)
        height = len(self._cells[0])

        for y in range(0, height):
            for x in range(0, width):
                if self._cells[x][y] == CellState.OBSTACLE:
                    result += " # "
                elif self._cells[x][y] == CellState.PADDING:
                    result += " + "
                else:
                    result += " . "
            result += "\n"
        return result
