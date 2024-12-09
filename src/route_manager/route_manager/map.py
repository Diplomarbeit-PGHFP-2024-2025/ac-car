import json
import os

from ament_index_python.packages import get_package_share_directory


class Obstacle:
    def __init__(self, rotation: float, x: int, y: int, width: int, height: int):
        self.rotation = rotation
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def __repr__(self):
        return (f"Obstacle(rotation={self.rotation}, x={self.x}, "
                f"y={self.y}, width={self.width}, height={self.height})")


class Station:
    def __init__(self, obstacle: Obstacle, opening_x: int, opening_y: int):
        self.obstacle = obstacle
        self.opening_x = opening_x
        self.opening_y = opening_y

    def __repr__(self):
        return f"Station(obstacle={self.obstacle}, opening_x={self.opening_x}, opening_y={self.opening_y})"


class Map:
    obstacles: list[Obstacle]
    stations: list[Station]

    def read_file(self):
        package_path = get_package_share_directory('route_manager')
        json_file_path = os.path.join(package_path, 'map.json')

        with open(json_file_path, 'r') as file:
            data = json.load(file)

        self.obstacles = []
        self.stations = []

        for obstacle in data["obstacles"]:
            self.obstacles.append(
                Obstacle(obstacle["rotation"], obstacle["x"], obstacle["y"], obstacle["width"], obstacle["height"]))

        for station in data["stations"]:
            obstacle = Obstacle(station["rotation"], station["x"], station["y"], station["width"], station["height"])
            self.stations.append(Station(obstacle, station["opening"]["x"], station["opening"]["y"]))

    def get_obstacles(self) -> list[Obstacle]:
        data = []
        data.extend(self.obstacles)

        for station in self.stations:
            data.append(station.obstacle)

        return data

    def get_stations(self) -> list[Station]:
        return self.stations
