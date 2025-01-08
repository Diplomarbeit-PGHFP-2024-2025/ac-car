from enum import Enum
from typing import Tuple, List

from .map_data import MapData, Station
from .point import Point


class CellState(Enum):
    EMPTY = 1
    PADDING = 2
    OBSTACLE = 3


class PathNode:
    def __init__(self, direction: Tuple[int, int] = None, position: Point = None):
        self.position = position
        self.direction = direction

    def __repr__(self):
        direction_map = {
            (0, -1): "↑",
            (0, 1): "↓",
            (-1, 0): "←",
            (1, 0): "→",
            (-1, -1): "↖",
            (-1, 1): "↙",
            (1, -1): "↗",
            (1, 1): "↘",
        }
        return direction_map.get(self.direction, "✦")


def find_path_node(nodes_list, pos: Point):
    for node in nodes_list:
        if node.position == pos:
            return node
    return None


class Node:
    def __init__(
        self,
        direction: Tuple[int, int] = None,
        parent: "Node" = None,
        position: Point = None,
    ):
        self.parent = parent
        self.position = position
        self.direction = direction

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


class Map:
    _cells: list[list[CellState]]
    _stations: List[Station]
    _cell_size: int

    def __init__(self, map_data: MapData):
        x_count = int(map_data.width / map_data.cell_size)
        y_count = int(map_data.height / map_data.cell_size)

        self._cells = [
            [CellState.EMPTY for _ in range(x_count)] for _ in range(y_count)
        ]
        self._stations = map_data.get_stations()
        self._cell_size = map_data.cell_size

        # fill map with obstacle markers
        for obstacle in map_data.get_obstacles():
            for x in range(0, x_count):
                for y in range(0, y_count):
                    if obstacle.is_inside(
                        Point(x * map_data.cell_size, y * map_data.cell_size)
                    ):
                        self._cells[x][y] = CellState.OBSTACLE

        # pad obstacle markers
        for x in range(0, x_count):
            for y in range(0, y_count):
                if self._cells[x][y] != CellState.EMPTY:
                    continue

                has_neighbor = False

                for x_offset in range(-1, 2):
                    if has_neighbor:
                        break

                    for y_offset in range(-1, 2):
                        x_check = x + x_offset
                        y_check = y + y_offset

                        # is center
                        if x_offset == y_offset and x_offset == 0:
                            continue

                        # is outside bounds
                        if (
                            x_check < 0
                            or y_check < 0
                            or x_check >= x_count
                            or y_check >= y_count
                        ):
                            continue

                        if self._cells[x_check][y_check] == CellState.OBSTACLE:
                            has_neighbor = True
                            break

                if has_neighbor:
                    self._cells[x][y] = CellState.PADDING

        # give open path to station center
        for station in self._stations:
            center = station.obstacle.center // map_data.cell_size
            opening = station.opening // map_data.cell_size

            for point in get_straight_line(center, opening):
                self._cells[point.x][point.y] = CellState.EMPTY

    def get_path(
        self, direction: Tuple[int, int], start: Point, goal_station: int
    ) -> List[PathNode] | None:
        start_node = Node(direction, None, start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(
            None, None, self._stations[goal_station].obstacle.center // self._cell_size
        )
        end_node.g = end_node.h = end_node.f = 0

        open_list = []
        closed_list = []

        open_list.append(start_node)

        while len(open_list) > 0:
            # Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # Found the goal
            if current_node == end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(PathNode(current.direction, current.position))
                    current = current.parent
                return path[::-1]  # Return reversed path

            # Generate children
            children = []
            for offset in get_possible_next_directions(
                current_node.direction
            ):  # Adjacent squares
                # Get node position
                node_position = Point(
                    current_node.position.x + offset[0],
                    current_node.position.y + offset[1],
                )

                # Make sure within range
                if (
                    node_position.x > (len(self._cells) - 1)
                    or node_position.x < 0
                    or node_position.y > (len(self._cells[len(self._cells) - 1]) - 1)
                    or node_position.y < 0
                ):
                    continue

                # Make sure walkable terrain
                if self._cells[node_position.x][node_position.y] != CellState.EMPTY:
                    continue

                # Create new node
                new_node = Node(
                    (
                        node_position.x - current_node.position.x,
                        node_position.y - current_node.position.y,
                    ),
                    current_node,
                    node_position,
                )

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:
                # Child is on the closed list
                if child in closed_list:
                    continue

                # Create the f, g, and h values
                child.g = current_node.g + current_node.position.distance(
                    child.position
                )
                child.h = child.position.distance(end_node.position)
                child.f = child.g + child.h

                # Check if this neighbor is in the open list
                existing_node = next(
                    (node for node in open_list if node == child), None
                )

                if existing_node:
                    # If the new path is better, update the existing node
                    if child.g < existing_node.g:
                        existing_node.g = child.g
                        existing_node.f = child.f
                        existing_node.parent = current_node
                else:
                    # Otherwise, add the neighbor to the open list
                    open_list.append(child)

    def simplify_path(self, nodes: List[PathNode]) -> List[PathNode]:
        index = 2

        while True:
            if len(nodes) <= index:
                break

            prev = nodes[index - 2]
            next = nodes[index]

            next_directions = get_possible_next_directions(prev.direction)
            if next.direction not in next_directions:
                index += 1
                continue

            if not self.is_straight_empty_line(next.position, prev.position):
                index += 1
                continue

            nodes.pop(index - 1)

        return nodes

    def is_straight_empty_line(self, a: Point, b: Point) -> bool:
        line_points = get_straight_line(a, b)

        for point in line_points:
            if self._cells[point.x][point.y] != CellState.EMPTY:
                return False

        return True

    def display_path(self, path: List[PathNode]) -> str:
        result = ""

        width = len(self._cells)
        height = len(self._cells[0])

        for y in range(0, height):
            for x in range(0, width):
                path_node = find_path_node(path, Point(x, y))

                if path_node is not None:
                    result += " " + path_node.__repr__() + " "
                elif self._cells[x][y] == CellState.OBSTACLE:
                    result += " # "
                elif self._cells[x][y] == CellState.PADDING:
                    result += " + "
                else:
                    result += " . "
            result += "\n"

        return result

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


def get_straight_line(a: Point, b: Point) -> List[Point]:
    dx = abs(b.x - a.x)
    dy = abs(b.y - a.y)
    sx = 1 if a.x < b.x else -1
    sy = 1 if a.y < b.y else -1
    err = dx - dy

    points = []

    while True:
        points.append(a)
        if a == b:
            break

        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            a = Point(a.x + sx, a.y)
        if e2 < dx:
            err += dx
            a = Point(a.x, a.y + sy)

    return points


def get_possible_next_directions(direction: Tuple[int, int]) -> List[Tuple[int, int]]:
    possible = [(1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1)]

    index = possible.index((direction[0], direction[1]))

    return [possible[index], possible[(index + 1) % len(possible)], possible[index - 1]]
