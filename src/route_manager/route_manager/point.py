import math


class Point:
    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y

    def distance(self, other: 'Point'):
        return math.sqrt(
            (self.x - other.x) ** 2 +
            (self.y - other.y) ** 2
        )

    def __floordiv__(self, other):
        return Point(self.x // other, self.y // other)

    def __repr__(self):
        return f"Point(x={self.x}, y={self.y})"

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
