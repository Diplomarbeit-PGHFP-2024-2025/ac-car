class DrivingCommand:
    distance: float
    angle: float|None

    def __init__(self, distance: float, angle: float|None):
        self.distance = distance
        self.angle = angle