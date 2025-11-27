import math

from kk_robotics.simulator import world


class Robot:
    def __init__(self, x: float = 10.0, y: float = 10.0, theta: float = 0.0) -> None:
        self._x = x
        self._y = y
        self._theta = theta
        self._v = 0.0
        self._omega = 0.0
        self._radius = 0.5

    def set_velocity(self, linear: float, angular: float) -> None:
        self._v = linear
        self._omega = angular

    def update(self, world: world.World, dt: float = 1.0) -> None:
        self._theta += self._omega * dt
        new_x = self._x + self._v * math.cos(self._theta) * dt
        new_y = self._y + self._v * math.sin(self._theta) * dt

        if not self._collides(new_x, new_y, world):
            self._x = new_x
            self._y = new_y
        else:
            self._v = 0.0

    def get_pose(self) -> tuple[float, float, float]:
        return self._x, self._y, self._theta

    def _collides(self, x: float, y: float, world: world.World) -> bool:
        r = int(math.ceil(self._radius))
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                cx = int(x + dx)
                cy = int(y + dy)
                if world.is_obstacle(cx, cy):
                    return True

        return False
