import math

from kk_robotics import pose_util
from kk_robotics.simulator import world


class Robot:
    def __init__(self, x: float = 10.0, y: float = 10.0, theta: float = 0.0) -> None:
        self._robot_pose = pose_util.Pose2D(x, y, theta)
        self._v = 0.0
        self._omega = 0.0
        self._radius = 0.5

    def set_velocity(self, linear: float, angular: float) -> None:
        self._v = linear
        self._omega = angular

    def update(self, world: world.World, dt: float = 1.0) -> None:
        delta_pose = pose_util.Pose2D(self._v * dt, 0.0, self._omega * dt)
        new_pose = self._robot_pose @ delta_pose

        if not self._collides(new_pose.x, new_pose.y, world):
            self._robot_pose = new_pose
        else:
            self._v = 0.0

    def get_pose(self) -> pose_util.Pose2D:
        return self._robot_pose

    def _collides(self, x: float, y: float, world: world.World) -> bool:
        r = int(math.ceil(self._radius))
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                cx = int(x + dx)
                cy = int(y + dy)
                if world.is_obstacle(cx, cy):
                    return True

        return False
