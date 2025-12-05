import abc
import dataclasses
import math

from kk_robotics import pose_util
from kk_robotics.simulator import world


def to_angles(lidar_specifications: "LiDARSpecifications") -> list[float]:
    return [
        lidar_specifications.angle_min + i * lidar_specifications.angle_increment
        for i in range(lidar_specifications.num_beams)
    ]


@dataclasses.dataclass
class SensorSpecifications:
    range_min: float
    range_max: float


class SensorInterface(abc.ABC):
    @property
    @abc.abstractmethod
    def distances(self) -> list[float]:
        """Distances from sensor measurement"""
        pass

    @property
    @abc.abstractmethod
    def specifications(self) -> SensorSpecifications:
        pass

    @abc.abstractmethod
    def measure(self, robot_pose: pose_util.Pose2D, world: world.World) -> None:
        """Perform measurement using the world and robot pose."""
        pass


@dataclasses.dataclass
class LiDARSpecifications(SensorSpecifications):
    angle_min: float
    angle_max: float
    angle_increment: float

    @property
    def fov(self) -> float:
        return self.angle_max - self.angle_min

    @property
    def num_beams(self) -> int:
        if self.angle_increment == 0:
            return 1
        return max(1, int(round(self.fov / self.angle_increment)) + 1)


class LiDAR(SensorInterface):
    def __init__(self, specification: LiDARSpecifications) -> None:
        self._specification = specification
        self._distances: list[float] = []

    @property
    def distances(self) -> list[float]:
        return self._distances

    @property
    def specifications(self) -> LiDARSpecifications:
        return self._specification

    def measure(self, robot_pose: pose_util.Pose2D, world: world.World) -> None:
        self._distances = []
        for i in range(self._specification.num_beams):
            beam_angle = self._specification.angle_min + i * self._specification.angle_increment
            global_angle = robot_pose.theta + beam_angle - math.pi / 2
            distance = self._cast_ray(
                robot_pose.x,
                robot_pose.y,
                global_angle,
                world,
            )
            self._distances.append(distance)

    def _cast_ray(self, x: float, y: float, angle: float, world: world.World) -> float:
        dx = math.cos(angle)
        dy = math.sin(angle)

        grid_x, grid_y = world.world_to_grid(x, y)
        if world.is_obstacle(grid_x, grid_y):
            return 0.0

        current_x = x
        current_y = y
        for i in range(int(self._specification.range_max / world.resolution)):
            current_x += dx * world.resolution
            current_y += dy * world.resolution
            grid_x, grid_y = world.world_to_grid(current_x, current_y)
            if world.is_obstacle(grid_x, grid_y):
                distance = math.sqrt((current_x - x) ** 2 + (current_y - y) ** 2)
                return distance
        return math.inf
