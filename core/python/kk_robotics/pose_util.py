import dataclasses
import math

import numpy as np


@dataclasses.dataclass
class Pose2D:
    x: float
    y: float
    theta: float

    def as_matrix(self) -> np.ndarray:
        s = math.sin(self.theta)
        c = math.cos(self.theta)
        return np.array(
            [
                [c, -s, self.x],
                [s, c, self.y],
                [0, 0, 1.0],
            ]
        )

    def transform_point(self, point: tuple[float, float]) -> tuple[float, float]:
        px, py = point
        vec = np.array([px, py, 1.0])
        res = self.as_matrix() @ vec
        return float(res[0]), float(res[1])

    def __matmul__(self, other: "Pose2D") -> "Pose2D":
        m = self.as_matrix() @ other.as_matrix()
        return Pose2D(
            x=float(m[0, 2]),
            y=float(m[1, 2]),
            theta=float(math.atan2(m[1, 0], m[0, 0])),
        )


def distance(p1: tuple[float, float], p2: tuple[float, float]) -> float:
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])
