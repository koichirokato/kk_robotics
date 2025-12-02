import math

import numpy as np


class CostMap2D:
    def __init__(self, width: int, height: int, resolution: float) -> None:
        self._width = width
        self._height = height
        self._resolution = resolution
        self._map = np.zeros((height, width), dtype=np.uint8)

    def _world_to_map(self, x: float, y: float) -> tuple[int, int]:
        mx = int(x / self._resolution + self._width / 2)
        my = int(y / self._resolution + self._height / 2)
        return mx, my

    def set_lidar(
        self, distances: list[float], angles: list[float], max_range: float
    ) -> None:
        self._map.fill(0)

        for distance, angle in zip(distances, angles):
            if 0 < distance < max_range:
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                mx, my = self._world_to_map(x, y)

                if 0 <= mx < self._width and 0 <= my < self._height:
                    self._map[my, mx] = 255

        self._inflate(3)

    def _inflate(self, radius: int) -> None:
        inflated = self._map.copy()
        for y in range(self._height):
            for x in range(self._width):
                if self._map[y, x] == 255:
                    y_min = max(0, y - radius)
                    y_max = min(self._height, y + radius)
                    x_min = max(0, x - radius)
                    x_max = min(self._width, x + radius)
                    inflated[y_min:y_max, x_min:x_max] = np.maximum(
                        inflated[y_min:y_max, x_min:x_max], 150
                    )

        self._map = inflated

    @property
    def costmap(self) -> np.ndarray:
        return self._map
