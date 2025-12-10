import math
import random

import numpy as np


class World:
    def __init__(self, size_m: float, resolution: float = 0.1) -> None:
        # size_m is size of this world [m]
        # resolition [m/cell]
        self._resolution = resolution
        self._number_of_grid_x = int(size_m / resolution)
        self._number_of_grid_y = int(size_m / resolution)
        self._grid: np.ndarray = np.zeros((self._number_of_grid_x, self._number_of_grid_y))
        self._obstacles: set[tuple[int, int]] = set()
        self._generate_obstacles()

    def _generate_obstacles(
        self, wall_count: int = 10, wall_length_range: tuple[int, int] = (10, 30)
    ) -> None:
        for _ in range(wall_count):
            x = random.randint(0, self._number_of_grid_x - 1)
            y = random.randint(0, self._number_of_grid_y - 1)
            min_length = int(wall_length_range[0] / self._resolution)
            max_length = int(wall_length_range[1] / self._resolution)
            length = random.randint(min_length, max_length)
            horizontal = random.choice([True, False])
            for i in range(length):
                ox = x + i if horizontal else x
                oy = y if horizontal else y + i
                if 0 <= ox < self._number_of_grid_x and 0 <= oy < self._number_of_grid_y:
                    self._update_obstacle(ox, oy)

        # Fixed walls for structure
        vertical_start_x = int(self._number_of_grid_x / 4)
        vertical_y = int(self._number_of_grid_y / 2)
        vertical_length = int(self._number_of_grid_x / 2)

        horizontal_x = int(self._number_of_grid_x / 2)
        horizontal_start_y = int(self._number_of_grid_y / 4)
        horizontal_length = int(self._number_of_grid_y / 2)
        for i in range(vertical_length):
            self._update_obstacle(vertical_start_x + i, vertical_y)

        for i in range(horizontal_length):
            self._update_obstacle(horizontal_x, horizontal_start_y + i)

        # World border
        for i in range(self._number_of_grid_x):
            self._update_obstacle(i, 0)
            self._update_obstacle(i, self._number_of_grid_x - 1)

        for i in range(self._number_of_grid_y):
            self._update_obstacle(0, i)
            self._update_obstacle(self._number_of_grid_y - 1, i)

    @property
    def grid(self) -> np.ndarray:
        return self._grid

    @property
    def resolution(self) -> float:
        return self._resolution

    def _update_obstacle(self, x: int, y: int) -> None:
        if 0 <= x < self._number_of_grid_x and 0 <= y < self._number_of_grid_y:
            self._grid[x, y] = 1.0
            self._obstacles.add((x, y))

    def obstacles(self) -> set[tuple[int, int]]:
        return self._obstacles

    def is_obstacle(self, x: int, y: int) -> bool:
        return (x, y) in self._obstacles

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        gx = int(math.floor(x / self._resolution))
        gy = int(math.floor(y / self._resolution))
        return gx, gy

    def grid_to_world(self, gx: float, gy: float) -> tuple[float, float]:
        x = gx * self._resolution
        y = gy * self._resolution
        return x, y
