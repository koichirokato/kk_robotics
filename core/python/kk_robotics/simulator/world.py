import random

import numpy as np


class World:
    def __init__(self, size: int) -> None:
        self._size: int = size
        self._grid: np.ndarray = np.zeros((size, size))
        self._obstacles: set[tuple[int, int]] = set()
        self._generate_obstacles()

    def _generate_obstacles(
        self, wall_count: int = 10, wall_length_range: tuple[int, int] = (10, 30)
    ) -> None:
        for _ in range(wall_count):
            x = random.randint(0, self._size - 1)
            y = random.randint(0, self._size - 1)
            length = random.randint(*wall_length_range)
            horizontal = random.choice([True, False])
            for i in range(length):
                ox = x + i if horizontal else x
                oy = y if horizontal else y + i
                if 0 <= ox < self._size and 0 <= oy < self._size:
                    self._update_obstacle(ox, oy)

        # Fixed walls for structure
        for i in range(50):
            self._update_obstacle(25 + i, 50)
            self._update_obstacle(50, 25 + i)

        for i in range(self._size):
            self._update_obstacle(0, i)
            self._update_obstacle(i, 0)
            self._update_obstacle(self._size - 1, i)
            self._update_obstacle(i, self._size - 1)

    @property
    def grid(self) -> np.ndarray:
        return self._grid

    def _update_obstacle(self, x: int, y: int) -> None:
        if 0 <= x < self._size and 0 <= y < self._size:
            self._grid[x, y] = 1.0
            self._obstacles.add((x, y))

    def obstacles(self) -> set[tuple[int, int]]:
        return self._obstacles

    def is_obstacle(self, x: int, y: int) -> bool:
        return (x, y) in self._obstacles
