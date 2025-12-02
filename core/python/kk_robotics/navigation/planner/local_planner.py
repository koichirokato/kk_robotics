import math


class PurePursuit:
    def __init__(self, lookahead_distance: float, base_speed: float) -> None:
        self._lookahead_distance = lookahead_distance
        self._base_speed = base_speed

    def find_target_point(
        self, pose: tuple[float, float, float], path: list[tuple[float, float]]
    ) -> tuple[float, float]:
        for px, py in path:
            dx = px - pose[0]
            dy = py - pose[1]
            distance = math.hypot(dx, dy)
            if distance < self._lookahead_distance:
                continue

            heading_vector = (math.cos(pose[2]), math.sin(pose[2]))
            point_vector = (dx / distance, dy / distance)
            dot = (
                heading_vector[0] * point_vector[0]
                + heading_vector[1] * point_vector[1]
            )
            if dot > 0.3:
                return px, py
        return path[-1]

    def compute_control(
        self, pose: tuple[float, float, float], path: list[tuple[float, float]]
    ) -> tuple[float, float]:
        target = self.find_target_point(pose, path)
        dx = target[0] - pose[0]
        dy = target[1] - pose[1]

        alpha = math.atan2(dy, dx) - pose[2]
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi

        v = self._base_speed
        omega = 2.0 * v * math.sin(alpha) / self._lookahead_distance
        return v, omega
