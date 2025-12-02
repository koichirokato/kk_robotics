import math
import tkinter as tk

import numpy as np

from kk_robotics import pose_util
from kk_robotics.simulator import sensor


class Visualizer:
    def __init__(self, size: int, scale: float = 10.0) -> None:
        self._scale = scale
        self._size = size

        self._root = tk.Tk()
        self._root.title("Vizualizer")
        size_px = int(size * scale)
        self._width = size_px
        self._height = size_px

        self._canvas = tk.Canvas(
            self._root,
            width=self._width,
            height=self._height,
            bg="#242424",
            highlightthickness=0,
        )
        self._canvas.pack(fill="both", expand=True)

        self._robot_pose_canvas_id: int | None = None
        self._robot_heading_canvas_id: int | None = None
        self._sensor_line_ids: list[int] = []

        self._robot_radius = 2.0
        self._robot_pose = pose_util.Pose2D(x=0.0, y=0.0, theta=0.0)

        self._cmd_vel: tuple[float, float, float] | None = None
        self._cmd_vel_id: int | None = None

        # name, distances, fov, number_of_beams
        self._lidar_specifications: dict[str, sensor.LiDARSpecifications] = {}
        self._lidar_distances = []
        self._lidar_fov: float | None = None
        self._lidar_number_of_beams: int | None = None

        self._global_path: list[tuple[float, float]] = []

        self._local_costmap = np.zeros((size, size), dtype=np.uint8)

        self._grid_ids = []
        self._draw_grid()

    def _draw_grid(self) -> None:
        scale = int(self._scale)
        width = self._width * scale
        height = self._height * scale
        for x in range(0, width, scale):
            self._grid_ids.append(self._canvas.create_line(x, 0, x, self._height, fill="#3A3A3A"))
        for y in range(0, height, scale):
            self._grid_ids.append(self._canvas.create_line(0, y, self._width, y, fill="#3A3A3A"))

    def set_robot_pose(self, pose: pose_util.Pose2D) -> None:
        self._robot_pose = pose

    def set_lidar_specifications(
        self,
        name: str,
        specifications: sensor.LiDARSpecifications,
        distances: list[float],
    ) -> None:
        self._lidar_specifications[name] = specifications
        self._lidar_distances = distances

    def set_cmd_vel(self, cmd_vel: tuple[float, float, float]) -> None:
        self._cmd_vel = cmd_vel

    def set_local_costmap(self, map: np.ndarray) -> None:
        self._local_costmap = map

    def update(self) -> None:
        self._draw_local_costmap()
        self._draw_lidar()
        self._draw_global_path()
        self._draw_robot()
        self._root.update()

    def _convert_to_canvas_coodinate(self, x: float, y: float) -> tuple[float, float]:
        return x * self._scale, (self._size - y) * self._scale

    def _draw_robot(self) -> None:
        x = self._robot_pose.x
        y = self._robot_pose.y
        theta = self._robot_pose.theta
        xc, yc = self._convert_to_canvas_coodinate(x, y)
        r = self._robot_radius * self._scale

        if self._robot_pose_canvas_id is None:
            self._robot_pose_canvas_id = self._canvas.create_oval(
                xc - r, yc - r, xc + r, yc + r, fill="#00FFFF"
            )
        else:
            self._canvas.coords(self._robot_pose_canvas_id, xc - r, yc - r, xc + r, yc + r)

        hx = xc + r * math.cos(theta)
        hy = yc - r * math.sin(theta)
        if self._robot_heading_canvas_id is None:
            self._robot_heading_canvas_id = self._canvas.create_line(
                xc, yc, hx, hy, fill="white", width=2
            )
        else:
            self._canvas.coords(self._robot_heading_canvas_id, xc, yc, hx, hy)

    def _draw_lidar(self) -> None:
        if len(self._lidar_specifications) < 1:
            return
        for line_id in self._sensor_line_ids:
            self._canvas.delete(line_id)
        self._sensor_line_ids.clear()

        x = self._robot_pose.x
        y = self._robot_pose.y
        theta = self._robot_pose.theta
        for i, dist in enumerate(self._lidar_distances):
            if dist == math.inf:
                continue
            beam_angle = (
                self._lidar_specifications["front_lidar"].angle_min
                + i * self._lidar_specifications["front_lidar"].angle_increment
            )
            sensor_angle = theta + beam_angle - math.pi / 2
            x_end = x + dist * math.cos(sensor_angle)
            y_end = y + dist * math.sin(sensor_angle)
            x_end_c, y_end_c = self._convert_to_canvas_coodinate(x_end, y_end)
            lidar_point_size = 0.5 * self._scale
            line_id = self._canvas.create_oval(
                x_end_c,
                y_end_c,
                x_end_c + lidar_point_size,
                y_end_c + lidar_point_size,
                fill="#00FF00",
                outline="",
            )
            self._sensor_line_ids.append(line_id)

    def _draw_global_path(self) -> None:
        if len(self._global_path) < 2:
            return

        for i in range(1, len(self._global_path)):
            current_pose = self._global_path[i]
            previous_pose = self._global_path[i - 1]
            current_xc, current_yc = self._convert_to_canvas_coodinate(*current_pose)
            previous_xc, previous_yc = self._convert_to_canvas_coodinate(*previous_pose)
            self._canvas.create_line(
                current_xc,
                current_yc,
                previous_xc,
                previous_yc,
                fill="#08f",
                width=2,
            )

    def set_global_path(self, path: list[tuple[float, float]]) -> None:
        self._global_path = path

    def _draw_local_costmap(self) -> None:
        # Clear previous costmap cells
        if hasattr(self, "_local_costmap_ids"):
            for rect_id in self._local_costmap_ids:
                self._canvas.delete(rect_id)
        self._local_costmap_ids: list[int] = []

        if self._local_costmap is None:
            return

        rows, cols = self._local_costmap.shape
        center_row = rows / 2
        center_col = cols / 2

        theta = self._robot_pose.theta - math.pi / 2
        current_sin = math.sin(theta)
        current_cos = math.cos(theta)

        for y in range(rows):
            for x in range(cols):
                cost = int(self._local_costmap[y, x])
                if cost == 0:
                    color = "lightgray"
                    continue
                else:
                    brightness = int(40 + (cost / 255.0) * 180)
                    color = f"#{brightness:02x}{brightness:02x}{brightness:02x}"

                x_local = x - center_col
                y_local = y - center_row

                global_x = self._robot_pose.x + x_local * current_cos - y_local * current_sin
                global_y = self._robot_pose.y + x_local * current_sin + y_local * current_cos

                xc, yc = self._convert_to_canvas_coodinate(global_x, global_y)

                size = self._scale
                rect_id = self._canvas.create_rectangle(
                    xc - size / 2,
                    yc - size / 2,
                    xc + size / 2,
                    yc + size / 2,
                    fill=color,
                    outline="",
                )
                self._local_costmap_ids.append(rect_id)

    def draw_points(self, points: list[list[float]], color: str | None = None) -> None:
        size = 10
        if color is None:
            color = "blue"
        for point in points:
            print(point)
            xc, yc = self._convert_to_canvas_coodinate(point[0], point[1])
            print(xc, yc)
            self._canvas.create_oval(
                xc - size / 2, yc - size / 2, xc + size / 2, yc + size / 2, fill=color
            )
        self._root.update()


if __name__ == "__main__":
    import time

    visalizer = Visualizer(10, 100)
    points = [[1, 1], [2, 2], [3, 3], [4, 4]]

    visalizer.draw_points(points)

    for i in range(10):
        time.sleep(1.0)
