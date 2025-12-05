import math
import tkinter as tk

from kk_robotics import pose_util
from kk_robotics.simulator import sensor
from kk_robotics.simulator import world


class SimulatorVisualizer:
    def __init__(self, size: float, scale: float = 5.0) -> None:
        self._scale = scale
        self._size = size

        self._root = tk.Tk()
        self._root.title("2D Robot Simulator (Grid + LiDAR)")
        size_px = int(size * scale)
        self._canvas = tk.Canvas(self._root, width=size_px, height=size_px, bg="white")
        self._canvas.pack()

        self._robot_pose_canvas_id: int | None = None
        self._robot_heading_canvas_id: int | None = None
        self._sensor_line_ids: list[int] = []

        self._robot_radius = 2.0
        self._robot_pose = pose_util.Pose2D(0.0, 0.0, 0.0)

        self._lidar_distances = []
        self._lidar_specifications: sensor.LiDARSpecifications | None = None

    def draw_world(self, world_impl: world.World) -> None:
        for grid_x, grid_y in world_impl.obstacles():
            x, y = world_impl.grid_to_world(grid_x, grid_y)
            x0, y0 = self._convert_to_canvas_coodinate(x, y)
            x1, y1 = self._convert_to_canvas_coodinate(x + 1, y + 1)
            self._canvas.create_rectangle(x0, y1, x1, y0, fill="black", outline="")

        scale = int(self._scale)
        width = int(self._size * scale)
        height = int(self._size * scale)
        for x in range(0, width, scale):
            self._canvas.create_line(x, 0, x, height * self._scale, fill="light gray")
        for y in range(0, height, scale):
            self._canvas.create_line(0, y, width * self._scale, y, fill="light gray")

    def set_robot_pose(self, pose: pose_util.Pose2D) -> None:
        self._robot_pose = pose

    def set_lidar_specifications(
        self, distances: list[float], specifications: sensor.LiDARSpecifications
    ) -> None:
        self._lidar_distances = distances
        self._lidar_specifications = specifications

    def update(self) -> None:
        self._draw_robot()
        self._draw_lidar()
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
                xc - r, yc - r, xc + r, yc + r, fill="blue"
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
        if self._lidar_specifications is None:
            return
        for line_id in self._sensor_line_ids:
            self._canvas.delete(line_id)
        self._sensor_line_ids.clear()

        x = self._robot_pose.x
        y = self._robot_pose.y
        theta = self._robot_pose.theta
        xc, yc = self._convert_to_canvas_coodinate(x, y)
        for i, dist in enumerate(self._lidar_distances):
            beam_angle = (
                self._lidar_specifications.angle_min
                + i * self._lidar_specifications.angle_increment
            )
            sensor_angle = theta + beam_angle - math.pi / 2
            x_end = x + dist * math.cos(sensor_angle)
            y_end = y + dist * math.sin(sensor_angle)

            x_end_c, y_end_c = self._convert_to_canvas_coodinate(x_end, y_end)
            line_id = self._canvas.create_line(
                xc, yc, x_end_c, y_end_c, fill="red", dash=(1, 3), width=5
            )
            self._sensor_line_ids.append(line_id)
