import math
import tkinter as tk


class SimulatorVisualizer:
    def __init__(self, size: int, scale: float = 5.0) -> None:
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
        self._robot_pose = [0.0, 0.0, 0.0]

    def draw_world(self, obstacles: set[tuple[int, int]]) -> None:
        for x, y in obstacles:
            x0, y0 = self._convert_to_canvas_coodinate(x, y)
            x1, y1 = self._convert_to_canvas_coodinate(x + 1, y + 1)
            self._canvas.create_rectangle(x0, y1, x1, y0, fill="black", outline="")

        scale = int(self._scale)
        width = self._size * scale
        height = self._size * scale
        for x in range(0, width, scale):
            self._canvas.create_line(x, 0, x, height * self._scale, fill="light gray")
        for y in range(0, height, scale):
            self._canvas.create_line(0, y, width * self._scale, y, fill="light gray")

    def set_robot_pose(self, pose: tuple[float, float, float]) -> None:
        self._robot_pose = pose

    def update(self) -> None:
        self._draw_robot()
        self._root.update()

    def _convert_to_canvas_coodinate(self, x: float, y: float) -> tuple[float, float]:
        return x * self._scale, (self._size - y) * self._scale

    def _draw_robot(self) -> None:
        x = self._robot_pose[0]
        y = self._robot_pose[1]
        theta = self._robot_pose[2]

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
