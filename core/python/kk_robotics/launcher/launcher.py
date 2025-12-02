import math

from kk_robotics.controller import tk_joy
from kk_robotics.navigation.costmap import costmap2d
from kk_robotics.navigation.planner import global_planner
from kk_robotics.navigation.planner import local_planner
from kk_robotics.simulator import sensor
from kk_robotics.simulator import simulator
from kk_robotics.vizualizer import vizualizer


class Launcher:
    def __init__(self):
        self._simulator: simulator.Simulator = simulator.Simulator.create_default()
        self._controller: tk_joy.MousePosition = tk_joy.MousePosition()
        self._vizualizer: vizualizer.Visualizer | None = None
        self._costmap2d: costmap2d.CostMap2D | None = None
        self._global_planner: global_planner.Planner | None = None
        self._local_planner: local_planner.PurePursuit | None = None

    def set_vizualizer(self, vizualizer: vizualizer.Visualizer) -> None:
        self._vizualizer = vizualizer

    def step(self, dt: float | None = None) -> None:
        mous_pose = self._controller.get_position()

        self._simulator.set_velocity(mous_pose[1] / 10.0, -math.atan2(mous_pose[0], mous_pose[1]))

        if dt is not None:
            self._simulator.update(dt)
        else:
            self._simulator.update()
        self._controller.update()

        if self._vizualizer is not None:
            self._vizualizer.set_robot_pose(self._simulator.get_pose())
            sensor_on_robot = self._simulator.get_sensor("front_lidar")
            if sensor_on_robot is not None and isinstance(
                sensor_on_robot.sensor.specifications, sensor.LiDARSpecifications
            ):
                self._vizualizer.set_lidar_specifications(
                    sensor_on_robot.name,
                    sensor_on_robot.sensor.specifications,
                    sensor_on_robot.sensor.distances,
                )
            self._vizualizer.update()
