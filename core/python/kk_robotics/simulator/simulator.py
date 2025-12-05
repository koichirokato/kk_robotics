import numpy as np

from kk_robotics import pose_util
from kk_robotics.simulator import robot
from kk_robotics.simulator import sensor
from kk_robotics.simulator import vizualizer
from kk_robotics.simulator import world


class Simulator:
    def __init__(
        self,
        world_impl: world.World,
        robot_impl: robot.Robot,
        vizualizer_impl: vizualizer.SimulatorVisualizer,
    ) -> None:
        self._world = world_impl
        self._robot = robot_impl
        self._vizualizer = vizualizer_impl

        self._vizualizer.draw_world(world_impl)

    def set_velocity(self, linear_velocity: float, angular_velocity: float) -> None:
        self._robot.set_velocity(linear_velocity, angular_velocity)

    def get_pose(self) -> pose_util.Pose2D:
        return self._robot.get_pose()

    def get_sensor(self, name: str) -> robot.SensorOnRobot | None:
        return self._robot.get_sensor(name)

    def get_world(self) -> np.ndarray:
        return self._world.grid

    def update(self, dt: float = 0.1) -> None:
        self._robot.update(self._world, dt)
        self._robot.sense(self._world)
        self._vizualizer.set_robot_pose(self._robot.get_pose())

        for sensor_on_robot in self._robot.sensors:
            sensor_interface = sensor_on_robot.sensor
            specification = sensor_on_robot.sensor.specifications
            if isinstance(specification, sensor.LiDARSpecifications):
                self._vizualizer.set_lidar_specifications(sensor_interface.distances, specification)
            else:
                # TODO: support other type
                pass
        self._vizualizer.update()
