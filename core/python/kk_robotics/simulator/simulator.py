import math

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

    @classmethod
    def create_default(cls, size: int = 100) -> "Simulator":
        """Create a simulator with default robot and LiDAR"""
        world_impl = world.World(size)
        robot_impl = robot.Robot(10, 10, 0.0)

        lidar_spec = sensor.LiDARSpecifications(
            range_min=0.1,
            range_max=10.0,
            angle_min=math.radians(-45),
            angle_max=math.radians(225),
            angle_increment=math.radians(0.125),
        )
        lidar = sensor.LiDAR(lidar_spec)
        robot_impl.add_sensor(
            robot.SensorOnRobot(
                name="front_lidar",
                relative_pose=pose_util.Pose2D(0.0, 0.0, 0.0),
                sensor=lidar,
            )
        )

        vizualizer_impl = vizualizer.SimulatorVisualizer(size)
        return cls(world_impl, robot_impl, vizualizer_impl)
