import math
import time

from kk_robotics import pose_util
from kk_robotics.controller import tk_joy
from kk_robotics.simulator import robot
from kk_robotics.simulator import sensor
from kk_robotics.simulator import simulator
from kk_robotics.simulator import vizualizer
from kk_robotics.simulator import world


def main() -> None:
    size = 100.0
    world_impl = world.World(size)
    robot_impl = robot.Robot(10, 10, math.pi / 2)
    joy_stick_impl = tk_joy.MousePosition()

    sensor_impl = sensor.LiDAR(
        sensor.LiDARSpecifications(
            0.1,
            10.0,
            -math.pi / 4,
            5 * math.pi / 4,
            math.radians(0.125),
        )
    )
    robot_impl.add_sensor(
        robot.SensorOnRobot("front_lidar", pose_util.Pose2D(0.0, 0.0, 0.0), sensor_impl)
    )
    vizualizer_impl = vizualizer.Visualizer(size)
    simulator_impl = simulator.Simulator(world_impl, robot_impl, vizualizer_impl)

    for _ in range(10000):
        mous_pose = joy_stick_impl.get_position()
        simulator_impl.set_velocity(mous_pose[1] / 10.0, -math.atan2(mous_pose[0], mous_pose[1]))
        simulator_impl.update(0.5)
        joy_stick_impl.update()
        time.sleep(0.05)


if __name__ == "__main__":
    main()
