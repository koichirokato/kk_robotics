from kk_robotics.launcher import launcher
from kk_robotics.simulator import sensor
from kk_robotics.simulator import simulator
from kk_robotics.vizualizer import vizualizer


def main() -> None:
    import random
    import time

    simulator_impl = simulator.Simulator.create_default()
    size = 100
    sensor_name = "front_lidar"

    vizualizer_impl = vizualizer.Visualizer(size)

    for i in range(10000):
        if i % 100 == 0:
            v = 0.5 * random.random()
            omega = 0.2 * random.random()
        simulator_impl.set_velocity(v, omega)
        simulator_impl.update(1.0)

        vizualizer_impl.set_robot_pose(simulator_impl.get_pose())
        sensor_on_robot = simulator_impl.get_sensor(sensor_name)
        if sensor_on_robot is not None and isinstance(
            sensor_on_robot.sensor.specifications, sensor.LiDARSpecifications
        ):
            vizualizer_impl.set_lidar_specifications(
                sensor_name,
                sensor_on_robot.sensor.specifications,
                sensor_on_robot.sensor.distances,
            )
        vizualizer_impl.set_cmd_vel((v, 0.0, omega))
        vizualizer_impl.update()
        time.sleep(0.05)


if __name__ == "__main__":
    import time

    launcher_impl = launcher.Launcher()
    vizualizer_impl = vizualizer.Visualizer(100)
    launcher_impl.set_vizualizer(vizualizer_impl)

    for i in range(10000):
        launcher_impl.step()
        time.sleep(0.05)
