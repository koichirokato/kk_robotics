import time

from kk_robotics.navigation.costmap import costmap2d
from kk_robotics.simulator import sensor
from kk_robotics.simulator import simulator
from kk_robotics.vizualizer import vizualizer


def main() -> None:
    size = 100
    simulator_impl = simulator.Simulator.create_default(size)
    costmap_impl = costmap2d.CostMap2D(50, 50, 1.0)
    vizualizer_impl = vizualizer.Visualizer(size)

    for _ in range(10000):
        sensor_data = simulator_impl.get_sensor("front_lidar")
        if sensor_data is not None and isinstance(
            sensor_data.sensor.specifications, sensor.LiDARSpecifications
        ):
            angles = sensor.to_angles(sensor_data.sensor.specifications)
            distances = sensor_data.sensor.distances

            costmap_impl.set_lidar(distances, angles, sensor_data.sensor.specifications.range_max)
            vizualizer_impl.set_local_costmap(costmap_impl.costmap)
            vizualizer_impl.set_lidar_specifications(
                sensor_data.name,
                sensor_data.sensor.specifications,
                sensor_data.sensor.distances,
            )

        vizualizer_impl.set_robot_pose(simulator_impl.get_pose())
        simulator_impl.set_velocity(0.5, 0.2)
        vizualizer_impl.update()
        simulator_impl.update()
        time.sleep(0.05)


main()
