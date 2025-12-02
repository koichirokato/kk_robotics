import time

from kk_robotics import pose_util
from kk_robotics.navigation.planner import global_planner
from kk_robotics.navigation.planner import local_planner
from kk_robotics.simulator import simulator
from kk_robotics.vizualizer import vizualizer


def main() -> None:
    simulator_impl = simulator.Simulator.create_default()
    planner_impl = global_planner.Planner()
    planner_impl.set_world(simulator_impl.get_world())

    vizualizer_impl = vizualizer.Visualizer(100)
    global_path = planner_impl.plan_goal((10.0, 10.0), (80.0, 80.0))
    vizualizer_impl.set_global_path(global_path)
    vizualizer_impl.update()

    local_planner_impl = local_planner.PurePursuit(1, 0.5)

    sensor_name = "front_lidar"
    path_index = 0
    while path_index < len(global_path):
        path = global_path[path_index]
        robot_pose = simulator_impl.get_pose()

        linear_velocity, angular_velocity = local_planner_impl.compute_control(
            (robot_pose.x, robot_pose.y, robot_pose.theta), global_path
        )

        simulator_impl.set_velocity(linear_velocity, angular_velocity)
        simulator_impl.update(0.5)
        vizualizer_impl.set_robot_pose(simulator_impl.get_pose())
        sensor = simulator_impl.get_sensor(sensor_name)
        if sensor is not None:
            vizualizer_impl.set_lidar_specifications(
                sensor.name, sensor.sensor.specifications, sensor.sensor.distances
            )
        vizualizer_impl.update()
        dist = pose_util.distance((robot_pose.x, robot_pose.y), path)
        if dist < 0.1:
            path_index += 1
        time.sleep(0.05)


if __name__ == "__main__":
    main()
