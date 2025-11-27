import math
import time

from kk_robotics.simulator import robot
from kk_robotics.simulator import simulator
from kk_robotics.simulator import vizualizer
from kk_robotics.simulator import world


def main() -> None:
    size = 100
    world_impl = world.World(size)
    robot_impl = robot.Robot(10, 10, math.pi / 2)
    vizualizer_impl = vizualizer.SimulatorVisualizer(size)
    simulator_impl = simulator.Simulator(world_impl, robot_impl, vizualizer_impl)

    for _ in range(10000):
        simulator_impl.set_velocity(0.5, 0.5)
        simulator_impl.update(0.5)
        time.sleep(0.05)


if __name__ == "__main__":
    main()
