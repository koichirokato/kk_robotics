import json
import time

import zenoh
from kk_robotics.simulator import simulator


def main() -> None:
    simulator_impl = simulator.Simulator.create_default()

    def _cmd_vel_callback(msg: zenoh.Sample) -> None:
        print(msg)
        cmd_vel = json.loads(msg.payload.to_string())
        simulator_impl.set_velocity(cmd_vel["linear"], cmd_vel["angular"])

    z = zenoh.open(zenoh.Config())
    z.declare_subscriber("cmd_vel", _cmd_vel_callback)

    try:
        while True:
            simulator_impl.update()
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        z.close()


if __name__ == "__main__":
    main()
