import json
import math
import time

import zenoh
from kk_robotics.controller import tk_joy


def main() -> None:
    z = zenoh.open(zenoh.Config())
    joy = tk_joy.MousePosition()

    try:
        while True:
            x, y = joy.get_position()
            data = {"linear": y / 10.0, "angular": -math.atan2(x, y)}

            z.put("cmd_vel", json.dumps(data))
            joy.update()
            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        z.close()


if __name__ == "__main__":
    main()
