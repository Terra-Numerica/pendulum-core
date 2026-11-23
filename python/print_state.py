#!/usr/bin/env python3
import sys

from pendulum_core import PendulumPyClient

if __name__ == '__main__':
    host = sys.argv[1] if len(sys.argv) > 1 else PendulumPyClient.DEFAULT_HOST

    with PendulumPyClient(wait=5, host=host) as pendulum:
        print(" POSITION       ANGLE      LIN.VEL.     ANG.VEL.")
        # print("12345678901  12345678901  12345678901  12345678901")

        try:
            while True:
                pendulum.read_state(blocking=True)
                print(
                    f"{pendulum.position:0=11.5f}  {pendulum.angle:0=11.5f}  "
                    f"{pendulum.linear_velocity:0=11.5f}  {pendulum.angular_velocity:0=11.5f}",
                    end='\r', flush=True
                )
        except KeyboardInterrupt:
            pass
