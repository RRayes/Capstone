from roboclaw.roboclaw_3 import Roboclaw
import time
import os
import sys


def forward_back(roboclaw):
    for i in range(0, 128):
        roboclaw.ForwardM1(0x80, i)
        roboclaw.ForwardM2(0x80, i)
        time.sleep(0.05)

    for i in reversed(range(0, 128)):
        roboclaw.ForwardM1(0x80, i)
        roboclaw.ForwardM2(0x80, i)
        time.sleep(0.05)

    for i in range(0, 128):
        roboclaw.BackwardM1(0x80, i)
        roboclaw.BackwardM2(0x80, i)
        time.sleep(0.05)

    for i in reversed(range(0, 128)):
        roboclaw.BackwardM1(0x80, i)
        roboclaw.BackwardM2(0x80, i)
        time.sleep(0.05)


if __name__ == '__main__':
    roboclaw = Roboclaw('/dev/ttyACM0', 38400)
    roboclaw.Open()
    try:
        forward_back(roboclaw)
    except KeyboardInterrupt:
        print('Interrupted')
        roboclaw.ForwardM1(0x80, 0)
        roboclaw.ForwardM2(0x80, 0)
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)

