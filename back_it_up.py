from roboclaw.roboclaw_3 import Roboclaw
import time
import os
import sys


def back(roboclaw):
    while True:
        roboclaw.BackwardM1(0x80, 127)
        roboclaw.BackwardM2(0x80, 127)
        time.sleep(0.05)


if __name__ == '__main__':
    roboclaw = Roboclaw('/dev/ttyACM0', 38400)
    roboclaw.Open()
    try:
        back(roboclaw)
    except KeyboardInterrupt:
        print('Interrupted')
        roboclaw.ForwardM1(0x80, 0)
        roboclaw.ForwardM2(0x80, 0)
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)

