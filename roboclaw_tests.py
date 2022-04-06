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


def forward_speed(roboclaw):

    direction = 'forward'
    speed = 0
    while True:

        speed += 1
        if speed == 20:
            speed = 0
            if direction == 'forward':
                direction = 'backward'
            else:
                direction = 'forward'
        if direction == 'forward':
            #roboclaw.ForwardM1(0x80, speed)
            #roboclaw.ForwardM2(0x80, speed)
            roboclaw.SpeedM1(0x80, speed * 1000)
            roboclaw.SpeedM2(0x80, speed * 1000)
        else:
            #roboclaw.BackwardM1(0x80, speed)
            #roboclaw.BackwardM2(0x80, speed)
            roboclaw.SpeedM1(0x80, speed * -1000)
            roboclaw.SpeedM2(0x80, speed * -1000)
        for i in range(1,3):
            print(roboclaw.ReadSpeedM1(0x80))
            print(roboclaw.ReadSpeedM2(0x80))
            print(str(speed) + ' ' + direction)
            print('  ---  ')
            time.sleep(0.2)


if __name__ == '__main__':
    roboclaw = Roboclaw('/dev/ttyACM0', 38400)
    roboclaw.Open()
    try:
        forward_speed(roboclaw)
    except KeyboardInterrupt:
        print('Interrupted')
        roboclaw.ForwardM1(0x80, 0)
        roboclaw.ForwardM2(0x80, 0)
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)

