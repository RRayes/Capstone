from roboclaw.roboclaw_3 import Roboclaw
import time
import os
import sys
import traceback
import statistics


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

        speed += 20
        if speed == 120:
            speed = 0
            if direction == 'forward':
                direction = 'backward'
            else:
                direction = 'forward'
        if direction == 'forward':
            roboclaw.ForwardM1(0x80, speed)
            roboclaw.ForwardM2(0x80, speed)
            #roboclaw.SpeedM1(0x80, speed * 1000)
            #roboclaw.SpeedM2(0x80, speed * 1000)
        else:
            roboclaw.BackwardM1(0x80, speed)
            roboclaw.BackwardM2(0x80, speed)
            #roboclaw.SpeedM1(0x80, speed * -1000)
            #roboclaw.SpeedM2(0x80, speed * -1000)
        for i in range(1,3):
            print(roboclaw.ReadSpeedM1(0x80))
            print(roboclaw.ReadSpeedM2(0x80))
            print(str(speed) + ' ' + direction)
            print('  ---  ')
            time.sleep(0.2)


def millis():
    return round(time.time() * 1000)


def turn_speed(roboclaw):
    turn_speed = 100
    power_per_second = 20
    l_power = 20
    r_power = -20
    now = 0

    while True:
        measurement_count = 2
        speed_l_list = []
        speed_r_list = []
        for i in range(0, measurement_count):
            speed_l_list.append(roboclaw.ReadSpeedM1(0x80)[1])
            speed_r_list.append(roboclaw.ReadSpeedM2(0x80)[1])
            time.sleep(0.001)
        speed_l = statistics.median(speed_l_list)
        speed_r = statistics.median(speed_r_list)
        print(speed_l, speed_r, l_power, r_power)

        old_now = now
        now = millis()
        elapsed = 0 if old_now == 0 else now - old_now

        if speed_l > turn_speed:
            l_power = l_power - (power_per_second * (elapsed / 1000))
        else:
            l_power = l_power + (power_per_second * (elapsed / 1000))

        if speed_r > (turn_speed * -1):
            r_power = r_power - (power_per_second * (elapsed / 1000))
        else:
            r_power = r_power + (power_per_second * (elapsed / 1000))

        left_speed = l_power
        right_speed = r_power

        if left_speed > 0:
            roboclaw.ForwardM1(0x80, min(127, int(left_speed)))
        else:
            roboclaw.BackwardM1(0x80, min(127, abs(int(left_speed))))

        if right_speed > 0:
            roboclaw.ForwardM2(0x80, min(127, int(right_speed)))
        else:
            roboclaw.BackwardM2(0x80, min(127, abs(int(right_speed))))


if __name__ == '__main__':
    roboclaw = Roboclaw('/dev/ttyACM0', 38400)
    roboclaw.Open()
    try:
        turn_speed(roboclaw)
    except (Exception, KeyboardInterrupt, SystemExit) as e:
        print(traceback.format_exc())
        print(e)
        print('Interrupted')
        roboclaw.ForwardM1(0x80, 0)
        roboclaw.ForwardM2(0x80, 0)
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)

