from roboclaw.roboclaw_3 import Roboclaw
import os
import sys
import cv2
from pupil_apriltags import Detector
import math
import time
import traceback

STATE_GO_FARTHEST = 'state_go_farthest'
STATE_GO_TURN = 'state_go_turn'
STATE_TURN_LEFT = 'state_turn_left'
STATE_GO_TURN_DELAY = 'state_go_turn_delay'
STATE_GO_HOME = 'state_go_home'

TAG_FORWARD = 1
TAG_TURN_LEFT = 2
TAG_TURN_LEFT_DELAY = 3
TAG_GO_HOME = 4

turn_tags = [TAG_TURN_LEFT, TAG_TURN_LEFT_DELAY]
go_to_turn_states = [STATE_GO_TURN, STATE_GO_TURN_DELAY]

def millis():
    return round(time.time() * 1000)


def rescale(val, in_min, in_max, out_min, out_max):
    return out_min + (val - in_min) * ((out_max - out_min) / (in_max - in_min))


def get_left_right_power_for_tag(tag, frame_width, max_power):
    distance_total = pow(tag.pose_t[0][0],2) + pow(tag.pose_t[1][0],2) + pow(tag.pose_t[2][0],2)
    distance_sqrt = math.sqrt(distance_total)
    forward_speed = int(rescale(distance_sqrt, 0, 3, 0, max_power))
    turning_weight = 0.5
    turning_speed = int(rescale(
        int(frame_width/2) - tag.center[0],
        -1 * (frame_width/2),
        (frame_width/2),
        -1 * (max_power * turning_weight),
        (max_power * turning_weight)))
    left_speed = int(forward_speed - turning_speed)
    right_speed = int(forward_speed + turning_speed)
    return left_speed, right_speed


def main(roboclaw):
    vid = cv2.VideoCapture(0)
    at_detector = Detector(families='tag36h11',
                           nthreads=1,
                           quad_decimate=1.0,
                           quad_sigma=0.0,
                           refine_edges=1,
                           decode_sharpening=0.25,
                           debug=0)
    width = int(vid.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT))

    left_speed = 0
    right_speed = 0
    max_speed = 127
    state = STATE_GO_FARTHEST
    state_history = [state]
    tag_missing_frames = 0
    tag_last_seen = 0
    slowest_turning_speed = 0
    turn_tag_last_seen = 0
    started_turning = 0
    target_tag = None
    last_target_tag_distance = 0

    while (True):

        ret, frame = vid.read()
        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        tag_size_cm = 20.5

        focal_length = (1, 1)
        camera_center = (1, 1)

        tags = at_detector.detect(gray_image, estimate_tag_pose=True,
                                  camera_params=[focal_length[0], focal_length[1], camera_center[0], camera_center[1]],
                                  tag_size=(tag_size_cm / 100))

        now = millis()
        target_tag = None

        # Display tag result info on image
        # From https://pyimagesearch.com/2020/11/02/apriltag-with-python/
        for r in tags:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the AprilTag detection
            cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
            cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
            cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
            cv2.line(frame, ptD, ptA, (0, 255, 0), 2)
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
            # draw the tag family on the image
            tagFamily = r.tag_family.decode("utf-8")
            cv2.putText(frame, str(r.tag_id), (cX, cY),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Find the furthest tag
        farthest_tag = None
        for tag in tags:
            if farthest_tag is None or tag.center[1] < farthest_tag.center[1]:
                farthest_tag = tag

        # Run state machine
        print(chr(27) + "[2J")
        print('State: ' + state)

        if state == STATE_GO_FARTHEST:
            # Adjust power to go to the farthest tag (if one exists)
            if farthest_tag is not None:
                target_tag = farthest_tag
                left_speed, right_speed = get_left_right_power_for_tag(farthest_tag, width, max_speed)
            else:
                left_speed = left_speed / 2
                right_speed = right_speed / 2

            # If a turn tag is seen, target that one instead
            for tag in tags:
                if tag.tag_id == TAG_TURN_LEFT:
                    state = STATE_GO_TURN
                    break
                if tag.tag_id == TAG_TURN_LEFT_DELAY:
                    state = STATE_GO_TURN_DELAY
                    break
        elif go_to_turn_states.count(state) > 0:
            turn_tag = None
            for tag in tags:
                if ((state == STATE_GO_TURN and tag.tag_id == TAG_TURN_LEFT) or
                        (state == STATE_GO_TURN_DELAY and tag.tag_id == TAG_TURN_LEFT_DELAY)):
                    turn_tag = tag
                    break
            delay_time = 6000 if (state == STATE_GO_TURN_DELAY) else 1000
            if now - turn_tag_last_seen > delay_time:
                # Start turning when the turn tag is no longer seen
                forward_adjust_speed = 30
                roboclaw.ForwardM1(0x80, forward_adjust_speed)
                roboclaw.ForwardM2(0x80, forward_adjust_speed)
                time.sleep(1.8)
                roboclaw.ForwardM1(0x80, 0)
                roboclaw.ForwardM2(0x80, 0)
                time.sleep(0.5)
                state = STATE_TURN_LEFT
                started_turning = now
            elif turn_tag is not None:
                target_tag = turn_tag
                left_speed, right_speed = get_left_right_power_for_tag(turn_tag, width, max_speed)
            else:
                left_speed = left_speed / 2
                right_speed = right_speed / 2
        elif state == STATE_TURN_LEFT:
            # Keep turning until the furthest non-turning tag is centered
            turning_speed = 50
            #satisfied = True
            #if roboclaw.ReadSpeedM2(0x80)[1] < 50:
            #    slowest_turning_speed = slowest_turning_speed + 1
            #    time.sleep(0.07)
            #    satisfied = False
            #else:
            #    started_turning = now

            min_turn_time_ms = 4000
            left_speed = -1 * turning_speed
            right_speed = turning_speed

            if started_turning < now - min_turn_time_ms:
                valid_tag = None
                for tag in tags:
                    if turn_tags.count(tag.tag_id) == 0 and (valid_tag is None or tag.center[1] < valid_tag.center[1]):
                        valid_tag = tag
                if valid_tag is not None:
                    # Get within x% of center
                    center_error_percentage = 15
                    distance_from_center = valid_tag.center[0] - (width/2)
                    distance_percentage = rescale(distance_from_center, -1 * (width/2), (width/2), -100, 100)
                    if abs(distance_percentage) < center_error_percentage:
                        left_speed = 0
                        right_speed = 0
                        target_tag = valid_tag
                        state = STATE_GO_FARTHEST

        elif state == STATE_GO_HOME:
            # Adjust power to go to the farthest tag (if one exists)

            if farthest_tag is not None:
                left_speed, right_speed = get_left_right_power_for_tag(farthest_tag, width, max_speed)
                target_tag = valid_tag
            else:
                left_speed = left_speed / 2
                right_speed = right_speed / 2

        if state_history[-1] != state:
            state_history.append(state)

        if len(tags) == 0:
            tag_missing_frames = tag_missing_frames + 1
        else:
            tag_missing_frames = 0
            tag_last_seen = now

        if target_tag is not None:
            distance_total = pow(target_tag.pose_t[0][0],2) + pow(target_tag.pose_t[1][0],2) + pow(target_tag.pose_t[2][0],2)
            distance_sqrt = math.sqrt(distance_total)
            last_target_tag_distance = distance_sqrt

            # Show line to target tag
            cv2.line(frame, (int(width/2), int(height)), (int(target_tag.center[0]), int(target_tag.center[1])), (0, 0, 255), 3)



        for t in tags:
            if turn_tags.count(t.tag_id) > 0:
                turn_tag_last_seen = now
                break

        print(left_speed)
        print(right_speed)
        print('State history: ')
        print(state_history)
        if abs(left_speed) < 0.1:
            left_speed = 0
        if abs(right_speed) < 0.1:
            right_speed = 0

        if left_speed > 0:
            roboclaw.ForwardM1(0x80, min(127, int(left_speed)))
        else:
            roboclaw.BackwardM1(0x80, max(-127, abs(int(left_speed))))

        if right_speed > 0:
            roboclaw.ForwardM2(0x80, min(127, int(right_speed)))
        else:
            roboclaw.BackwardM2(0x80, max(-127, abs(int(right_speed))))

        # Display the resulting frame
        cv2.imshow('Frame', frame)

        # q to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    vid.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    roboclaw = Roboclaw('/dev/ttyACM0', 38400)
    roboclaw.Open()
    try:
        main(roboclaw)
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
