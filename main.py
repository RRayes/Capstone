from roboclaw.roboclaw_3 import Roboclaw
import time
import os
import sys
import cv2
from pupil_apriltags import Detector
import math


def rescale(val, in_min, in_max, out_min, out_max):
    return out_min + (val - in_min) * ((out_max - out_min) / (in_max - in_min))


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

    forward_speed = 0
    turning_speed = 0
    max_speed = 127

    while (True):

        ret, frame = vid.read()
        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        tag_size_cm = 20.5

        focal_length = (1, 1)
        camera_center = (1, 1)

        tags = at_detector.detect(gray_image, estimate_tag_pose=True,
                                  camera_params=[focal_length[0], focal_length[1], camera_center[0], camera_center[1]],
                                  tag_size=(tag_size_cm / 100))

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

            print(chr(27) + "[2J")
            print(str(r))
            distance_total = pow(r.pose_t[0][0],2) + pow(r.pose_t[1][0],2) + pow(r.pose_t[2][0],2)
            distance_sqrt = math.sqrt(distance_total)
            print(distance_sqrt)
            forward_speed = int(rescale(distance_sqrt, 0, 3, 0, max_speed))
            turning_speed = int(rescale(int(width/2) - cX, -1 * (width/2), (width/2), -1 * (max_speed / 1), (max_speed / 1)))

            cv2.line(frame, (cX, cY), (int(width/2), height), (0,255,0), 2)

            print('Forward speed ' + str(forward_speed))
            print('Turning speed ' + str(turning_speed))

        if len(tags) == 0:
            forward_speed = forward_speed / 2
            turning_speed = turning_speed / 2

        # Negative means left
        left_speed = int(forward_speed - turning_speed)
        right_speed = int(forward_speed + turning_speed)

        if left_speed > 0:
            roboclaw.ForwardM1(0x80, left_speed)
        else:
            roboclaw.BackwardM1(0x80, abs(left_speed))

        if right_speed > 0:
            roboclaw.ForwardM2(0x80, right_speed)
        else:
            roboclaw.BackwardM2(0x80, abs(right_speed))

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
        print('Interrupted')
        roboclaw.ForwardM1(0x80, 0)
        roboclaw.ForwardM2(0x80, 0)
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)
