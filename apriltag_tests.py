import cv2
from pupil_apriltags import Detector
import math


vid = cv2.VideoCapture(0)
at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)
width  = int(vid.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT))


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


        cv2.line(frame, (cX, cY), (int(width/2), height), (0,255,0), 2)


    # Display the resulting frame
    cv2.imshow('Frame', frame)

    # q to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()
