import cv2
from pupil_apriltags import Detector


vid = cv2.VideoCapture(0)
at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)


while (True):

    ret, frame = vid.read()
    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    tags = at_detector.detect(gray_image, estimate_tag_pose=False, camera_params=None, tag_size=None)

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
        cv2.putText(frame, tagFamily, (ptA[0], ptA[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the resulting frame
    cv2.imshow('Frame', frame)

    # q to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()
