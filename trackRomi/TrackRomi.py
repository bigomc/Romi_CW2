# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import rangeFinder
import gridMap
import croppingExample
# construct the argument parse and parse the arguments



ap = argparse.ArgumentParser()
# ap.add_argument("-m", "--map", required=True,
#                 help="path to the map without romi image file")
# ap.add_argument("-i", "--image", required=True,
#                 help="path to the calibration image file")
ap.add_argument("-v", "--video", required = True,
                help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=32768,
                help="max buffer size")
args = vars(ap.parse_args())




pts = deque(maxlen=args["buffer"])

# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
    vs = VideoStream(src=0).start()

# otherwise, grab a reference to the video file
else:
    vs = cv2.VideoCapture(args["video"])

# allow the camera or video file to warm up
time.sleep(2.0)

frame = vs.read()
frame = frame[1] if args.get("video", False) else frame
baseImg = frame.copy()

print("Region of interest for romi detecion: \nPress r to reset \n Press c if you are happy!")
[roi, p1, p2, p3, p4] = croppingExample.get_crop(baseImg)

print("Press 's' if you are happy with the range")
H_min, S_min, V_min, H_max, S_max, V_max = rangeFinder.rangefinder(roi)# H, S, V (min) | H, S, V (max)
# keep going with "s"


# define the lower and upper boundaries for the object to track HSV color space, then initialize the
# list of tracked points

##these values give the romi color
HSV_Lower = (H_min, S_min, V_min)
HSV_Upper = (H_max, S_max, V_max)
####

print("Region of interest to crop the map: \nPress r to reset \n Press c if you are happy!")
[frame, p1, p2, p3, p4] = croppingExample.get_cropping_map(frame)

print("Making the initial map with points")
[baseImg, baseSum] = gridMap.gridMapping(frame, False)
print("The base points that romi is required to complete are {0}".format(baseSum))
visitedImg = baseImg.copy()

# keep looping
while (frame is not None):
    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video

    # resize the frame, blur it, and convert it to the HSV
    # color space
    # frame = imutils.resize(frame, width=600)
    frame = croppingExample.four_point_transform(frame, np.array([p1, p2, p3, p4]))  ## roi
    goalImg = frame

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the marked area, then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, HSV_Lower, HSV_Upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 5 and radius <10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            # cv2.circle(frame, (int(x), int(y)), int(radius),
            #            (0, 255, 255), 2)
            # cv2.circle(frame, center, 5, (0, 0, 255), -1)

            cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
            cv2.circle(visitedImg, center, 5, (0, 0, 255), -1)




    # update the points queue
    pts.appendleft(center)




    # loop over the set of tracked points
    for i in range(1, len(pts)):
        # if either of the tracked points are None, ignore
        # them
        if pts[i - 1] is None or pts[i] is None:
            continue


        thickness = 5
        cv2.line(visitedImg, pts[i - 1], pts[i], (255, 0, 0), thickness)
        cv2.line(frame, pts[i - 1], pts[i], (255, 0, 0), thickness)

    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    # if the 'q' key is pressed, stop the loop
    if (key == ord("q")):
        break
    # grab the current frame
    frame = vs.read()

    # handle the frame from VideoCapture or VideoStream
    frame = frame[1] if args.get("video", False) else frame



# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
    vs.stop()

# otherwise, release the camera
else:
    vs.release()

print("making the grid")

[visitedImg, visitedPoints] = gridMap.subdivide(visitedImg, True)

print("Now we can compare both images")

# cv2.imshow("Base Image", baseImg)
cv2.imshow("Visited Image", visitedImg)

print("Required spaces to visit: ", baseSum, "\nVisited Spaces: ", visitedPoints)

cv2.waitKey(0)
# close all windows
cv2.destroyAllWindows()