# import the necessary packages
import argparse
import cv2
import numpy as np
# initialize the list of reference points and boolean indicating
# whether cropping is being performed or not
refPtTuple = []
cropping = False
counter = 0
refPt = []


def order_points(pts):
  rect = np.zeros((4, 2), dtype="float32")


  s = pts.sum(axis=1)
  rect[0] = pts[np.argmin(s)]
  rect[2] = pts[np.argmax(s)]

  diff = np.diff(pts, axis=1)
  rect[1] = pts[np.argmin(diff)]
  rect[3] = pts[np.argmax(diff)]

  # return the ordered coordinates
  return rect


def four_point_transform(image, pts):

    ## Taken from  Adrian Rosebrock
  rect = order_points(pts)
  (tl, tr, br, bl) = rect


  widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
  widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
  maxWidth = max(int(widthA), int(widthB))


  heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
  heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
  maxHeight = max(int(heightA), int(heightB))


  dst = np.array([
    [0, 0],
    [maxWidth - 1, 0],
    [maxWidth - 1, maxHeight - 1],
    [0, maxHeight - 1]], dtype="float32")

  # compute the perspective transform matrix and then apply it
  M = cv2.getPerspectiveTransform(rect, dst)
  warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
  return warped
# return the warped




def click_and_crop(event, x, y, flag, params):
    # grab references to the global variables
    global refPtTuple, cropping

    # if the left mouse button was clicked, record the starting
    # (x, y) coordinates and indicate that cropping is being
    # performed
    if event == cv2.EVENT_LBUTTONDOWN:
        refPtTuple = [(x, y)]
        cropping = True

    # check to see if the left mouse button was released
    elif event == cv2.EVENT_LBUTTONUP:
        # record the ending (x, y) coordinates and indicate that
        # the cropping operation is finished
        refPtTuple.append((x, y))
        cropping = False

        # draw a rectangle around the region of interest
        cv2.rectangle(image, refPtTuple[0], refPtTuple[1], (0, 255, 0), 2)
        cv2.imshow("image", image)


def get_points(event, x, y, flag, params):
    # grab references to the global variables
    global refPt, cropping

    # if the left mouse button was clicked, record the starting
    # (x, y) coordinates and indicate that cropping is being
    # performed
    if event == cv2.EVENT_LBUTTONDOWN:
        cropping = True

    # check to see if the left mouse button was released
    elif event == cv2.EVENT_LBUTTONUP:
        # record the ending (x, y) coordinates and indicate that
        # the cropping operation is finished
        cropping = False
        global counter


        refPt.append([])
        refPt[counter].append(x)
        refPt[counter].append(y)
        counter += 1
        print("X: ", x, ", Y: ", y)
        # # draw a rectangle around the region of interest
        # cv2.rectangle(image, refPt[0], refPt[1], (0, 255, 0), 2)
        cv2.imshow("image", image)


def get_cropping_map(imageName):
    # construct the argument parser and parse the arguments
    # load the image, clone it, and setup the mouse callback function
    global image
    image = imageName
    clone = image.copy()
    cv2.namedWindow("image")
    global counter
    flag = False
    cv2.setMouseCallback("image", get_points)



    # keep looping until the 'q' key is pressed
    while True:
        # display the image and wait for a keypress
        cv2.imshow("image", image)
        key = cv2.waitKey(1) & 0xFF

        # if the 'r' key is pressed, reset the cropping region
        if key == ord("r"):
            image = clone.copy()
            global counter, refPt
            counter = 0
            refPt = []
            print("Please choose points again")

        # if the 'c' key is pressed, break from the loop
        elif key == ord("c"):
            break

        if(counter == 4):
            p1 = [refPt[0][0], refPt[0][1]]
            p2 = [refPt[1][0], refPt[1][1]]
            p3 = [refPt[2][0], refPt[2][1]]
            p4 = [refPt[3][0], refPt[3][1]]
            pts = np.array([p1, p2, p3, p4])
            # print(pts)
    # apply the four point tranform to obtain a "birds eye view" of
    # the image
    roi = four_point_transform(image, pts)
    cv2.imshow("ROI", roi)

    return  roi, p1, p2, p3, p4


def get_crop(imageName):
    # construct the argument parser and parse the arguments
    # load the image, clone it, and setup the mouse callback function
    global image
    image = imageName
    clone = image.copy()
    cv2.namedWindow("image")
    cv2.setMouseCallback("image", click_and_crop)

    # keep looping until the 'q' key is pressed
    while True:
        # display the image and wait for a keypress
        cv2.imshow("image", image)
        key = cv2.waitKey(1) & 0xFF

        # if the 'r' key is pressed, reset the cropping region
        if key == ord("r"):
            image = clone.copy()

        # if the 'c' key is pressed, break from the loop
        elif key == ord("c"):
            break

    # if there are two reference points, then crop the region of interest
    # from teh image and display it
    if len(refPtTuple) == 2:
        p1 = refPtTuple[0][1]
        p2 = refPtTuple[1][1]
        p3 = refPtTuple[0][0]
        p4 = refPtTuple[1][0]
        roi = clone[p1:p2, p3:p4]
        cv2.imshow("ROI", roi)
        cv2.waitKey(0)
    # close all open windows
    cv2.destroyAllWindows()
    return roi, p1, p2, p3, p4


def map_cropped(frame, p1, p2, p3, p4):
    return frame[p1:p2, p3:p4]
