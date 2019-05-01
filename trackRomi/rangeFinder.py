#!/usr/bin/env python
# -*- coding: utf-8 -*-

# USAGE: You need to specify a filter and "only one" image source
#
# (python) range-detector --filter RGB --image /path/to/image.png
# or
# (python) range-detector --filter HSV --webcam

import cv2
import argparse
from operator import xor

def callback(value):
    pass


def setup_trackbars(range_filter):
    cv2.namedWindow("Trackbars", 0)

    for i in ["MIN", "MAX"]:
        v = 0 if i == "MIN" else 255

        for j in range_filter:
            cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, 255, callback)


def get_arguments(imagename):
    ap = argparse.ArgumentParser()
    ap.add_argument('-f', '--filter', default='HSV',
                    help='Range filter. RGB or HSV')
    ap.add_argument('-i', '--image', default = imagename,
                    help='Path to the image')
    ap.add_argument('-w', '--webcam', required=False,
                    help='Use webcam', action='store_true')
    ap.add_argument("-v", "--video",
                    help="path to the (optional) video file")
    args1 = vars(ap.parse_args())


    return args1


def get_trackbar_values(range_filter):
    values = []

    for i in ["MIN", "MAX"]:
        for j in range_filter:
            v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
            values.append(v)

    return values


def rangefinder(imagename):

    range_filter = 'HSV'

    image = imagename

    frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    setup_trackbars(range_filter)

    while True:
        v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = get_trackbar_values(range_filter)

        thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))


        preview = cv2.bitwise_and(image, image, mask=thresh)
        cv2.imshow("Preview", preview)

        # cv2.imshow("Original", image)



        if cv2.waitKey(1) & 0xFF is ord('s'):
            cv2.destroyAllWindows()
            return v1_min, v2_min, v3_min, v1_max, v2_max, v3_max
        # H, S, V (min) | H, S, V (max)