import cv2
import numpy as np


def subdivide(image):
    gridsize = 100
    outputimage = image
    rows, cols, chann = image.shape
    x = 0
    y = 0
    while(y<rows-gridsize):
        x = 0
        while(x<cols-gridsize):
            k = x*y+x
            cv2.rectangle(outputimage, (x,y), (x+gridsize,y+gridsize), (0, 0, 255), 1)
            x += gridsize
        y += gridsize
    return outputimage


nameOfImg = 'Photos/WellDoneMap2.jpg'

img = cv2.imread(nameOfImg)
img = cv2.resize(img, (810, 1080))
blankImg = np.zeros((1080,810,3), np.uint8)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

edges = cv2.Canny(gray, 75, 150)

minLineLength = 50

lines = cv2.HoughLinesP(edges, 1, np.pi/180, minLineLength, maxLineGap=80)
for line in lines:
    x1, y1, x2, y2 = line[0]
    cv2.line(blankImg, (x1, y1), (x2, y2), (0, 255, 0), 3)


subdivide(blankImg)

cv2.imshow("Edges", img)
cv2.imshow("Subdivided", blankImg)

cv2.waitKey(0)
cv2.destroyAllWindows()


