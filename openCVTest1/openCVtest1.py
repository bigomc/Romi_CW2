import cv2
import numpy as np


def subdivide(image):
    gridsize = 100
    outputimage = image
    rows, cols, chann = image.shape
    x = 0
    y = 0
    weight = 0
    while(y<rows-gridsize):
        x = 0
        while(x<cols-gridsize):
            k = x*y+x
            cv2.rectangle(outputimage, (x,y), (x+gridsize,y+gridsize), (0, 0, 255), 1)
            weight += weightImage(outputimage, x , y, gridsize)
            x += gridsize
        y += gridsize
    return [outputimage, weight]

def weightImage(image, x, y, gridsize):
    auximage = image[x:x + gridsize, y:y + gridsize]
    # grab the image dimensions
    h = auximage.shape[0]
    w = auximage.shape[1]
    weigthsum = 0

    # loop over the image, pixel by pixel
    for y in range(0, h):
        for x in range(0, w):
            # look at the values of each pixel
            px = auximage[y,x]
            bluepx = px[0]
            greenpx = px[1]
            redpx = px[2]
            if(bluepx != 255 and greenpx != 255):
                weigthsum += 0
            else:
                weigthsum += 1
    if weigthsum > 0:
        return 0
    else:
        return 1


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


[dividedImage, weightSum] = subdivide(blankImg)

cv2.imshow("Edges", img)
cv2.imshow("Subdivided", dividedImage)
print(weightSum)

cv2.waitKey(0)
cv2.destroyAllWindows()


