import cv2
import numpy as np


def subdivide(image, inRomi):

    outputimage = image
    rows, cols, chann = image.shape
    gridsize_y = round(rows/25)
    gridsize_x = round(cols/25)
    x = 0
    y = 0
    weight = 0
    while(y<rows-gridsize_y):
        x = 0
        while(x<cols-gridsize_x):
            k = x*y+x
            cv2.rectangle(outputimage, (x,y), (x+gridsize_x,y+gridsize_y), (0, 0, 255), 1)
            weight += weightImage(outputimage, x , y, gridsize_x, gridsize_y, inRomi)
            x += gridsize_x
        y += gridsize_y
    return [outputimage, weight]

def weightImage(image, x, y, gridsize_x, gridsize_y, inRomi):
    auximage = image[x:x + gridsize_x, y:y + gridsize_y]
    # grab the image dimensions
    h = auximage.shape[0]
    w = auximage.shape[1]
    weigthsum = 0

    # loop over the image, pixel by pixel
    if inRomi == False:
        for y in range(0, h):
            for x in range(0, w):
                # look at the values of each pixel
                px = auximage[y,x]
                bluepx = px[0]
                greenpx = px[1]
                redpx = px[2]
                if(bluepx <= 250 and greenpx <=250 and redpx<=250):
                    weigthsum += 1
                else:
                    weigthsum += 0
    else:
        for y in range(0, h):
            for x in range(0, w):
                # look at the values of each pixel
                px = auximage[y,x]
                bluepx = px[0]
                greenpx = px[1]
                redpx = px[2]
                if(bluepx <= 250 and greenpx <=250 and redpx<=250):
                    weigthsum += 0
                else:
                    if(bluepx>250):
                        weigthsum += 1
                    else:
                        weigthsum += 0

    if weigthsum > 0:
        return 1
    else:
        return 0


# nameOfImg = 'Photos/WellDoneMap2.jpg'
def gridMapping(nameOfImg, inRomi):

    # img = cv2.resize(nameOfImg, (810, 1080))
    h, w, ch = nameOfImg.shape
    blankImg = np.zeros((h,w,ch), np.uint8)
    gray = cv2.cvtColor(nameOfImg, cv2.COLOR_BGR2GRAY)

    edges = cv2.Canny(gray, 75, 150)

    minLineLength = 50

    lines = cv2.HoughLinesP(edges, 1, np.pi/180, minLineLength, maxLineGap=80)
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(blankImg, (x1, y1), (x2, y2), (0, 255, 0), 3)


    [dividedImage, weightSum] = subdivide(blankImg, inRomi)

    # cv2.imshow("Edges", img)
    cv2.imshow("Subdivided", dividedImage)
    # print(weightSum)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return dividedImage, weightSum


