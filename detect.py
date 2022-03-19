import cv2
import numpy as np
import math

def getAngle(x1, y1, x2, y2):
    if x1 == x2:
        return np.pi / 2
    slope = (y2 - y1) / (x2 - x1)
    if slope >= 0:
        return math.atan(slope)
    else:
        return np.pi - math.atan(abs(slope))

def getSlopeOfLine(line):
    xDis = line[0][2] - line[0][0]
    if (xDis == 0):
        return None

    return (line[0][3] - line[0][1]) / xDis

def extractLines(img):
    # Find the black lines in the image
    low_black = np.array([0, 0, 0])
    up_black = np.array([48, 45, 45])
    mask = cv2.inRange(img, low_black, up_black)

    edges = cv2.Canny(mask, 75, 150)

    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 550, maxLineGap=60)
    return lines

def getAngleAndSlopeOfLine(line):
    slope = getSlopeOfLine(line)
    if slope is None:
        return np.pi / 2, None
    elif slope >= 0:
        return math.atan(slope), slope
    else:
        return np.pi - math.atan(abs(slope)), slope

def getParallelLines(lines):
    angle, slope = getAngleAndSlopeOfLine(lines[0])

    t = tuple([lines[0], slope])
    length = len(lines)

    parallelLines = [t]

    for i in range(1, length):
        angleA, slopeA = getAngleAndSlopeOfLine(lines[i])
        if abs(angleA - angle) <= 5:
            t = tuple([lines[i], slopeA])
            parallelLines.append(t)
    return parallelLines


def splitLinesIntoGroups(parallelLines):
    # Split l into subsets
    set1 = []
    set2 = []
    line0 = parallelLines[0]
    x1, y1, x2, y2 = line0[0][0]
    v = y1 + line0[1] * (x2- x1)

    set1.append(line0[0])
    for index in range(1, len(parallelLines)):
        line1 = parallelLines[index]
        if abs(line0[1] - line1[1]) > 0.5:
            set2.append(line1[0])
        else:
            x21, y21, x22, y22 = line1[0][0]
            v = y1 + line0[1] * (x21- x1)
            if abs(v - y21) <= 150:
                set1.append(line1[0])
            else:
                set2.append(line1[0])
    return set1, set2

def findMinMaxPoints(lines):
    xmin1, ymin1, xmax1, ymax1 = lines[0][0]

    for index in range(1, len(lines)):
        x1, y1, x2, y2 = lines[index][0]
        if (xmin1 > x1):
            xmin1 = x1
            ymin1 = y1
        if (xmax1 < x2):
            xmax1 = x2
            ymax1 = y2
    if ymin1 < ymax1:
        return xmin1, ymin1, xmax1, ymax1
    else:
        return xmax1, ymax1, xmin1, ymin1



def drawArrowedLine(startx, starty, endx, endy, img):
    if starty < endy:
        cv2.arrowedLine(img, (endx, endy), (startx, starty), (0, 255, 0), thickness = 50)
    else:
        cv2.arrowedLine(img, (startx, starty), (endx, endy), (0, 255, 0), thickness = 50)

imageName="IMG-7532.jpeg"
resultImage="IMG-7532-result.jpeg"
img = cv2.imread(imageName)

lines = extractLines(img)

parallelLines = getParallelLines(lines)

set1, set2 = splitLinesIntoGroups(parallelLines)

x11,y11,x12,y12 = findMinMaxPoints(set1)
angle1 = getAngle(x11,y11,x12,y12)

x21,y21,x22,y22 = findMinMaxPoints(set2)
angle2 = getAngle(x21,y21,x22,y22)


angleAvg = (angle1 + angle2)/2

slopeAvg = math.tan(angleAvg)

# mid points of the 4 points
midx = int((x11 + x12 + x21 + x22) / 4)
midy = int((y11 + y12 + y21 + y22) / 4)

slopeAvg = math.tan(angleAvg)

length = 500

# Treat it as vertical line
if slopeAvg < -80 or slopeAvg > 80:
    endx = midx
    endy = midy + length
else:
    endx = midx + length
    endy = int(midy + slopeAvg * length)
#print(midx, midy, endx, endy)

cv2.line(img, (x11,y11), (x12,y12), (0, 255, 255), thickness = 30)
cv2.line(img, (x21,y21), (x22,y22), (0, 255, 255), thickness = 30)
drawArrowedLine(midx, midy, endx, endy, img)

cv2.imshow("Updated Image", img)

cv2.imwrite(resultImage,img)

cv2.waitKey(0)
