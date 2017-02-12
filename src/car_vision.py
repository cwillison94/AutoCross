#opens up a webcam feed so you can then test your classifer in real time
#using detectMultiScale
import numpy as np
import cv2
import time
import math
from picamera.array import PiRGBArray
from picamera import PiCamera
import line_helper
import argparse
import track
import detect


p = argparse.ArgumentParser()
p.add_argument("-hl", "--headless", help="Run in headless mode", action="store_true")

args = p.parse_args()
HEADLESS = args.headless

print "HEADLESS = ", HEADLESS

STOP_SIGN_HAAR = "stop_sign_haar.xml"

CONSECUTIVE_BUFFER_SIZE = 5
CONSECUTIVE_THRESHOLD = 0.50
DEFAULT_RESOLUTION_WIDTH = 608
DEFAULT_RESOLUTION_HEIGHT = 608

HOUGH_LINE_RHO = 1
HOUGH_LINE_THRESHOLD = 75
HOUGH_LINE_MIN_LENGTH = 60
HOUGH_LINE_MAX_GAP = 20

CAMERA_ALPHA = 8.0 * math.pi / 180
CAMERA_V_0 = 119.865631204
CAMERA_A_Y = 32.262498472

LANE_ROI =  0.40 * DEFAULT_RESOLUTION_HEIGHT

print "Lane ROI horizontal ", LANE_ROI


STOP_SIGN_HEIGHT = 15 #cm

def drawBoxes(rects, img):

    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (x1, y1), (x2, y2), (127, 255, 0), 2)

def drawText(img, message):
   cv2.putText(img, message, (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 1)

def detectLines(img):
    (thresh, img_bw) = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY)
    img_bw = cv2.GaussianBlur(img_bw, (5, 5), 0)
    #gray = cv2.equalizeHist(gray)

    # smooth and canny edge detection
    canny = cv2.Canny(img_bw, 50, 200)

    #minLineLength = 10, maxLineGap = 10)
    lines = cv2.HoughLinesP(canny, HOUGH_LINE_RHO, np.pi/180, HOUGH_LINE_THRESHOLD, minLineLength = HOUGH_LINE_MIN_LENGTH, maxLineGap = HOUGH_LINE_MAX_GAP)

    if (lines is None or len(lines) == 0):
        return []

    return lines

def detectStopSign(cascade, img):
    rects = cascade.detectMultiScale(img, 1.1, 5, cv2.CASCADE_SCALE_IMAGE , (20, 20))

    if len(rects) == 0:
        return []
    rects[:, 2:] += rects[:, :2]

    return rects

def detectLanes(lines):
    
    filtered_lines = horizontal_filter(lines[:,0], LANE_ROI, DEFAULT_RESOLUTION_HEIGHT)
    

    #lines_left, lines_right = line_helper.split_lines(filtered_lines, DEFAULT_RESOLUTION_WIDTH/2.)

    #print "left ", lines_left
    #print "right ", lines_right

    #inner_left_m, inner_left_b = find_inner_line(lines_left)
    #inner_right_m, inner_right_b = find_inner_line(lines_right)

    #print "inner left ", inner_left_m, inner_left_b
    #print "inner right ", inner_right_m, inner_right_b



    #return [
    #    [DEFAULT_RESOLUTION_WIDTH, int(inner_left_m * DEFAULT_RESOLUTION_WIDTH + inner_left_b), 0, int(inner_left_m * 0 + inner_left_b)],
    #    [DEFAULT_RESOLUTION_WIDTH, int(inner_right_m * DEFAULT_RESOLUTION_WIDTH + inner_right_b), 0, int(inner_right_m * 0 + inner_right_b)]
    #]
    return filtered_lines

def find_inner_line(lines):
    if len(lines) > 0:
        x1, y1, x2, y2 = lines[0]
        current_right_most_line_m, current_right_most_line_b = line_helper.line([x1, y1], [x2, y2])
        #consider changing to check point at x = 0 instead
        for i in range(len(lines)):
            x1, y1, x2, y2 = lines[i]
            m, b = line_helper.line([x1, y1], [x2, y2])
            if b <= current_right_most_line_b:
                #better choice
                current_right_most_line_m = m
                current_right_most_line_b = b
        return current_right_most_line_m, current_right_most_line_b
    else:
        return 0, 0


def horizontal_filter(lines, minValue, maxValue):
    linesFiltered = []
    for i in range(len(lines)):
        x1, y1, x2, y2 = lines[i]
        if between(y1, minValue, maxValue) and between(y2, minValue, maxValue):
            linesFiltered += [[x1, y1, x2, y2]]

    return linesFiltered


def between(value, minValue, maxValue):
    return value > minValue and value < maxValue

def drawLanes(lines, img):
    for i in range(len(lines)):
        x1, y1, x2, y2 = lines[i]
        cv2.line(img,(x1, y1),(x2, y2), (0, 255, 0), 2) #rem out if u want to used  polykines

def determineStopSignal(stop_sign_buffer, rects):
    stop_sign_buffer += [len(rects)]

    if (len(stop_sign_buffer) >= CONSECUTIVE_BUFFER_SIZE):
        stop_sign_buffer.pop(0)

        #check percentage
        threshold = float(sum(stop_sign_buffer))/float(len(stop_sign_buffer))
        if (threshold >= CONSECUTIVE_THRESHOLD):
            if len(rects) >0:
                x1, y1, x2, y2 = rects[0]
                # y position of target point P
                v = (y2 + y1) / 2 -15
                dist = STOP_SIGN_HEIGHT / math.tan(CAMERA_ALPHA + math.atan((v - CAMERA_V_0) / CAMERA_A_Y))
                return True, dist
            else:
                return True, 0
        else:
            return False, -1
    else:
        return False, -1

def startVision():
    try:

        camera = PiCamera()
        camera.resolution = (DEFAULT_RESOLUTION_WIDTH, DEFAULT_RESOLUTION_HEIGHT)
        camera.framerate = 32
        rawCapture = PiRGBArray(camera, size=(DEFAULT_RESOLUTION_WIDTH, DEFAULT_RESOLUTION_HEIGHT))

        #let camera start up
        time.sleep(0.5)

        stopSignCascade = cv2.CascadeClassifier(STOP_SIGN_HAAR)


        stop_sign_buffer_count = [0]

        print "Starting... press q or ctrl+C to quit"

        ticks = 0

        lt = track.LaneTracker(2, 0.1, 608)
        ld = detect.LaneDetector(200)


        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

            img = frame.array
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            rawCapture.truncate(0)

            rects = detectStopSign(stopSignCascade, img)

            stopSignDetected, stopSignDistance = determineStopSignal(stop_sign_buffer_count, rects)

            drawBoxes(rects, img)

            lines = detectLines(img_gray)

            precTick = ticks
            ticks = cv2.getTickCount()
            dt = (ticks - precTick) / cv2.getTickFrequency()


            predicted = lt.predict(dt)

            lanes = ld.detect(img)

            if predicted is not None:
                cv2.line(img, (predicted[0][0], predicted[0][1]), (predicted[0][2], predicted[0][3]), (0, 0, 255), 5)
                cv2.line(img, (predicted[1][0], predicted[1][1]), (predicted[1][2], predicted[1][3]), (0, 0, 255), 5)

            lt.update(lanes)

            #print "lines ",lines

            if stopSignDetected:
                print "STOP SIGN DETECTED"
                if not(HEADLESS):
                    drawText(img, "Status: STOP SIGN DETECTED dist=%.1fcm" % stopSignDistance)

            #TODO: process lines to detect lanes via length, and location
##            if len(lines) != 0:
##                lanes = detectLanes(lines)
##                if not(HEADLESS):
##                    drawLanes(lanes, img)

            if not(HEADLESS):
                cv2.imshow("auto-live", img)

                cv2.waitKey(10)
            else:
                cv2.waitKey(0)

    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        #I am really not sure why this works... I need to commit
        for i in range(4):
            cv2.waitKey(1)


if __name__ == "__main__": startVision()
