import sys
sys.path.append('/usr/local/lib/python2.7/site-packages')
import numpy as np
import cv2
import time
import math
from picamera.array import PiRGBArray
from picamera import PiCamera
import line_helper
import argparse
import detect
import track
import steering
import car_motor
import sonar_range
import timeit

p = argparse.ArgumentParser()
p.add_argument("-hl", "--headless", help="Run in headless mode", action="store_true")

args = p.parse_args()
HEADLESS = True #args.headless
DEBUG_MODE = True

print "HEADLESS = ", HEADLESS

STOP_SIGN_HAAR = "stop_sign_haar.xml"

CONSECUTIVE_STOP_SIGN_BUFFER_SIZE = 5
CONSECUTIVE_STOP_SIGN_THRESHOLD = 0.50


DEFAULT_RESOLUTION_WIDTH = 608
DEFAULT_RESOLUTION_HEIGHT = 608

HOUGH_LINE_RHO = 1
HOUGH_LINE_THRESHOLD = 30
HOUGH_LINE_MIN_LENGTH = 2
HOUGH_LINE_MAX_GAP = 30

CAMERA_ALPHA = 8.0 * math.pi / 180
CAMERA_V_0 = 119.865631204
CAMERA_A_Y = 32.262498472

#WORKING SLOW SPEED VALUES
# DON'T LOOSE ME
# K_p = 0.7
# K_i = 0
# K_d = 0.15
CAMERA_X_OFFSET = 0

#CONTROLLER_K_P = 0.60
#CONTROLLER_K_I = 0
#CONTROLLER_K_D = 0.25

CONTROLLER_K_P = 0.62 #0.63
CONTROLLER_K_I = 0
CONTROLLER_K_D = 0.19
CONTROLLER_ANGLE_SCALE = 10


STOP_SIGN_HEIGHT = 15 #cm

car_steering = steering.Steering()
car_motor = car_motor.CarMotor()
ranger = sonar_range.ranger()

def drawBoxes(rects, img):

    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (x1, y1), (x2, y2), (127, 255, 0), 2)

def drawText(img, message, location = (50, 50)):
   cv2.putText(img, message, location, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 1)

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
    img_roi = img[0:img.shape[1], img.shape[0]/2:img.shape[0]]
    rects = cascade.detectMultiScale(cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY), 1.1, 5, cv2.CASCADE_SCALE_IMAGE , (25, 25))


    if len(rects) == 0:
        return []

    rects[:, 2:] += rects[:, :2]

    # shift back to original image
    for i in range(len(rects)):
        rects[i][0] += img.shape[0]/2
        rects[i][2] += img.shape[0]/2

        #rects[i][1] += img.shape[1]/2
        #rects[i][3] += img.shape[1]/2

    return rects

def detectLanes(lines):
    return lines[:,0]

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
    #return filtered_lines

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

    if (len(stop_sign_buffer) >= CONSECUTIVE_STOP_SIGN_BUFFER_SIZE):
        stop_sign_buffer.pop(0)

        #check percentage
        threshold = float(sum(stop_sign_buffer))/float(len(stop_sign_buffer))
        if (threshold >= CONSECUTIVE_STOP_SIGN_THRESHOLD):
            if len(rects) > 0:
                x1, y1, x2, y2 = rects[0]
                # y position of target point P
                #v = (y2 + y1) / 2 -15
                #dist = STOP_SIGN_HEIGHT / math.tan(CAMERA_ALPHA + math.atan((v - CAMERA_V_0) / CAMERA_A_Y))
                dist = 0
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
        camera.framerate = 40
        rawCapture = PiRGBArray(camera, size=(DEFAULT_RESOLUTION_WIDTH, DEFAULT_RESOLUTION_HEIGHT))

        #let camera start up
        time.sleep(0.5)

        stopSignCascade = cv2.CascadeClassifier(STOP_SIGN_HAAR)


        stop_sign_buffer_count = [0]

        print "Starting... press q or ctrl+C to quit"

        ticks = 0
        ld = detect.LaneDetector(300, DEFAULT_RESOLUTION_WIDTH, DEFAULT_RESOLUTION_HEIGHT, debug_mode = not(HEADLESS))

        car_mid = DEFAULT_RESOLUTION_WIDTH/2 + CAMERA_X_OFFSET

        prev_car_error = 0
        integral = 0


        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            loop_start = timeit.default_timer()
            img = frame.array
            rawCapture.truncate(0)

            precTick = ticks
            ticks = cv2.getTickCount()
            dt = (ticks - precTick) / cv2.getTickFrequency()

            #print "Delta time = ", dt

            range_dist = ranger.read_cm()
            #print "Range =", range_dist

            rects = detectStopSign(stopSignCascade, img)
            stopSignDetected, stopSignDistance = determineStopSignal(stop_sign_buffer_count, rects)
            drawBoxes(rects, img)

            if stopSignDetected:
                print "Stop Sign Detected"
                car_motor.stop()
            else:


                lanes = ld.detect(img)

                try:
                    left_lane = lanes[0]
                except:
                    left_lane = None

                try:
                    right_lane = lanes[1]
                except:
                    left_lane = None

                if range_dist < 50:
                    car_motor.stop()
                    #print "Object in range"

                elif left_lane != None and right_lane!= None:
                    if car_motor.moving != True:
                        #pass
                        car_motor.set_percent_power(13)

                    base_left = left_lane[4]
                    base_right = right_lane[4]
                    theta_left = left_lane[5]
                    theta_right = right_lane[5]

                    car_error = (base_right + base_left)/2
                    integral = integral + car_error * dt
                    derivative = (car_error - prev_car_error)/dt

                    output = CONTROLLER_K_P * car_error + CONTROLLER_K_I * integral + CONTROLLER_K_D * derivative
                    prev_car_error = car_error

                    car_steering.set_percent_direction(output)

                    #print "left=" + str(base_left) + " right=" + str(base_right)
                    #print "Theta Left=" + str(theta_left) + " theta right=" + str(theta_right)
                    #print "average angle: " + str(avg_angle)
                    #print "Percent direction: ", output

                    if not(HEADLESS):
                        drawText(img, "steering %.1f " % output, (50, 300))
                        cv2.line(img, (left_lane[0], left_lane[1]), (left_lane[2], left_lane[3]), (0, 0, 255), 5)
                        cv2.line(img, (right_lane[0], right_lane[1]), (right_lane[2], right_lane[3]), (0, 0, 255), 5)
                else:
                    print "No lanes detected"
                    car_motor.stop()
                    #stop car

            if not(HEADLESS):
                cv2.line(img, (car_mid, 0), (car_mid, img.shape[1]),(0, 255,0), 5)

            #if stopSignDetected:
                #print "STOP SIGN DETECTED"
                #if not(HEADLESS):
                    #drawText(img, "Status: STOP SIGN DETECTED dist=%.1fcm" % stopSignDistance)

            #if DEBUG_MODE:
                ##TODO: process lines to detect lanes via length, and location
                #lines = detectLines(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
                #print "lines",lines
                #if len(lines) != 0:
                    #lanes = detectLanes(lines)
                    #if not(HEADLESS):
                        #drawLanes(lanes, img)

            if not(HEADLESS):
                cv2.imshow("auto-live", img)

                cv2.waitKey(5)

            #print "Loop time =\t", timeit.default_timer() - loop_start

    except KeyboardInterrupt:

        car_motor.stop()
        cv2.destroyAllWindows()
        #I am really not sure why this works... I need to commit
        for i in range(4):
            cv2.waitKey(1)
        raise
    except:
        car_motor.stop()
        raise


if __name__ == "__main__": startVision()
