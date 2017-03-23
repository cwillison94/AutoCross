import sys
sys.path.append('/usr/local/lib/python2.7/site-packages')
import numpy as np
import cv2
import time
import math
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import lane_detector
import steering
import car_motor
import sonar_range
import timeit
import stop_sign_detector
#Abhi
from threading import Thread

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

#WITH BASE DISTANCE MODIFIER
CONTROLLER_K_P = 0.65 #0.63
CONTROLLER_K_I = 0
CONTROLLER_K_D = 0.19


#NO BASE DISTANCE MODIFIER
#CONTROLLER_K_P = 0.65 #0.63
#CONTROLLER_K_I = 0
#CONTROLLER_K_D = 0.2

CONTROLLER_ANGLE_SCALE = 10

CAR_POWER = 0


STOP_SIGN_HEIGHT = 15 #cm

car_steering = steering.Steering()
car_motor = car_motor.CarMotor()
ranger = sonar_range.Ranger()
stop_detector = stop_sign_detector.StopSignDetector()

sending_stop_signal = False
stopping_car = False

def draw_boxes(rects, img):
    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (x1, y1), (x2, y2), (127, 255, 0), 2)

def draw_text(img, message, location = (50, 50)):
    cv2.putText(img, message, location, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 1)


def draw_lines(lines, img):
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

#Abhi - will get back to you later
# def stopSignThreadOffload():
#     while true:
#         threading.Thread()

def start_vision():
    try:

        camera = PiCamera()
        camera.resolution = (DEFAULT_RESOLUTION_WIDTH, DEFAULT_RESOLUTION_HEIGHT)
        camera.framerate = 40
        rawCapture = PiRGBArray(camera, size=(DEFAULT_RESOLUTION_WIDTH, DEFAULT_RESOLUTION_HEIGHT))

        #let camera start up
        time.sleep(0.5)

        stop_sign_buffer_count = [0]

        print "Starting... press q or ctrl+C to quit"

        ticks = 0
        ld = lane_detector.LaneDetector(300, DEFAULT_RESOLUTION_WIDTH, DEFAULT_RESOLUTION_HEIGHT, debug_mode = not(HEADLESS))

        car_mid = DEFAULT_RESOLUTION_WIDTH/2 + CAMERA_X_OFFSET

        prev_car_error = 0
        integral = 0

        stop_detector.start()

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

            #rects = detectStopSign(stopSignCascade, img)
            #stopSignDetected, stopSignDistance = determineStopSignal(stop_sign_buffer_count, rects)
            #drawBoxes(rects, img)

            stop_detector.push_img(img)

            stop_detected, normalized_dist = stop_detector.get_stop_info()

            #TODO: Move logic into a car class



            if stop_detected and normalized_dist < 70:
                drawText(img, "Stopping car")
                car_motor.stop()
            else:
                # find lanes
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
                    print "Object in range"

                elif left_lane != None and right_lane!= None:
                    if car_motor.moving != True:
                        #pass
                        car_motor.set_percent_power(CAR_POWER)

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
                        self.drawText(img, "steering %.1f " % output, (50, 300))
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
                        #drawLines(lanes, img)

            if not(HEADLESS):
                cv2.imshow("auto-live", img)

                cv2.waitKey(5)

            #print "Loop time =\t", timeit.default_timer() - loop_start

    except KeyboardInterrupt:

        car_motor.stop()
        stop_detector.stop()
        ranger.cancel()
        cv2.destroyAllWindows()
        #I am really not sure why this works... I need to commit
        for i in range(4):
            cv2.waitKey(1)
        raise
    except:
        stop_detector.stop()
        car_motor.stop()
        raise


if __name__ == "__main__": start_vision()