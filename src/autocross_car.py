import sys
sys.path.append('/usr/local/lib/python2.7/site-packages')
import timeit
import logging
import time
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera

# import autocross modules
from vision.stop_sign_detector import StopSignDetector
from vision.lane_detector import LaneDetector
from car_control.car_motor import CarMotor
from car_control.car_steering import CarSteering
from distance.sonar_range import Ranger, FRONT_LEFT_SONAR_PINS, FRONT_RIGHT_SONAR_PINS

DEFAULT_CAMERA_PARAMS = (608, 608, 32)

CONTROLLER_K_P = 0.65 #0.63
CONTROLLER_K_I = 0
CONTROLLER_K_D = 0.19

class AutoCrossCar:
    def __init__(self, camera_params = DEFAULT_CAMERA_PARAMS, with_display = False):
        self.camera_initiliazed = False
        self.resolution = (camera_params[0], camera_params[1])
        self.framerate = camera_params[2]
        self.car_midline = self.resolution[0]/2

        self.with_display = with_display

        # setup steering PID variables
        self.ticks = 0
        self.prev_ticks = 0
        self.dt = 0
        self.prev_car_error = 0
        self.car_error = 0
        self.integral = 0
        self.derivative = 0

        self.car_power = 14

    def _initialize_camera(self):
        if self.camera_initiliazed:
            logging.info("Camera has already been initialized.")
        else:
            logging.info("Initiliazing camera")
            self.camera = PiCamera()
            self.camera.resolution = self.resolution
            self.camera.framerate = self.framerate
            self.capture = PiRGBArray(self.camera, self.camera.resolution)

            # wait for camera module to start up
            time.sleep(0.5)

    def _steering_PID(self, left_lane, right_lane):
        self.prev_ticks = self.ticks
        self.ticks = cv2.getTickCount()
        self.dt = (self.ticks - self.prev_ticks) / cv2.getTickFrequency()

        base_left = left_lane[4]
        base_right = right_lane[4]

        self.car_error = (base_right + base_left)/2
        self.integral = self.integral + self.car_error * self.dt
        self.derivative = (self.car_error - self.prev_car_error)/self.dt

        output = CONTROLLER_K_P * self.car_error + CONTROLLER_K_I * self.integral + CONTROLLER_K_D * self.derivative

        self.prev_car_error = self.car_error

        return output

    def _draw_text(self, img, message, location = (50, 50)):
        cv2.putText(img, message, location, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 1)

    def _draw_lanes(self, img, left_lane, right_lane, steering_output):
        if self.with_display:
            self._draw_text(img, "steering %.1f " % steering_output, (50, 300))
            cv2.line(img, (left_lane[0], left_lane[1]), (left_lane[2], left_lane[3]), (0, 0, 255), 5)
            cv2.line(img, (right_lane[0], right_lane[1]), (right_lane[2], right_lane[3]), (0, 0, 255), 5)

    def _draw_midline(self, img):
        if self.with_display:
            cv2.line(img, (self.car_midline, 0), (self.car_midline, img.shape[1]),(0, 255,0), 5)

    def start_auto(self):
        logging.info("Starting AutoCross car in automatic mode")
        self._initialize_camera()

        lane_detector = LaneDetector(300, self.resolution[0], self.resolution[1], debug_mode = self.with_display)
        stop_detector = StopSignDetector()

        steering = CarSteering()
        motor = CarMotor()

        #fl_ranger = Ranger(FRONT_LEFT_SONAR_PINS)
        #fr_ranger = Ranger(FRONT_RIGHT_SONAR_PINS)

        # start stop sign detector
        stop_detector.start()

        logging.info("Starting car in automatic mode")
        try:
            for frame in self.camera.capture_continuous(self.capture, format="bgr", use_video_port=True):
                img = frame.array
                self.capture.truncate(0)

                stop_detector.push_img(img)
                lanes = lane_detector.detect(img)
                left_lane = lanes[0]
                right_lane = lanes[1]

                #stop_detected, normalized_dist = stop_detector.get_stop_info()

                #if fl_ranger.read_cm() < 60 or fr_ranger.read_cm() < 60:
                #    motor.stop()
                if left_lane is not None and right_lane is not None:
                    motor.set_percent_power(self.car_power)
                    steering_output = self._steering_PID(left_lane, right_lane)
                    steering.set_percent_direction(steering_output)
                    self._draw_lanes(img, left_lane, right_lane, steering_output)


                if self.with_display:
                    self._draw_midline(img)
                    cv2.imshow("auto-live", img)
                    cv2.waitKey(3)

        except Exception as ex:  #Exception as ex:
            stop_detector.stop()
            steering.stop()
            motor.stop()
            fl_ranger.cancel()
            fr_ranger.cancel()

            cv2.destroyAllWindows()
            #I am really not sure why this works... I need to commit
            for i in range(4):
                cv2.waitKey(1)

            logging.error(ex)

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    car = AutoCrossCar(with_display = True)
    car.start_auto()


