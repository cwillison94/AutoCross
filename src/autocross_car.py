import sys
sys.path.append('/usr/local/lib/python2.7/site-packages')
import os
import logging
import time
import cv2
import multiprocessing
import traceback
from picamera.array import PiRGBArray
from picamera import PiCamera

# import autocross modules
import helpers.draw_helper
import vision.stop_sign_detector
from vision.lane_detector import LaneDetector
from car_control.car_motor import CarMotor
from car_control.car_steering import CarSteering
from distance.distance import Distance, FRONT_LEFT_SONAR_PINS, FRONT_RIGHT_SONAR_PINS
from v2v.v2v_module import V2VModule
from speed_control.speed_controller_nothread import SpeedController
#from speed_control.speed_encoder import SpeedEncoder
from helpers.PID import PID

DEFAULT_CAMERA_PARAMS = (608, 608, 32)

STEERING_K_P = 0.64#0.61
STEERING_K_I = 0
STEERING_K_D = 0.19

STOPPING_K_P = 0.2
STOPPING_K_I = 0.0
STOPPING_K_D = 0.1

# States
IDLE = 0
OBSTACLE_DETECTED = 10
STOP_SIGN_DETECTED = 20
FOLLOW_LANES = 30
INTERSECTION_DETECTED = 40
WAITING_AT_INTERSECTION = 50
PROCEED_THROUGH_INTERSECTION = 60

#Turning actions
ACTION_STRAIGHT = 100
ACTION_TURN_LEFT = 110
ACTION_TURN_RIGHT = 120

SPEED_HIGH = 7.5#15
SPEED_LOW = 7.0#15

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

        # speed in m/s
        self.car_desired_speed = 0.0#0.8#1.0

        self.min_obj_dist = 50

        self.v2v_ready_for_transit = False
        self.v2v_module = V2VModule(self._v2v_on_ready_callback, debug_mode = True)
        self.state = -1
        self.previous_state = -1

        self.camera = None

        self.turn_action = ACTION_STRAIGHT

        # [is_detected, normalized_dist]
        self.stop_sign_detected = False
        self.stop_sign_distance = 0
        self.stop_sign_position = []
        self.stop_img = None

    def _initialize_camera(self):
        if self.camera_initiliazed:
            logging.info("Camera has already been initialized.")
        else:
            logging.info("Initiliazing camera")
            self.camera = PiCamera()
            self.camera.resolution = self.resolution
            self.camera.framerate = self.framerate
            self.capture = PiRGBArray(self.camera, self.camera.resolution)
            logging.info("Camera Initiliazed")

            # wait for camera module to start up
            #time.sleep(0.5)


    def _v2v_on_ready_callback(self):
        logging.info("Received Ready for transit")
        self.v2v_ready_for_transit = True


    def get_state_description(self, state):
        if state == OBSTACLE_DETECTED:
            return "OBSTACLE_DETECTED"
        elif state == FOLLOW_LANES:
            return "FOLLOW_LANES"
        elif state == STOP_SIGN_DETECTED:
            return "STOP_SIGN_DETECTED"
        elif state == WAITING_AT_INTERSECTION:
            return "WAITING_AT_INTERSECTION"
        elif state == PROCEED_THROUGH_INTERSECTION:
            return "PROCEED_THROUGH_INTERSECTION"
        elif state == INTERSECTION_DETECTED:
            return "INTERSECTION_DETECTED"
        else:
            return "NONE"

    def _lane_base_distance(self, lane):
        if lane is not None:
            return lane[4]
        else:
            return 0
    
    def _lane_is_approximated(self, lane):
        if lane is not None:
            return lane[5]
        else:
            return True

    def get_steering_error(self, left_lane, right_lane):
        base_left = self._lane_base_distance(left_lane)
        base_right = self._lane_base_distance(right_lane)
        return (base_left + base_right)/2


    def _log_exception(self):
        exc_type, exc_obj, tb = sys.exc_info()
        f = tb.tb_frame
        lineno = tb.tb_lineno
        filename = f.f_code.co_filename
        linecache.checkcache(filename)
        line = linecache.getline(filename, lineno, f.f_globals)
        logging.error('EXCEPTION IN ({}, LINE {} "{}"): {}'.format(filename, lineno, line.strip(), exc_obj))


    def cleanup(self):
        self.car_motor.set_percent_power(0)

    def _stop_sign_detected_callback(self, stop_sign_info):
        #print "stop_sign_info= ", stop_sign_info
        if self.state != PROCEED_THROUGH_INTERSECTION:
            self.stop_sign_detected = stop_sign_info[0]
            self.stop_sign_distance = stop_sign_info[1]
            self.stop_sign_position = stop_sign_info[2]
            self.stop_img = stop_sign_info[3]

    
    def adjust_steering(self, lanes):
        left_lane = lanes[0]
        right_lane = lanes[1]
        stop_line = lanes[2]

        steering_error = self.get_steering_error(left_lane, right_lane)
        steering_output = self.steering_pid.update(error = steering_error)

        if stop_line is not None:
            logging.info("Stop line detected. Stopping")                    
            self.state = WAITING_AT_INTERSECTION

            start_intersection_time = time.time()

            self.v2v_module.set_stopped(0)

            speed_controller.stop()
            # self.car_motor.set_percent_power(0)

        elif left_lane is not None and right_lane is not None:  
            self.steering.set_percent_direction(steering_output)

        return (steering_output, steering_error)

    
    def check_for_obstacle(self):
        dist_fl = self.distance_fl.read_cm()
        dist_fr = self.distance_fr.read_cm()

        return (dist_fl < self.min_obj_dist or dist_fr < self.min_obj_dist)

    def draw_img(self, img, lanes, steering_output, steering_error):
        if self.with_display:

            if lanes is not None:
                left_lane = lanes[0]
                right_lane = lanes[1]
                stop_line = lanes[2]

                helpers.draw_helper.draw_lanes(img, left_lane, right_lane)
                helpers.draw_helper.draw_line(img, stop_line)
                helpers.draw_helper.draw_rectangle(img, self.stop_sign_position)

            if steering_output is not None:

                helpers.draw_helper.draw_steering_output(img, steering_output)
                helpers.draw_helper.draw_error(img, int(steering_error), self.car_midline, self.lane_detector.base_distance_height)

            helpers.draw_helper.draw_midline(img, self.car_midline, self.lane_detector.base_distance_height)
            helpers.draw_helper.draw_roi(img, [int(x) for x in self.lane_detector.roi])


            cv2.imshow("auto-live", img)
            if cv2.waitKey(3) & 0xFF == ord('q'):
                self.camera.close()
                #break

    # get direction of approach from color of stop sign
    def get_stop_direction(self):
        mean_color = cv2.mean(self.stop_img)
        print("STOP BGR MEANS: ", mean_color)
        bgr_max = max(mean_color)
        bgr_min = min(mean_color)

        if bgr_max < 80:
            print("BLACK/GRAY")
            return 0
        elif mean_color.index(bgr_max) == 0:
            print("BLUE")
            return 1
        elif mean_color.index(bgr_max) == 1:
            print("GREEN")
            return 2
        else:
            print("RED")
            return 3


    def start_auto(self):
        logging.info("Starting AutoCross car in automatic mode")

        try:
            lanes = None
            steering_error = None
            steering_output = None

            self.distance_fl = Distance(FRONT_LEFT_SONAR_PINS)
            self.distance_fr = Distance(FRONT_RIGHT_SONAR_PINS)

            speed_controller = SpeedController()

            logging.info("Creating lane detector")
            self.lane_detector = LaneDetector(self.resolution[0], self.resolution[1], enable_stop_line_detection = False, debug_mode = self.with_display)

            logging.info("Creating stop detector")
            stop_detection_process_pool = multiprocessing.Pool(processes = 3) #processes = 3
            
            logging.info("Initializing steering")
            self.steering = CarSteering()

            self.steering_pid = PID(STEERING_K_P, STEERING_K_I, STEERING_K_D)
            stopping_pid = PID(STOPPING_K_P, STOPPING_K_I, STOPPING_K_D)

            logging.info("Starting V2V module")
            self.v2v_module.start()

            # Set drawing level
            helpers.draw_helper.set_display_on(self.with_display)

            logging.info("Starting main loop and initialiazing camera. Please wait...")
            
            self.camera = PiCamera(resolution = self.resolution, framerate = self.framerate)
            rawCapture = PiRGBArray(self.camera, size = self.resolution)
            for frame in self.camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):


                # obstacle detection must be checked regardless of state
                if self.check_for_obstacle():
                    logging.info("OBSTACLE DETECTED")
                    self.previous_state = self.state
                    self.state = OBSTACLE_DETECTED
                    speed_controller.stop()

                start_loop_time = time.time()

                img = frame.array
                rawCapture.truncate(0)          
                
                stop_detection_process_pool.apply_async(vision.stop_sign_detector.detect_stop_sign, args=(img,), callback=self._stop_sign_detected_callback )
                # stop_s = time.time()
                # derp = vision.stop_sign_detector.detect_stop_sign(img)
                # stop_end = time.time() - stop_s
                # print("stop loop: %f" % (stop_end))
                if self.state == OBSTACLE_DETECTED:

                    while self.check_for_obstacle():
                        pass

                    self.state = self.previous_state
                    logging.info("obstacle removed. returning to previous state")

                elif self.state == STOP_SIGN_DETECTED:
                    logging.info("STATE: STOP SIGN DETECTED. dist: %d" % (self.stop_sign_distance))

                    speed_controller.set_speed(SPEED_LOW)

                    lanes = self.lane_detector.detect(img)   
                    left_lane = lanes[0]
                    right_lane = lanes[1]
                    stop_line = lanes[2]

                    # continue following lanes until we are close enough to stop completely
                    # note: larger distance value means closer
                    if (self.stop_sign_distance < 50):
                        steering_output, steering_error = self.adjust_steering(lanes)
                    else:
                        # close enough to stop
                        speed_controller.stop()
                        self.state = WAITING_AT_INTERSECTION


                #switch to STOPPED when we are at threshold
                elif self.state == WAITING_AT_INTERSECTION:
                    direction = self.get_stop_direction()
                    self.v2v_module.set_stopped(direction)
                    while not(self.v2v_ready_for_transit):
                        pass
                    proceed_through_intersection_time = time.time()
                    self.state = PROCEED_THROUGH_INTERSECTION
                    self.v2v_module.set_in_transit()
                    self.v2v_ready_for_transit = False


                elif self.state == PROCEED_THROUGH_INTERSECTION:

                    # clear stop data
                    self.stop_sign_detected = False
                    self.stop_sign_distance = 0
                    self.stop_sign_position = []
                    self.stop_img = None

                    speed_controller.set_speed(SPEED_HIGH)
                    # use previous lanes until later
                    steering_output, steering_error = self.adjust_steering(lanes)
                    # assume it take 3 sec to get through intersection
                    if time.time() > (proceed_through_intersection_time + 3):
                        self.v2v_module.set_cleared()
                        self.state = FOLLOW_LANES

                elif self.state == FOLLOW_LANES:

                    speed_controller.set_speed(SPEED_HIGH)

                    lanes = self.lane_detector.detect(img)   
                    left_lane = lanes[0]
                    right_lane = lanes[1]
                    stop_line = lanes[2]

                    if self.stop_sign_detected:
                        self.state = STOP_SIGN_DETECTED
                    else:       
                        steering_output, steering_error = self.adjust_steering(lanes)                   

                else:
                    self.state = FOLLOW_LANES

                self.draw_img(img, lanes, steering_output, steering_error)
                #logging.debug("State: " + self.get_state_description(self.state))
                looptime = ((time.time() - start_loop_time) * 1000)
                if looptime > 200:
                    logging.critical("Loop time (ms): " + str(looptime) + " ms")
                #END OF FRAME LOOP

        except Exception as ex:
            logging.info("CLOSING DOWN")
            traceback.print_exc()
            if self.camera is not None:
                self.camera.close()
            logging.error(ex)

            stop_detection_process_pool.close()
            stop_detection_process_pool.join()
            
            speed_controller.stop()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)#CRITICAL, DEBUG, INFO
    #logging.propagate = False
    car = AutoCrossCar(with_display = False)

    # try: 
    car.start_auto()
    # except:
        # car.cleanup()
       





