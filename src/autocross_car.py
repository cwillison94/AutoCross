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
from speed_control.speed_controller import SpeedController
from speed_control.speed_encoder import SpeedEncoder

DEFAULT_CAMERA_PARAMS = (608, 608, 32)

CONTROLLER_K_P = 0.61
CONTROLLER_K_I = 0
CONTROLLER_K_D = 0.19

# States
IDLE = 0
OBSTACLE_DETECTED = 10
STOP_SIGN_DETECTED = 20
DETECT_LANES = 30
INTERSECTION_DETECTED = 40
WAITING_AT_INTERSECTION = 50
PROCEED_THROUGH_INTERSECTION = 60

#Turning actions
ACTION_STRAIGHT = 100
ACTION_TURN_LEFT = 110
ACTION_TURN_RIGHT = 120

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
        self.car_desired_speed = 0.2#0.8#1.0

        # TODO: REMOVE THIS AFTER TESTING
        # self.car_power = 15

        self.min_obj_dist = 50

        self.v2v_ready_for_transit = False
        self.v2v_module = V2VModule(self._v2v_on_ready_callback, debug_mode = True)
        self.state = -1

        self.camera = None

        self.turn_action = ACTION_STRAIGHT

        # [is_detected, normalized_dist]
        self.stop_detected_info = (False, 100, [])

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

    def _steering_PID(self, left_lane, right_lane):
        self.ticks = cv2.getTickCount()
        self.dt = (self.ticks - self.prev_ticks) / cv2.getTickFrequency()

        base_left = self._lane_base_distance(left_lane)
        base_right = self._lane_base_distance(right_lane)

        slope_adjustment = 0
        #print "right_lane, ", right_lane
        # if not self._lane_is_approximated(left_lane) and  not self._lane_is_approximated(right_lane) and left_lane[2] != left_lane[0] and right_lane[2] != right_lane[0]:
            
            
        #     slope_left = -1 * (left_lane[3] - left_lane[1])/float(left_lane[2] - left_lane[0])
        #     slope_right = -1 * (right_lane[3] - right_lane[1])/float(right_lane[2] - right_lane[0])

        #     slope_adjustment = 1/ ((slope_left + slope_right)/2.)
        #     logging.debug("SLOPE ADJUSTMENT = " + str(slope_adjustment))

        self.car_error = (base_left + base_right + slope_adjustment)/2
        self.integral = self.integral + self.car_error * self.dt
        self.derivative = (self.car_error - self.prev_car_error)/self.dt

        output = CONTROLLER_K_P * self.car_error + CONTROLLER_K_I * self.integral + CONTROLLER_K_D * self.derivative
        
        self.prev_ticks = self.ticks        
        self.prev_car_error = self.car_error

        return output

    def get_state_description(self, state):
        if state == OBSTACLE_DETECTED:
            return "OBSTACLE_DETECTED"
        elif state == DETECT_LANES:
            return "DETECT_LANES"
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
        self.stop_detected_info = stop_sign_info

    def start_auto(self):
        logging.info("Starting AutoCross car in automatic mode")

        try:

            distance_fl = Distance(FRONT_LEFT_SONAR_PINS)
            distance_fr = Distance(FRONT_RIGHT_SONAR_PINS)
            speed_controller = SpeedController()
            speed_controller.start()

            #self._initialize_camera()

            logging.info("Creating lane detector")
            lane_detector = LaneDetector(300, self.resolution[0], self.resolution[1], enable_stop_line_detection = False, debug_mode = self.with_display)

            logging.info("Creating stop detector")
            stop_detection_process_pool = multiprocessing.Pool() #processes = 3
            

            logging.info("Initializing steering")
            steering = CarSteering()
            self.car_motor = CarMotor()

            logging.info("Starting V2V mOdule")
            self.v2v_module.start()

            # Set drawing level
            helpers.draw_helper.set_display_on(self.with_display)

            logging.info("Starting main loop and initialiazing camera. Please wait...")
            
            self.camera = PiCamera(resolution = self.resolution, framerate = self.framerate)
            rawCapture = PiRGBArray(self.camera, size = self.resolution)
            for frame in self.camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

                start_loop_time = time.time()
                img = frame.array
                rawCapture.truncate(0)            

                dist_fl = distance_fl.read_cm()
                dist_fr = distance_fr.read_cm()
                
                stop_detection_process_pool.apply_async(vision.stop_sign_detector.detect_stop_sign, args=(img,), callback=self._stop_sign_detected_callback )
                lanes = lane_detector.detect(img)   

                left_lane = lanes[0]
                right_lane = lanes[1]
                stop_line = lanes[2]

                # drawing functions for lanes and stop sign
                img = helpers.draw_helper.draw_lanes(img, left_lane, right_lane)
                img = helpers.draw_helper.draw_line(img, stop_line)
                img = helpers.draw_helper.draw_rectangle(img, self.stop_detected_info[2])
                
                if dist_fl < self.min_obj_dist or dist_fr < self.min_obj_dist:
                    logging.info("OBSTACLE DETECTED")
                    # self.car_motor.set_percent_power(0)
                    speed_controller.stop()
                    
                elif self.state == STOP_SIGN_DETECTED:
                    pass
                elif self.state == WAITING_AT_INTERSECTION:
                    if self.v2v_ready_for_transit: #or (time.time() - start_intersection_time) > 3:
                        proceed_through_intersection_time = time.time()
                        self.state = PROCEED_THROUGH_INTERSECTION
                        self.v2v_module.set_in_transit()
                        self.v2v_ready_for_transit = False

                elif self.state == PROCEED_THROUGH_INTERSECTION:
                    # lanes = lane_detector.detect(img)
                    # left_lane = lanes[0]
                    # right_lane = lanes[1]

                    if self.turn_action == ACTION_STRAIGHT:

                        if self.stop_detected_info[0]:
                            logging.info("STOP SIGN STILL DETECTED")
                            left_lane = intersection_left_lane
                            right_lane = intersection_right_lane
                        elif self._lane_is_approximated(left_lane) and self._lane_is_approximated(right_lane):
                            left_lane = intersection_left_lane
                            right_lane = intersection_right_lane
                        else:
                            self.v2v_module.set_cleared()
                            self.state = DETECT_LANES
                        
                        # elif self._lane_is_approximated(left_lane):
                        #     left_lane = intersection_left_lane
                        # elif self._lane_is_approximated(right_lane):
                        #     right_lane = intersection_right_lane

                        # self.car_motor.set_percent_power(self.car_power)
                        speed_controller.set_speed(self.car_desired_speed)

                        steering_output = self._steering_PID(left_lane, right_lane)
                        steering.set_percent_direction(steering_output)
                    elif self.turn_action == ACTION_TURN_LEFT:
                        pass
                    elif self.turn_action == ACTION_TURN_RIGHT:
                        pass

                    # if stop_line is not None:
                    #     proceed_through_intersection_time = time.time()

                    # if  stop_line is not None and (time.time() - proceed_through_intersection_time) < 3 or (self._lane_is_approximated(left_lane) or self._lane_is_approximated(right_lane)):
                    #     left_lane = intersection_left_lane
                    #     right_lane = intersection_right_lane
                    # elif not self._lane_is_approximated(left_lane) and not self._lane_is_approximated(right_lane):
                    #     self.v2v_module.set_cleared()
                    #     self.state = DETECT_LANES
                    


                elif self.stop_detected_info[0]:
                    if self.stop_detected_info[1] > 40: #stop sign detected...
                        self.state = WAITING_AT_INTERSECTION
                        # self.car_motor.set_percent_power(0)
                        speed_controller.stop()                    
                        self.v2v_module.set_stopped(0)
                        intersection_left_lane = left_lane
                        intersection_right_lane = right_lane
                    else: 
                        logging.info("SLOWING DOWN")
                        # speed_controller.slowdown()
                else:

                    self.state = DETECT_LANES

                    #self.stop_detector.push_img(img)
                    # lanes = lane_detector.detect(img)

                    # left_lane = lanes[0]
                    # right_lane = lanes[1]
                    # stop_line = lanes[2]
                    # steering_output = 0

                    if stop_line is not None:
                        logging.info("Stop line detected. Stopping")                    
                        self.state = WAITING_AT_INTERSECTION

                        start_intersection_time = time.time()

                        self.v2v_module.set_stopped(0)
                        intersection_left_lane = left_lane
                        intersection_right_lane = right_lane

                        #TODO: Notify V2V at intersection

                        speed_controller.stop()
                        # self.car_motor.set_percent_power(0)

                    elif left_lane is not None and right_lane is not None:
                        self.state = DETECT_LANES
                        ##motor.set_percent_power(self.car_power)
                        speed_controller.set_speed(self.car_desired_speed)
                        # self.car_motor.set_percent_power(self.car_power)
                        steering_output = self._steering_PID(left_lane, right_lane)
                        steering.set_percent_direction(steering_output)

                        img = helpers.draw_helper.draw_steering_output(img, steering_output)
                    # img = helpers.draw_helper.draw_lanes(img, left_lane, right_lane, steering_output)
                    # img = helpers.draw_helper.draw_line(img, stop_line)
                
                # speed_controller.maintain_speed_PID()
                

                if self.with_display:
                    img = helpers.draw_helper.draw_midline(img, self.car_midline)
                    cv2.imshow("auto-live", img)
                    if cv2.waitKey(3) & 0xFF == ord('q'):
                        self.camera.close()
                        break

                logging.debug("State: " + self.get_state_description(self.state))
                logging.debug("Loop time (ms): " + str((time.time() - start_loop_time) * 1000) + " ms")
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
    logging.basicConfig(level=logging.DEBUG)#DEBUG
    #logging.propagate = False
    car = AutoCrossCar(with_display = True)

    # try: 
    car.start_auto()
    # except:
        # car.cleanup()
       




