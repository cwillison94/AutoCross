
from speed_control.speed_encoder import SpeedEncoder
from car_control.car_motor import CarMotor
import cv2
import logging


CONTROLLER_K_P = 5.0
CONTROLLER_K_I = 0
CONTROLLER_K_D = 0.2

MAX_POWER = 15

logger = logging.getLogger(__name__)
logger.info("Speed Controller Imported")

class SpeedController:
    def __init__(self, max_power = MAX_POWER):
        # setup speed PID variables
        self.ticks = 0
        self.prev_ticks = 0
        self.dt = 0
        self.prev_car_speed_error = 0
        self.car_speed_error = 0
        self.integral = 0
        self.derivative = 0

        # speed in m/s
        self.desired_speed = 0
        self.speed_encoder = SpeedEncoder()
        self.speed_encoder.start()

        self.max_power = max_power
        self.car_motor = CarMotor()

        self.power_output = 0

    def stop(self):
        print "STOPPING CAR MOTOR"
        self.car_motor.set_percent_power(0)
        self.desired_speed = 0

    def set_speed(self, speed_m_s):
        self.desired_speed = speed_m_s

    def maintain_speed_PID(self):

        if self.desired_speed == 0:
            self.power_output = 0
            self.car_motor.set_percent_power(0)
            return 
        else:

            current_speed = self.speed_encoder.get_speed_m_s()
            # logger.debug("Current Speed (m/s): " + str(current_speed))

            self.ticks = cv2.getTickCount()
            self.dt = (self.ticks - self.prev_ticks) / cv2.getTickFrequency()

            # logger.debug("dt " + str(self.dt))

            self.car_speed_error = self.desired_speed - current_speed
            self.integral = self.integral + self.car_speed_error * self.dt
            self.derivative = (self.car_speed_error - self.prev_car_speed_error)/self.dt

            power_output_adjustment = CONTROLLER_K_P * self.car_speed_error + CONTROLLER_K_I * self.integral + CONTROLLER_K_D * self.derivative

            # logger.debug("Power output adjustment " + str(power_output_adjustment))

            self.power_output = self.power_output + power_output_adjustment
            self.prev_car_speed_error = self.car_speed_error
            self.prev_ticks = self.ticks

            # logger.debug("power_output: " + str(self.power_output))

            # clamp power
            if self.power_output > self.max_power:
                self.power_output = self.max_power

        # set power output here
            self.car_motor.set_percent_power(13)
            #self.car_motor.set_percent_power(self.power_output)
        
            return 
        

    def cleanup(self):
        self.car_motor.set_percent_power(0)
        self.stop()
        self.speed_encoder.stop()
