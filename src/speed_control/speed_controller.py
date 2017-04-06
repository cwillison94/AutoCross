
import sys
sys.path.insert(0, "../")
from speed_control.speed_encoder import SpeedEncoder
from car_control.car_motor import CarMotor
from threading import Thread
import cv2
import logging
import time


CONTROLLER_K_P = 0.8
CONTROLLER_K_I = 0
CONTROLLER_K_D = 0.35

MAX_POWER = 18

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
        self.current_speed = 0
        self._average_speed = 0
        self._speeds = []
        self.speed_encoder = SpeedEncoder()
        self.speed_encoder.start()

        self.max_power = max_power
        self.car_motor = CarMotor()
        self.running = False

        self.power_output = 0

    def start(self):
        self.thread = Thread(target=self.maintain_speed_PID, args=())
        self.thread.daemon = True
        self.running = True
        self.thread.start()
        
    def stop(self):
        print "STOPPING CAR MOTOR"
        self.power_output = 0
        self.car_motor.set_percent_power(self.power_output)
        self.desired_speed = 0

    def set_speed(self, speed_m_s):
        self.desired_speed = speed_m_s

    def set_power_percent(self, power_percent):
        power = power_percent * self.max_power
        if power_percent * self.max_power > self.max_power:
            self.power_output = self.max_power
        else:
            self.power_output = power

        self.car_motor.set_percent_power(self.power_output)

    def slowdown(self):
        self.power_output *= 1
        self.car_motor.set_percent_power(self.power_output) 


    def get_speed(self):
        return self.speed_encoder.get_speed_m_s()

    def maintain_speed_PID(self):

        while self.running:

            if self.desired_speed == 0:
                self.power_output = 0
                self.car_motor.set_percent_power(0)
            else:

                self.current_speed = self.speed_encoder.get_speed_m_s()

                print("CURRENT SPEED (m/s): " + str(self.current_speed))

                self.ticks = cv2.getTickCount()
                self.dt = (self.ticks - self.prev_ticks) / cv2.getTickFrequency()


                self.car_speed_error = self.desired_speed - self.current_speed
                self.integral = self.integral + self.car_speed_error * self.dt
                self.derivative = (self.car_speed_error - self.prev_car_speed_error)/self.dt

                power_output_adjustment = CONTROLLER_K_P * self.car_speed_error + CONTROLLER_K_I * self.integral + CONTROLLER_K_D * self.derivative

                self.power_output = self.power_output + power_output_adjustment
                self.prev_car_speed_error = self.car_speed_error
                self.prev_ticks = self.ticks

                

                # clamp power
                if self.power_output > self.max_power:
                    self.power_output = self.max_power
                elif self.power_output < 10:
                    self.power_output = 10

                print("POWER_OUPUT: " + str(self.power_output))

                self.car_motor.set_percent_power(self.power_output)

            time.sleep(0.1)

        

    def cleanup(self):
        self.stop()
        self.speed_encoder.stop()


if __name__=="__main__":
    logger = logging.getLogger(__name__)
    
    logger.info("Speed Controller Imported")
    speed_controller = SpeedController()
    speed_controller.start()

    try:
        speed = input("Speed (m/s): ")
        speed_controller.set_speed(speed)
        while True:

            print("Average speed: ", str(speed_controller.get_speed()))
            time.sleep(1)
    except:
        speed_controller.cleanup()
    
