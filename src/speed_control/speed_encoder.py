# import RPi.GPIO as GPIO
# import time

# GPIO.setmode(GPIO.BCM)
# GPIO.setup(2, GPIO.IN)

# while True:
#     print "Input: ", GPIO.input(2)

import time
import RPi.GPIO as GPIO
import numpy as np
from threading import Thread

DEFAULT_ENCODER_PIN = 5
WHEEL_DIAMETER = 6.5
WHEEL_CIRCUMFERENCE = np.pi * WHEEL_DIAMETER

class SpeedEncoder:

    def __init__(self, pin = DEFAULT_ENCODER_PIN):
        self.pin = pin
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN)
        self.running = False
        self.update_rate =  0.6#0.5 seconds

        self._speeds = []


    def start(self):
        self.running = True
        self.rpm_counter = 0
        self.rpm = 0
        self.time_old = time.time()
        GPIO.add_event_detect(self.pin, GPIO.FALLING, callback=self.callback_function) #bouncetime=200
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()
        return self

    def callback_function(self, channel):
        self.rpm_counter += 1

    def update(self):
        while self.running:
            if (time.time() - self.time_old > self.update_rate):
                # self.rpm =
                self.rpm = (self.rpm + self.rpm_counter * 30 * self.update_rate)/2.

                self.rpm_counter = 0
                self.time_old = time.time()
            # time.sleep(0.01) # sleep for 10 ms

    def get_rpm(self):
        return self.rpm

    def get_speed_m_s(self):
        return self.rpm * WHEEL_CIRCUMFERENCE / 6000

    def _add_to_average_speed(self, speed):
        if len(self._speeds) < 5:
            self._speeds.append(speed)
        else:
            self._speeds.pop(0)
            self._speeds.append(speed)

        # print "SPeed len: ", len(self._speeds)

        if len(self._speeds) > 0:
            self._average_speed = sum(self._speeds) / len(self._speeds)


    def stop(self):
        self.running = False
        GPIO.cleanup()

if __name__ == "__main__":
    encoder = SpeedEndcoder()
    encoder.start()

    try:

        while True:
            print "speed (m/s): ", encoder.get_speed_m_s()
    except:
        encoder.stop()
