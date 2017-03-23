# import RPi.GPIO as GPIO
# import time

# GPIO.setmode(GPIO.BCM)
# GPIO.setup(2, GPIO.IN)

# while True:
#     print "Input: ", GPIO.input(2)

import time
import pigpio

DEFAULT_ENCODER_PIN = 2

class SpeedEndcoder:

    def __init__(self, pin = DEFAULT_ENCODER_PIN):

        self.pi = pigpio.pi()
        self.pin = pin
        self.running = False

    def start(self):
        self.running = True
        self.rev_counter = 0
        self.rpm = 0
        self.time_old = time.time()
        self.pi.set_mode(self.pin, pigio.INPUT)
        self.callback = self.pi.callback(self.pin, pigpio.RISING_EDGE, self.callback_function)
        Thread(target=self.update, args=()).start()
        return self

    def callback_function(self, gpio, level, tick):
        rev_counter += 1

    def update(self):
        while self.running:
            self.rpm = self.rev_counter / ((1/60.) * (time.time() - self.time_old))

    def get_rpm(self):
        return self.rpm
    
    def stop(self):
        self.running = False
        self.callback.cancel()





