import sys
sys.path.insert(0, "../")
from car_control.car_motor import CarMotor
import logging
import time

MAX_POWER = 18

class SpeedController:
	def __init__(self, max_power = MAX_POWER):

		self.max_power = max_power
		self.car_motor = CarMotor()
		self.power_output = 0


	def set_speed(self, speed):
		if self.power_output != speed:
			print("setting speed to %d" % (speed))
			self.power_output = speed
			self.car_motor.set_percent_power(speed)


	def stop(self):
		self.set_speed(0)
