
import pigpio
import time

DEFAULT_SERVO_BCM_PIN = 21
DEFAULT_MIN_ANGLE = -20
DEFAULT_MAX_ANGLE = 20

DIRECTION_LEFT = 1
DIRECTION_RIGHT = 2

PULSEWIDTH_LEFT = 900
PULSEWIDTH_RIGHT = 1350
PULSEWIDTH_MID = (PULSEWIDTH_LEFT + PULSEWIDTH_RIGHT)/2

PULSE_RANGE = PULSEWIDTH_RIGHT - PULSEWIDTH_LEFT

class CarSteering():
	def __init__(self, servo_bcm_pin = DEFAULT_SERVO_BCM_PIN):
		self.servo_bcm_pin = servo_bcm_pin
		self.pi = pigpio.pi()

	def set_percent_direction(self, percent):
		if percent > 100:
			percent = 100
		elif percent < -100:
			percent = -100

		pulse = PULSEWIDTH_MID + (percent/100.) * (PULSE_RANGE/2)
		self.pi.set_servo_pulsewidth(self.servo_bcm_pin, pulse)


	def set_percent(self, percent):
		self.pi.set_servo_pulsewidth(self.servo_bcm_pin, PULSEWIDTH_LEFT + (percent/100.) * PULSE_RANGE)

	def reset(self):
		self.set_percent_direction(0)

	def stop(self):
		#stop pwm on pin
		self.set_percent_direction(0)

	def cleanup(self):
		self.stop()
		self.pi.stop()

	def set_pulse_width(self, pulse_width):
		self.pi.set_servo_pulsewidth(self.servo_bcm_pin, pulse_width)


def clamp(value, min_value, max_value):
	if value > max_value:
		return  max_value
	elif value < min_value:
		return  min_value
	else:
		return value

#if called directly
if __name__ == "__main__":
	steering = CarSteering()
	while True:

		try:
			percent = input("percent(-100-100):")

			steering.set_percent_direction(int(percent))

		except KeyboardInterrupt:
			steering.cleanup()

