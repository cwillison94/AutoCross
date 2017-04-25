
import pigpio
import time

DEFAULT_MOTOR_BCM_PIN = 20

#DIRECTION_LEFT = 1
#DIRECTION_RIGHT = 2

PULSEWIDTH_NEUTRAL = 1500
PULSEWIDTH_FULL = 1000#1200 # this value can be decreased to go faster

MIN_PERCENT = 0#10

PULSEWIDTH_RANGE = PULSEWIDTH_NEUTRAL - PULSEWIDTH_FULL

class CarMotor():
	def __init__(self, motor_bmc_pin = DEFAULT_MOTOR_BCM_PIN):
		self.motor_bmc_pin = motor_bmc_pin		

		self.pi = pigpio.pi()
		self.set_percent_power(0)

	def set_percent_power(self, percent):
		if percent > 100:
			percent = 100
		elif percent < MIN_PERCENT and not(percent == 0):
			percent = MIN_PERCENT

		self.pi.set_servo_pulsewidth(self.motor_bmc_pin, PULSEWIDTH_NEUTRAL - (percent/100.) * PULSEWIDTH_RANGE)

	def stop(self):
		self.set_percent_power(0)

	def cleanup(self):
		self.stop()
		#self.pi.stop()

	def set_pulse_width(self, pulse_width):
		self.pi.set_servo_pulsewidth(self.motor_bmc_pin, pulse_width)

#if called directly
if __name__ == "__main__":

	motor = CarMotor()

	running = True

	while running:

		try:
			# pulse_width = input("pulse width= ")

			# if pulse_width == -1:
			# 	running = False
			# else:
			# 	motor.set_pulse_width(int(pulse_width))

			percent = input("percent(0-100):")

			if percent == -1:
				running = False
			else:
				motor.set_percent_power(int(percent))


		except KeyboardInterrupt:
			running = False
			motor.cleanup()

	#motor.cleanup()
	#steering.setPercentDirection(2, 100)



	#try:
		#while True:
			#for i in range(DEFAULT_MIN_ANGLE, DEFAULT_MAX_ANGLE, 1):
				#steering.setAngle(i)
				#time.sleep(0.01)

			#for i in range(DEFAULT_MAX_ANGLE, DEFAULT_MIN_ANGLE, 1):
				#steering.setAngle(i)
				#time.sleep(0.01)
	#except KeyboardInterrupt:
		#steering.cleanup()


