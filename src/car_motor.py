
import pigpio
import time

DEFAULT_MOTOR_BCM_PIN = 27

#DIRECTION_LEFT = 1
#DIRECTION_RIGHT = 2

PULSEWIDTH_MIN = 900
PULSEWIDTH_MAX = 1350
PULSEWIDTH_MID = (PULSEWIDTH_MAX + PULSEWIDTH_MIN)/2

PULSEWIDTH_RANGE = PULSEWIDTH_MAX - PULSEWIDTH_MIN

class CarMotor():
	def __init__(self, motor_bmc_pin = DEFAULT_MOTOR_BCM_PIN):
		self.motor_bmc_pin = motor_bmc_pin

		self.pi = pigpio.pi()


	def set_percent_power(self, percent):
		if percent > 100:
			print "Percent is out of range"
		elif (direction == DIRECTION_LEFT):
			pulse = PULSEWIDTH_MID - (percent/100.) * (PULSE_RANGE/2)
			print "pulse width: ", pulse
			self.pi.set_servo_pulsewidth(self.motor_bmc_pin, pulse)
		else:
			pulse = PULSEWIDTH_MID + (percent/100.) * (PULSE_RANGE/2)
			print "pulse width: ", pulse
			self.pi.set_servo_pulsewidth(self.motor_bmc_pin, pulse)

	def set_percent_power(self, percent):
		if percent > 100:
			print "Percent is out of range setting to 100"
			percent = 100
		else:
			self.pi.set_servo_pulsewidth(self.motor_bmc_pin, PULSEWIDTH_MIN + (percent/100.) * PULSEWIDTH_RANGE)

	def reset(self):
		self.setAngle(0)

	def stop(self):
		#stop pwm on pin
		self.pi.set_servo_pulsewidth(self.motor_bmc_pin, 0)

	def cleanup(self):
		self.stop()
		self.pi.stop()

	def set_pulse_width(self, pulse_width):
		self.pi.set_servo_pulsewidth(self.motor_bmc_pin, pulse_width)


def clamp(value, min_value, max_value):
	if value > max_value:
		return  max_value
	elif value < min_value:
		return  min_value
	else:
		return value

#if called directly
if __name__ == "__main__":

	motor = CarMotor()



	while True:

		try:
			pulse_width = input("pulse width= ")
			motor.set_pulse_width(int(pulse_width))

			#percent = input("percent(0-100):")
			#motor.set_percent_power(int(percent))


		except KeyboardInterrupt:
			motor.cleanup()

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


