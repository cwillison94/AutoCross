
import pigpio
import time

#Broadcom gpio pin (board pin 37)
DEFAULT_SERVO_BCM_PIN = 26
DEFAULT_MIN_ANGLE = -20
DEFAULT_MAX_ANGLE = 20

DIRECTION_LEFT = 1
DIRECTION_RIGHT = 2

PULSEWIDTH_LEFT = 900
PULSEWIDTH_RIGHT = 1350
PULSEWIDTH_MID = (PULSEWIDTH_LEFT + PULSEWIDTH_RIGHT)/2

PULSE_RANGE = PULSEWIDTH_RIGHT - PULSEWIDTH_LEFT

class Steering():
	def __init__(self, servo_bcm_pin = DEFAULT_SERVO_BCM_PIN , min_angle = DEFAULT_MIN_ANGLE, max_angle = DEFAULT_MAX_ANGLE):
		self.servo_bcm_pin = servo_bcm_pin
		self.min_angle = min_angle
		self.max_angle = max_angle

		self.pi = pigpio.pi()

	def set_percent_direction(self, percent):
		if percent > 100:
			print "Percent is out of range"
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
		self.pi.set_servo_pulsewidth(self.servo_bcm_pin, 0)

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

	print "Starting sweep cycle on broadcom pin" + str(DEFAULT_SERVO_BCM_PIN)
	print "Sweeping through " + str(DEFAULT_MIN_ANGLE) + " to " + str(DEFAULT_MAX_ANGLE)

	print "Sweeping... Ctr+C to stop"

	steering = Steering(servo_bcm_pin = 27)



	while True:

		try:
			#pulse_width = input("pulse width= ")
			#steering.set_pulse_width(int(pulse_width))
			#direction = raw_input("Direction (l/r):")
			percent = input("percent(0-100):")

			#if direction == "l":
			steering.set_percent(int(percent))
			#else:
			#	steering.set_percent(DIRECTION_RIGHT, int(percent))
		except KeyboardInterrupt:
			steering.cleanup()

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


