
import pigpio
import time

#Broadcom gpio pin (board pin 37)
DEFAULT_SERVO_BCM_PIN = 26
DEFAULT_MIN_ANGLE = -20 
DEFAULT_MAX_ANGLE = 20

class Steering():
	def __init__(self, servo_bcm_pin = DEFAULT_SERVO_BCM_PIN , min_angle = DEFAULT_MIN_ANGLE, max_angle = DEFAULT_MAX_ANGLE):
		self.servo_bcm_pin = servo_bcm_pin
		self.min_angle = min_angle
		self.max_angle = max_angle
		
		self.pi = pigpio.pi()
		
	def setAngle(self, angle):
	
		#clamp angle to allowable on rc car
		angle = clamp(angle, self.min_angle, self.max_angle)
	
		#(2500 - 500)/180 = 11.111 
		self.pi.set_servo_pulsewidth(self.servo_bcm_pin, 1500 + angle * 11.1)
	
	
	def cleanup(self):
		#stop pwm on pin
		self.pi.set_servo_pulsewidth(self.servo_bcm_pin, 0)
		self.pi.stop()
		
	
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
	
	steering = Steering()
	
	
	try:
		while True:
			for i in range(DEFAULT_MIN_ANGLE, DEFAULT_MAX_ANGLE, 1):
				steering.setAngle(i)
				time.sleep(0.5)
				
			for i in range(DEFAULT_MAX_ANGLE, DEFAULT_MIN_ANGLE, 1):
				steering.setAngle(i)
				time.sleep(0.5)
	except KeyboardInterrupt:
		steering.cleanup()
	
		
