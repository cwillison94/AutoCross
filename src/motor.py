import pigpio
import time


#board pin 13
DEFAULT_MOTOR_BCMM_PIN = 27
DEFAULT_PWM_FREQUENCY = 100
DEFAULT_SWEEP_TIME = 30

SPEED_LOW = 1
SPEED_MID = 2
SPEED_MAX = 3

#TODO rename to SpeedController and work the hall effect sensor into this module
class Motor:
	def __init__(self, motorBCMPin = DEFAULT_MOTOR_BCMM_PIN):
		self.motorBCMPin = motorBCMPin
		
		self.pi = pigpio.pi()
		self.pi.set_PWM_frequency(self.motorBCMPin, DEFAULT_PWM_FREQUENCY)
		
	#to be used until the speed can be determined	
	def setSpeedMode(self, speedMode):
		
		if speedMode == SPEED_LOW:
			setPower(20)
		elif speedMode == SPEED_MID:
			setPower(50)
		elif speedMode == SPEED_MAX:
			setPower(100)
		
	def setPower(self, percentPower):
		self.pi.set_PWM_dutycycle(self.motorBCMPin, int((percentPower/100.) * 255))
		
	
	def stop(self):
		self.setPower(0)
		
	def cleanup(self):
		self.setPower(0)
		self.pi.stop()
		
	def sweep(self):
		for i in range(20, 100):
			self.setPower(i)
			time.sleep(0.01)
			
		for i in range(20, 100):
			self.setPower(100 - i)
			time.sleep(0.01)	
		
	
	def sweep(self, sweepTime = DEFAULT_SWEEP_TIME):
		#run sweep test for 30 seconds
		start = time.time()
		while time.time() - start > sweepTime:
			self.sweep()
		
		self.stop()
		


if __name__ == "__main__":
	print "Starting motor cycle on broadcom pin" + str(DEFAULT_MOTOR_BCMM_PIN)
	
	print "Sweeping... Ctr+C to stop"
	
	carMotor = Motor()
	
	running
	
	try:
		while True:
			carMotor.sweep()
	except KeyboardInterrupt:
		carMotor.cleanup()
		quit()

