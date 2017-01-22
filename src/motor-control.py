import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

GPIO.setup(13, GPIO.OUT)

p = GPIO.PWM(13,207)
p.start(0)

print "Starting"

try:
	while True:
		for i in range(20, 100):
			p.ChangeDutyCycle(i)
			time.sleep(0.02)
			
		for i in range(20, 100):
			p.ChangeDutyCycle(100 - i)
			time.sleep(0.02)
	
except KeyboardInterrupt:
	print "Stopping"
	pass
	
p.stop()
GPIO.cleanup()

