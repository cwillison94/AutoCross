import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

FRONT_LEFT_SONAR_PINS = (16, 12) # CHANGE PIN 19 in use!! 
LEFT_SONAR_PINS = (6, 5)

FRONT_RIGHT_SONAR_PINS = (21, 20)
RIGHT_SONAR_PINS = (24, 23)

TRIG, ECHO = FRONT_RIGHT_SONAR_PINS

def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.output(TRIG, 0)
    GPIO.setup(ECHO, GPIO.IN)

    #wait for initialization
    #time.sleep(0.1)

def read_cm():
    GPIO.output(TRIG, 1)
    time.sleep(0.00001)
    GPIO.output(TRIG, 0)

    start = time.time()
    while GPIO.input(ECHO) == 0 and time.time() - start < 0.01:
        pass
    
    while GPIO.input(ECHO) == 1 and time.time() - start < 0.01:
        pass

    stop = time.time()

    return (stop - start) * 17150

def main():
    setup()

    while True:
        dist = read_cm()

        print "Distance: " + str(dist) + " cm"
        
        #time.sleep(0.5)

if __name__ == "__main__": main()
