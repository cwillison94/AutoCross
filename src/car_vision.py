import RPi.GPIO as GPIO
import time


#global pin locations
TRIG = 7
ECHO = 12

def setup():

    GPIO.setmode(GPIO.BOARD)
    
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.output(TRIG, 0)
    GPIO.setup(ECHO, GPIO.IN)

    #wait for initialization
    time.sleep(0.1)

def getUltraSonicDistanceCM():
    GPIO.output(TRIG, 1)
    time.sleep(0.00001)
    GPIO.output(TRIG, 0)

    start = time.time()
    while GPIO.input(ECHO) == 0:
        pass
    
    while GPIO.input(ECHO) == 1:
        pass

    stop = time.time()

    return (stop - start) * 17150

def main():
    setup()

    while True:
        dist = getUltraSonicDistanceCM()

        print "Distance: " + str(dist) + " cm"
        
        time.sleep(0.5)

if __name__ == "__main__": main()
import RPi.GPIO as GPIO
import time


#global pin locations
TRIG = 7
ECHO = 12

def setup():

    GPIO.setmode(GPIO.BOARD)
    
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.output(TRIG, 0)
    GPIO.setup(ECHO, GPIO.IN)

    #wait for initialization
    time.sleep(0.1)

def getUltraSonicDistanceCM():
    GPIO.output(TRIG, 1)
    time.sleep(0.00001)
    GPIO.output(TRIG, 0)

    start = time.time()
    while GPIO.input(ECHO) == 0:
        pass
    
    while GPIO.input(ECHO) == 1:
        pass

    stop = time.time()

    return (stop - start) * 17150

def main():
    setup()

    while True:
        dist = getUltraSonicDistanceCM()

        print "Distance: " + str(dist) + " cm"
        
        time.sleep(0.5)

if __name__ == "__main__": main()
