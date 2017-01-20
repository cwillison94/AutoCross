import RPi.GPIO as GPIO
import time


#global pin locations
TRIG = 7
ECHO = 12

def setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.output(TRIG, 0)
    GPIO.setup(ECHO, GPIO.IN)

    #wait for initialization
    time.sleep(0.1)

def getUltraSonicDistanceCM():
    GPIO.ouput(TRIG, HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG, LOW)

    start = time.time()
    while GPIO.input(ECHO) == LOW:
        pass
    
    while GPIO.input(ECHO) == HIGH:
        pass

    stop = time.time()

    return (stop - start) * 17000

def main():
    setup()

    while True:
        dist = getUltraSonicDistanceCM()

        print "Distance: " + str(dist) + " cm"

if __name__ == "__main__": main()