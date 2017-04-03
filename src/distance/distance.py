import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

FRONT_LEFT_SONAR_PINS = (6, 13) 
FRONT_RIGHT_SONAR_PINS = (19, 26)

class Distance:

    def __init__(self, pins):
        self.trigger, self.echo = pins
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        
        GPIO.setup(self.trigger, GPIO.OUT)
        GPIO.output(self.trigger, 0)
        GPIO.setup(self.echo, GPIO.IN)

        self.max_round_trip_time = 0.01

    def read_cm(self):
        GPIO.output(self.trigger, 1)
        time.sleep(0.00001)
        GPIO.output(self.trigger, 0)

        start = time.time()
        while GPIO.input(self.echo) == 0 and time.time() - start < self.max_round_trip_time:
            pass
        
        while GPIO.input(self.echo) == 1 and time.time() - start < self.max_round_trip_time:
            pass

        stop = time.time()

        return (stop - start) * 17150


def main():
    
    range_fl = Distance(FRONT_LEFT_SONAR_PINS)
    range_fr = Distance(FRONT_RIGHT_SONAR_PINS)

    while True:
        dist_fl = range_fl.read_cm()
        dist_fr = range_fr.read_cm()

        print "Distance fr: " + str(dist_fl) + " cm"
        print "Distance fl: " + str(dist_fr) + " cm"
        
        #time.sleep(0.5)

if __name__ == "__main__": main()
