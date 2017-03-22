#!/usr/bin/env python

import time
import pigpio

FRONT_LEFT_SONAR_PINS = (19, 13)
LEFT_SONAR_PINS = (6, 5)
FRONT_RIGHT_SONAR_PINS = (21, 20)
RIGHT_SONAR_PINS = (24, 23)

class Ranger:
   """
   This class encapsulates a type of acoustic ranger.  In particular
   the type of ranger with separate trigger and echo pins.

   A pulse on the trigger initiates the sonar ping and shortly
   afterwards a sonar pulse is transmitted and the echo pin
   goes high.  The echo pins stays high until a sonar echo is
   received (or the response times-out).  The time between
   the high and low edges indicates the sonar round trip time.
   """

   def __init__(self, pins = FRONT_LEFT_SONAR_PINS):
      """
      The class is instantiated with the Pi to use and the
      gpios connected to the trigger and echo pins.
      """

      trigger, echo = pins
      self.pi    = pigpio.pi()
      self._trig = trigger
      self._echo = echo

      self._ping = False
      self._high = None
      self._time = None

      self._triggered = False

      self._trig_mode = self.pi.get_mode(self._trig)
      self._echo_mode = self.pi.get_mode(self._echo)

      self.pi.set_mode(self._trig, pigpio.OUTPUT)
      self.pi.set_mode(self._echo, pigpio.INPUT)

      self._cb = self.pi.callback(self._trig, pigpio.EITHER_EDGE, self._cbf)
      
      self._inited = True

   def _cbf(self, gpio, level, tick):
      if gpio == self._trig:
         if level == 0: # trigger sent
            self._triggered = True
            self._high = None
      else:
         if self._triggered:
            if level == 1:
               self._high = tick
            else:
               if self._high is not None:
                  self._time = tick - self._high
                  self._high = None
                  self._ping = True

   def read(self):
      """
      Triggers a reading.  The returned reading is the number
      of microseconds for the sonar round-trip.

      round trip cms = round trip time / 1000000.0 * 34030
      """
      if self._inited:
         self._ping = False
         self.pi.gpio_trigger(self._trig)
         start = time.time()
         while not self._ping:
            if (time.time()-start) > 2.0:
               return 20000
            time.sleep(0.001)
         return self._time
      else:
         return None

   def read_cm(self):
       dist = self.read() / 1000000.0 *  17015
       if dist < 2.0:
           return 400
       else:
           return dist

   def cancel(self):
      """
      Cancels the ranger and returns the gpios to their
      original mode.
      """
      if self._inited:
         self._inited = False
         self._cb.cancel()
         self.pi.set_mode(self._trig, self._trig_mode)
         self.pi.set_mode(self._echo, self._echo_mode)

if __name__ == "__main__":

    import time

    import pigpio

    pi = pigpio.pi()

    sonar_fl = Ranger(FRONT_LEFT_SONAR_PINS)
    sonar_l = Ranger(LEFT_SONAR_PINS) #Working...
    sonar_fr = Ranger(FRONT_RIGHT_SONAR_PINS)
    sonar_r = Ranger(RIGHT_SONAR_PINS)

    end = time.time() + 100.0

    try:

        while time.time() < end:

            print "Left: " + str(sonar_l.read_cm()) +" cm" #WORKING
            print "Front Left: " + str(sonar_fl.read_cm()) +" cm" #WORKING
            print "Front Right: " + str(sonar_fr.read_cm()) +" cm" #WORKING
            #print "Right: " + str(sonar_r.read_cm()) +" cm"
            time.sleep(0.03)
    except KeyboardInterrupt:

        sonar_l.cancel()
        sonar_fl.cancel()
        sonar_fr.cancel()
        sonar_r.cancel()

        pi.stop()

