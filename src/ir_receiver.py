import pigpio
import time
import thread

GPIO_IR_BCM_PIN = 4
TOLERANCE = 15
TOLER_MIN =  (100 - TOLERANCE) / 100.0
TOLER_MAX =  (100 + TOLERANCE) / 100.0
GLITCH = 100
PRE_MS     = 200
POST_MS    = 15
FREQ       = 38.0
SHORT      = 10
GAP_MS     = 100

POST_US    = POST_MS * 1000
PRE_US     = PRE_MS  * 1000
GAP_S      = GAP_MS  / 1000.0

last_tick = 0
in_code = False
code = []
fetching_code = False
listening = False


KEY_0 = "0"
KEY_1 = "1"
KEY_2 = "2"
KEY_3 = "3"
KEY_4 = "4"
KEY_5 = "5"
KEY_6 = "6"
KEY_7 = "7"
KEY_8 = "8"
KEY_9 = "9"
KEY_100_PLUS = "100+"
KEY_200_PLUS = "200+"
KEY_CH = "CH"
KEY_CH_PLUS = "CH+"
KEY_CH_MINUS = "CH-"
KEY_EQ = "eq"
KEY_NEXT = "next"
KEY_PLAY = "play"
KEY_PREV = "prev"
KEY_V_MINUS = "v-"
KEY_V_PLUS = "v+"

IR_WAVES = {KEY_0: [8963, 4442, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 540, 580, 540, 580, 1632, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580],
 KEY_1: [8963, 4442, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580],
 KEY_100_PLUS: [8963, 4442, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580],
 KEY_2: [8963, 4442, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580],
 KEY_200_PLUS: [8963, 4442, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580],
 KEY_3: [8963, 4442, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580, 540, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 540, 580, 1632, 580],
 KEY_4: [8963, 4442, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580],
 KEY_5: [8963, 4442, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580],
 KEY_6: [8963, 4442, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580, 540, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580, 540, 580, 1632, 580, 540, 580, 1632, 580, 540, 580, 540, 580, 1632, 580, 540, 580, 1632, 580],
 KEY_7: [8963, 4442, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 540, 580, 1632, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580],
 KEY_8: [8963, 4442, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580, 540, 580, 540, 580, 1632, 580, 540, 580, 1632, 580, 540, 580, 1632, 580, 540, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580, 540, 580, 1632, 580],
 KEY_9: [8963, 4442, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580, 540, 580, 1632, 580, 540, 580, 540, 580, 1632, 580, 540, 580, 1632, 580, 540, 580, 1632, 580, 540, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580],
 KEY_CH: [8963, 4442, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 540, 580, 1632, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580],
 KEY_CH_PLUS: [8963, 4442, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580],
 KEY_CH_MINUS: [8963, 4442, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 540, 580, 540, 580, 1632, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580],
 KEY_EQ: [8963, 4442, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580],
 KEY_NEXT: [8963, 4442, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580],
 KEY_PLAY: [8963, 4442, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580],
 KEY_PREV: [8963, 4442, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 540, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 1632, 580],
 KEY_V_MINUS: [8963, 4442, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 540, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580, 1632, 580],
 KEY_V_PLUS: [8963, 4458, 561, 559, 561, 559, 561, 559, 561, 559, 561, 559, 561, 559, 561, 559, 561, 559, 561, 1653, 561, 1653, 561, 1653, 561, 1653, 561, 1653, 561, 1653, 561, 1653, 561, 1653, 561, 1653, 561, 559, 561, 1653, 561, 559, 561, 1653, 561, 559, 561, 559, 561, 559, 561, 559, 561, 1653, 561, 559, 561, 1653, 561, 559, 561, 1653, 561, 1653, 561, 1653, 561]}


def compare(p1, p2):
   """
   Check that both recodings correspond in pulse length to within
   TOLERANCE%.  If they do average the two recordings pulse lengths.

   Input

        M    S   M   S   M   S   M    S   M    S   M
   1: 9000 4500 600 560 600 560 600 1700 600 1700 600
   2: 9020 4570 590 550 590 550 590 1640 590 1640 590

   Output

   A: 9010 4535 595 555 595 555 595 1670 595 1670 595
   """
   if len(p1) != len(p2):
      return False

   for i in range(len(p1)):
      v = p1[i] / p2[i]
      if (v < TOLER_MIN) or (v > TOLER_MAX):
         return False

   for i in range(len(p1)):
       p1[i] = int(round((p1[i]+p2[i])/2.0))

   return True


def cbf(gpio, level, tick):

	global last_tick, in_code, code, fetching_code, pi


	if level != pigpio.TIMEOUT:

		edge = pigpio.tickDiff(last_tick, tick)
		last_tick = tick

		if fetching_code:

			if (edge > PRE_US) and (not in_code): # Start of a code.
				in_code = True
				pi.set_watchdog(GPIO_IR_BCM_PIN, POST_MS) # Start watchdog.

			elif (edge > POST_US) and in_code: # End of a code.
				in_code = False
				pi.set_watchdog(GPIO_IR_BCM_PIN, 0) # Cancel watchdog.
				end_of_code()

			elif in_code:
				code.append(edge)

	else:
		pi.set_watchdog(GPIO_IR_BCM_PIN, 0) # Cancel watchdog.
		if in_code:
			in_code = False
			end_of_code()



def normalise(c):
   """
   Typically a code will be made up of two or three distinct
   marks (carrier) and spaces (no carrier) of different lengths.

   Because of transmission and reception errors those pulses
   which should all be x micros long will have a variance around x.

   This function identifies the distinct pulses and takes the
   average of the lengths making up each distinct pulse.  Marks
   and spaces are processed separately.

   This makes the eventual generation of waves much more efficient.

   Input

     M    S   M   S   M   S   M    S   M    S   M
   9000 4500 600 540 620 560 590 1660 620 1690 615

   Distinct marks

   9000                average 9000
   600 620 590 620 615 average  609

   Distinct spaces

   4500                average 4500
   540 560             average  550
   1660 1690           average 1675

   Output

     M    S   M   S   M   S   M    S   M    S   M
   9000 4500 609 550 609 550 609 1675 609 1675 609
   """
   entries = len(c)
   p = [0]*entries # Set all entries not processed.
   for i in range(entries):
      if not p[i]: # Not processed?
         v = c[i]
         tot = v
         similar = 1.0

         # Find all pulses with similar lengths to the start pulse.
         for j in range(i+2, entries, 2):
            if not p[j]: # Unprocessed.
               if (c[j]*TOLER_MIN) < v < (c[j]*TOLER_MAX): # Similar.
                  tot = tot + c[j]
                  similar += 1.0

         # Calculate the average pulse length.
         newv = round(tot / similar, 2)
         c[i] = newv

         # Set all similar pulses to the average value.
         for j in range(i+2, entries, 2):
            if not p[j]: # Unprocessed.
               if (c[j]*TOLER_MIN) < v < (c[j]*TOLER_MAX): # Similar.
                  c[j] = newv
                  p[j] = 1



def end_of_code():
   global code, fetching_code
   if len(code) > SHORT:
      normalise(code)
      fetching_code = False
   else:
      code = []
      print "Short code, probably a repeat, try again"


def ir_start_listener(callback):
	global pi, fetching_code, code, listening
	pi = pigpio.pi()
	pi.set_mode(GPIO_IR_BCM_PIN, pigpio.INPUT) # IR RX connected to this GPIO.

	pi.set_glitch_filter(GPIO_IR_BCM_PIN, GLITCH) # Ignore glitches.

	pi.set_mode(GPIO_IR_BCM_PIN, pigpio.INPUT) # IR RX connected to this GPIO.

	pi.set_glitch_filter(GPIO_IR_BCM_PIN, GLITCH) # Ignore glitches.

	cb = pi.callback(GPIO_IR_BCM_PIN, pigpio.EITHER_EDGE, cbf)

	listening = True

	thread.start_new_thread(ir_listen, (callback,))

def ir_listen(callback):
	global pi, fetching_code, code, listening


	while listening:
		fetching_code = True
		while fetching_code:
			 time.sleep(0.1)
		   # check code

		match_found = False

		for key, value in IR_WAVES.iteritems():
		   if compare(value, code):
			   match_found = True
			   callback(key)
		#reset code
		code = []

	pi.set_glitch_filter(GPIO_IR_BCM_PIN, 0) # Cancel glitch filter.
	pi.set_watchdog(GPIO_IR_BCM_PIN, 0) # Cancel watchdog.
	pi.stop()


def main():
	global pi, fetching_code, code
	pi = pigpio.pi()
	pi.set_mode(GPIO_IR_BCM_PIN, pigpio.INPUT) # IR RX connected to this GPIO.

	pi.set_glitch_filter(GPIO_IR_BCM_PIN, GLITCH) # Ignore glitches.

	pi.set_mode(GPIO_IR_BCM_PIN, pigpio.INPUT) # IR RX connected to this GPIO.

	pi.set_glitch_filter(GPIO_IR_BCM_PIN, GLITCH) # Ignore glitches.

	cb = pi.callback(GPIO_IR_BCM_PIN, pigpio.EITHER_EDGE, cbf)

	print "press ctrl+C to stop"

	while True:

		try:
			fetching_code = True
			while fetching_code:
				 time.sleep(0.1)
			   # check code

			match_found = False

			for key, value in IR_WAVES.iteritems():
			   if compare(value, code):
				   match_found = True
				   print "Match found for ", key
			#reset code
			code = []

		except KeyboardInterrupt:
			print "Stopping"
			pi.set_glitch_filter(GPIO_IR_BCM_PIN, 0) # Cancel glitch filter.
			pi.set_watchdog(GPIO_IR_BCM_PIN, 0) # Cancel watchdog.
			pi.stop()

			quit()

if __name__ == "__main__": main()

