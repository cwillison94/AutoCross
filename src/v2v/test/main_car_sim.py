import os, sys
sys.path.insert(0, "../")
from v2v_module import *
from constants import *
import curses
import time


# simulated time it takes to cross intersection
TRANSIT_DURATION = 3

# defined in constants.py
DIRECTIONS = {
	"KEY_UP": NORTH,
	"KEY_DOWN" : SOUTH,
	"KEY_RIGHT" : EAST,
	"KEY_LEFT"	: WEST
}

DIRECTION_STRINGS = {
	NORTH: "NORTH",
	SOUTH : "SOUTH",
	EAST : "EAST",
	WEST	: "WEST"
}

def send_stopped_signal(module, direction):
	module.set_stopped(direction)


def send_transit_signal(module):
	module.set_in_transit()

def send_cleared_signal(module):
	module.set_cleared()


def get_ready_signal(module):
	return module.get_transit_permission()

def get_key_press(win):


	direction = None

	while 1:    
	    try:                 
			key = win.getkey() 
			if str(key) in DIRECTIONS.keys():
				direction = DIRECTIONS[str(key)]
				win.addstr("Current approach direction: %s\n" % DIRECTION_STRINGS[direction])
			elif key == os.linesep:
				return direction
			elif str(key) == 'q':
				sys.exit(0)

	    except Exception as e:
	       pass  


def main(win):

	debug = True
	v2v = V2VModule(debug)
	direction = NORTH

	#win.nodelay(True)

	key=""
	win.clear()                
	win.addstr("Use the arrow Keys to set approach direction.\n")
	win.addstr("Press Enter to send STOPPED signal.\n")
	win.addstr("Press q to quit.\n")


	while True:
		win.addstr("Current approach direction: %s\n" % DIRECTION_STRINGS[direction])

		# wait for enter key to send STOP signal using current direction or new one
		direction = get_key_press(win) or direction
		send_stopped_signal(v2v,direction)
		win.addstr("Sent STOPPED signal from %s\n" % (DIRECTION_STRINGS[direction]))
		win.addstr("Waiting for ready signal from V2V...\n")
		ready = get_ready_signal(v2v)
		while not ready:
			ready = get_ready_signal(v2v)
			time.sleep(0.1)
		win.addstr("Received ready signal. Entering intersection.\n")
		send_transit_signal(v2v)
		time.sleep(TRANSIT_DURATION)
		win.addstr("Cleared intersection.\n")
		send_cleared_signal(v2v)


if __name__ == "__main__":
	curses.wrapper(main)
