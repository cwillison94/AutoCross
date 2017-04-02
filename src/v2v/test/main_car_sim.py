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

def mycallback(data):
	print(str(data))

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


def main():

	debug = True
	v2v = V2VModule(mycallback, debug)
	v2v.start()
	direction = NORTH

	print('listening')


	while True:
		time.sleep(0.1)


if __name__ == "__main__":
	#curses.wrapper(main)
	main()