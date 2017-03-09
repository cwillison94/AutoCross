import os, sys
sys.path.insert(0, "../")
from v2v_module import *


"""
Simulate a car state for testing v2v behavior

E.g.,

send stopped signal to V2V Module

wait for all clear signal from module

send in_transit signal
"""

def send_stopped_signal():


def send_transit_signal():


def get_ready_signal():
	return v2v_module.getTransitPermission()




if __name__ == "__main__":

	v2v_module = V2VModule()

	# car state
	stopped = False
	in_transit = False

	# set to True when V2V module gives all-clear to enter intersection
	ready = False

	v2v_module.start()
	try:
		while True:
			time.sleep(1)




	except KeyboardInterrupt:
		console.log("Keyboard interrupt. Exiting.")
		sys.exit()
