import time
import os,binascii 

from threading import Thread

from receive_thread import *
from transmit_thread import *

from constants import *

"""
V2V Application Layer Module

"Brain" of the V2V module. Sends and receives messages with other vehicles, and communicates with main car controller. 

Main car controller SENDS information about the car state (Stopped, in intersection, etc).
Main car controller RECEIVES go-ahead signal to proceed through intersection.

Main controller initializes V2VModule and polls for variable changes as needed. 


BEHAVIOR
see documentation (requirements, V&V) for details.

1. A vehicle already in the intersection takes precedence over all other vehicles.

2. All vehicles must wait for a minimum time equal to BUFFER_PERIOD before entering
intersection, ​ in all circumstances.

3. Vehicles which arrive ​ first take precedence, except in the following conditions

4. If a vehicle observes an interarrival time less than BUFFER_PERIOD, it must defer to
lane direction precedence* regardless of which vehicle was observed to arrive first.

5. If a vehicle arrives (Car A) and observes another vehicle (Car B) in the o
pposing parallel lane, Car A can enter the intersection as soon as its BUFFER_PERIOD elapses, as long
as it does not receive a “transit completed” message from CAR B in that time.

*Vehicles in the NORTH/SOUTH lane take precedence over vehicles in the EAST/WEST lane.


"""

class V2VModule(Thread):

	def __init__(self):
		super(V2VModule, self).__init__()

		# transmitter and receiver threads
		#receiver callback : _on_message_received
		self.condition = Condition() # thread control variable (ensures alternating threads)
		self.receiver = ReceiveThread(condition, _on_message_received)
		self.transmitter = TransmitThread(None, condition, None)


		# car state, set externally ONLY by main car controller
		self.stopped_at_intersection = False
		self.in_transit = False

		# when we are ready to go through intersection
		self.ready = False

		# queue all vehicles at intersection, according to behavior rules specified above
		self.vehicle_queue = [] # TODO 

		self.running = True


	def run(self):
		_set_transmitter_state(IDLE)
		transmitter.start()
		receiver.start()

		while self.running:

			# do nothing until we get to an intersection
			# during this time vehicle queue is constantly being updated as _on_message_received is triggered
			time.sleep(0.1)

			# transmit STOPPED signal and wait for our turn to transit
			if self.stopped_at_intersection:
				_set_transmitter_state(STOPPED)
				time.sleep(BUFFER_PERIOD) # minimum time we have to wait
				
				# wait for our turn in queue to go
				while not self.ready:
					time.sleep(0.1)

				# our turn to go. set TRANSIT signal
				_set_transmitter_state(IN_TRANSIT)

				# wait for car controller to receive ready signal and drive through intersection
				while not self.in_transit:
					time.sleep(0.1)

				# car is moving. wait until we have left intersection
				while self.in_transit:
					time.sleep(0.1)

				# through intersection. broadcast CLEARED signal for 3 seconds.
				_set_transmitter_state(CLEARED)
				time.sleep(3)

				# go back to idle state until we arrive at intersection again
				_set_transmitter_state(IDLE)
				self.ready = False


""" 
PRIVATE METHODS
"""
	# TODO
	# bulk of our calculations go here
	# determine vehicle transit order based on arrival time and lane precedence
	# this must be constantly updated - position can change!
	def _update_queue(arrive_time, vehicle_data):

		if first_in_queue :
			# if we are first in queue, and BUFFER_PERIOD has elapsed, we are ready to go
			self.ready = True
		else:
			self.ready = False


	# Receiver callback, triggered when a NEW message is received
	# update our queue with the new vehicle data
	def _on_message_received(self, msg):
		arrive_time = time.time()
		vehicle_data = _decode_vehicle_data(msg)
		_update_queue(arrive_time, vehicle_data)


	def _decode_vehicle_data(self, msg):
		string = ''.join(chr(e) for e in msg)
		return string

	# set the message being broadcasted continuously by transmitter
	def _set_transmitter_state(self, state):
		if state == IDLE:
			self.transmitter.set_enabled(False)

		else:
			self.transmitter.set_enabled(True)
			if state == STOPPED:

			elif state == IN_TRANSIT:

			elif state == CLEARED:
			else:
				print('transmitter state not recognized')

"""
PUBLIC METHODS
called by main controller
"""

	# check if we are allowed to drive into intersection
	def getTransitPermission(self):
		return self.ready

	# 1 arrive at intersection
	def setStopped(self):
		self.stopped_at_intersection = True
		self.in_transit  = False

	# 2 driving through intersection after receiving permission
	def setInTransit(self):
		self.in_transit = True
		self.stopped_at_intersection = False

	# 3 left intersection
	def setCleared(self):
		self.in_transit = False