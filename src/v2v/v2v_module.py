import time
import os,binascii 

from threading import Thread

from receive_thread import *
from transmit_thread import *

from constants import *

import re

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
intersection, in all circumstances.

3. Vehicles which arrive ​ first take precedence, except in the following conditions

4. If a vehicle observes an interarrival time less than BUFFER_PERIOD, it must defer to
lane direction precedence* regardless of which vehicle was observed to arrive first.

5. If a vehicle arrives (Car A) and observes another vehicle (Car B) in the o
pposing parallel lane, Car A can enter the intersection as soon as its BUFFER_PERIOD elapses, as long
as it does not receive a "transit completed" message from CAR B in that time.

*Vehicles in the NORTH/SOUTH lane take precedence over vehicles in the EAST/WEST lane.


"""

class V2VModule(Thread):

	# 4 digit unique id
	DEVICE_ID = binascii.b2a_hex(os.urandom(2)) 
	MESSAGE_FORMAT = re.compile('^([a-fA-F0-9]{4}):([0-3]{1}):([0-3]{1}):([0-9]{3})$')


	def __init__(self, debug_mode=False):
		super(V2VModule, self).__init__()

		self.debug_mode = debug_mode

		# transmitter and receiver threads
		#receiver callback : _on_message_received
		self.condition = Condition() # thread control variable (ensures alternating threads)
		self.receiver = ReceiveThread(condition, _on_message_received)
		self.transmitter = TransmitThread(None, condition, None)


		# car state, set externally ONLY by main car controller
		# IDLE, STOPPED, IN_TRANSIT, CLEARED
		self.state = IDLE

		# when we are ready to go through intersection
		self.ready = False

		# data for determining queue/departure order
		self.vehicles = {} 
		self.arrival_time = None
		self.direction = None

		# thread run condition
		self.running = True


	def run(self):
		_set_transmitter_state(IDLE)
		transmitter.start()
		receiver.start()
		debug_print("V2V Module started.")
		while self.running:


			# do nothing until we get to an intersection
			# during this time vehicle queue is constantly being updated as _on_message_received is triggered
			time.sleep(0.1)

			# transmit STOPPED signal and wait for our turn to transit
			if self.state == STOPPED:
				debug_print("Stopped. Waiting for turn to transit.")
				_set_transmitter_state(STOPPED)
				time.sleep(BUFFER_PERIOD) # minimum time we have to wait
				debug_print("Minimum wait time (BUFFER_PERIOD = %.1f) elapsed." % (BUFFER_PERIOD))

				# wait for our turn in queue to go, either when its our turn or if there is no one else
				while not ( self.ready or len(self.vehicles.keys()) == 0 ):
					time.sleep(0.1)
				debug_print("ready flag set. broadcasting IN_TRANSIT signal.")
				# our turn to go. set TRANSIT signal
				_set_transmitter_state(IN_TRANSIT)

				# wait for car controller to receive ready signal and drive through intersection
				while self.state != IN_TRANSIT:
					time.sleep(0.1)
				debug_print("Entered intersection...")

				# car is moving. wait until we have left intersection
				while self.state != CLEARED:
					time.sleep(0.1)
				debug_print("Cleared intersection. Broadcasting CLEARED signal.")
				# through intersection. broadcast CLEARED signal for 3 seconds.
				_set_transmitter_state(CLEARED)
				time.sleep(3)

				debug_print("Returning to idle state.")
				# go back to idle state until we arrive at intersection again
				_set_idle()


	# bulk of our calculations go here
	# determine vehicle transit order based on arrival time and lane precedence
	#
	# example of data structure :
	#
	# self.vehicles = {
	# 	"id_x" : {
	# 		state: STOPPED,
	# 		direction: SOUTH,
	# 		speed: 555
	# 	},
	# 	...
	# }
	def _update_vehicles(self, vehicle_id, vehicle_data):

		self.ready = False

		state = vehicle_data["state"]


		# vehicle was already recorded. update its state
		if vehicle_id in self.vehicles.keys():
			if state == CLEARED: #remove the vehicle from our record
				del self.vehicles[vehicle_id]
			else:
				self.vehicles[vehicle_id]["state"] = vehicle_data["state"]

		# new vehicle. record time of arrival
		elif state != CLEARED:
			self.vehicles[vehicle_id] = vehicle_data
			self.vehicles[vehicle_id]["timestamp"] = time.time()

		#get the earliest arrival time
		earliest_arrival = min([v["timestamp"] for v in self.vehicles]) 


		# check if there is already a car in the intersection
		transit_vehicle = None
		for v in self.vehicles:
			if v["state"] == IN_TRANSIT:
				transit_vehicle = v
		if transit_vehicle:
			if (transit_vehicle["direction"] in [NORTH,SOUTH] and self.direction in [NORTH, SOUTH]) \
					or (transit_vehicle["direction"] in [EAST,WEST] and self.direction in [EAST, WEST]):

				self.ready =  True

		# check for arrival time precedence
		elif (self.arrival_time + BUFFER_PERIOD) < earliest_arrival :
			self.ready = True

		# check for lane precedence (NORTH/SOUTH gets to go first)
		elif abs(self.arrival_time - earliest_arrival) < BUFFER_PERIOD:
			if self.direction in [NORTH,SOUTH]:
				self.ready = True



	# Receiver callback, triggered when a NEW message is received
	# update our vehicle data
	def _on_message_received(self, msg):

		params = _parse_message(msg)
		if params:
			vehicle_id = str(params[0])
			vehicle_data = {
					"state": int(params[1]),
					"direction": int(params[2]),
					"speed": int(params[3])
			}

			_update_vehicles(vehicle_id, vehicle_data)

	# validate format, extract vehicle data and remove self-messages
	def _parse_message(msg):
		# regex match
		match = MESSAGE_FORMAT.match(msg)
		if match and str(match.groups()[0]) != DEVICE_ID:
			return match.groups()
		else:
			return False


	# set the message being broadcasted continuously by transmitter
	def _set_transmitter_state(self, state):
		if state == IDLE:
			self.transmitter.set_enabled(False)
		else:
			# e.g:  ab12:1:3:777
			message = str(DEVICE_ID) + ":" + str(self.state) + str(self.direction) + str(DUMMY_SPEED)
			self.transmitter.set_message(message)
			self.transmitter.set_enabled(True)


	# reset after leaving intersection
	def _set_idle(self):
		self.direction = None
		self.arrival_time = None
		self.ready = False
		self.state = IDLE
		_set_transmitter_state(IDLE)


	"""
	public methods called by main vehicle controller 
	"""

	# check if we are allowed to drive into intersection
	def get_transit_permission(self):
		return self.ready

	# 1 arrive at intersection
	def set_stopped(self, direction):
		debug_print('Stopped state received from main controller')
		self.direction = direction
		self.arrival_time = time.time()
		self.state = STOPPED

	# 2 driving through intersection after receiving permission
	def set_in_transit(self):
		debug_print('In transit state received from main controller')
		self.state = IN_TRANSIT

	# 3 left intersection
	def set_cleared(self):
		debug_print('Cleared state received from main controller')
		self.state = CLEARED

	# debugging
	def debug_print(self, string):
		if self.debug_mode:
			print( str(DEVICE_ID) + ": " + string)

