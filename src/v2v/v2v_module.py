import time
import os,binascii 
from uuid import getnode as get_mac

from threading import Thread, Condition

from receive_thread import *
from transmit_thread import *

from constants import *

import re


# use last 4 hex digits of MAC address as unique car id
DEVICE_ID = hex(get_mac())[-5:-1] 
MESSAGE_FORMAT = re.compile('^([a-fA-F0-9]{4}):([0-3]{1}):([0-3]{1}):([0-9]{3})$')

last_message = ""

class V2VModule(Thread):


	def __init__(self, ready_callback, debug_mode=True):
		super(V2VModule, self).__init__()

		self.ready_callback = ready_callback
		self.debug_mode = debug_mode

		# transmitter and receiver threads
		#receiver callback : _on_message_received
		self.condition = Condition() # thread control variable (ensures alternating threads)
		self.receiver = ReceiveThread(self.condition, self._on_message_received)
		self.transmitter = TransmitThread(None, self.condition, None)

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
		self._set_transmitter_state(IDLE)
		self.transmitter.start()
		self.receiver.start()
		self.debug_print("V2V Module started.")
		while self.running:


			# do nothing until we get to an intersection
			# during this time vehicle queue is constantly being updated as _on_message_received is triggered
			time.sleep(0.05)

			# transmit STOPPED signal and wait for our turn to transit
			if self.state == STOPPED:
				self.debug_print("Stopped. Waiting for turn to transit.")
				self._set_transmitter_state(STOPPED)

				time.sleep(MINIMUM_WAIT_PERIOD) # minimum time we have to wait
				self.debug_print("Minimum wait time (MINIMUM_WAIT_PERIOD = %.1f) elapsed." % (MINIMUM_WAIT_PERIOD))

				# wait for our turn in queue to go, either when its our turn or if there is no one else
				while not ( self.ready or len(self.vehicles.keys()) == 0 ):
					time.sleep(0.1)

				self.debug_print("ready flag set. broadcasting IN_TRANSIT signal.")
				self.ready_callback()
				# our turn to go. set TRANSIT signal
				self._set_transmitter_state(IN_TRANSIT)

				# wait for car controller to receive ready signal and drive through intersection
				while self.state != IN_TRANSIT:
					time.sleep(0.1)
				self.debug_print("Entered intersection...")

				# car is moving. wait until we have left intersection
				while self.state != CLEARED:
					time.sleep(0.1)
				self.debug_print("Cleared intersection. Broadcasting CLEARED signal.")
				# through intersection. broadcast CLEARED signal for 3 seconds.
				self._set_transmitter_state(CLEARED)
				time.sleep(3)

				self.debug_print("Returning to idle state.")
				# go back to idle state until we arrive at intersection again
				self._set_idle()


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

		#self.debug_print("new vehicle data received (%s): %s " % ( str(vehicle_id), str(vehicle_data) ) )
		#self.debug_print("# of vehicles in vehicle data ")
		#self.debug_print("_update_vehicles called. # of vehicles recorded: %d" % (len(self.vehicles.keys())) )
		self.ready = False

		state = vehicle_data["state"]


		# vehicle was already recorded. update its state
		if vehicle_id in self.vehicles.keys():
			if state == CLEARED: #remove the vehicle from our record
				self.debug_print("cleared state read for %s. removing." % (vehicle_id))
				del self.vehicles[vehicle_id]
			elif state != self.vehicles[vehicle_id]["state"]:
				self.debug_print("updating state for %s" % (vehicle_id))
				self.vehicles[vehicle_id]["state"] = vehicle_data["state"]


		# new vehicle. record time of arrival
		elif state != CLEARED:
			self.debug_print("new vehicle arrived and stopped: %s" % (vehicle_id))
			self.vehicles[vehicle_id] = vehicle_data
			self.vehicles[vehicle_id]["timestamp"] = time.time()

		if self.arrival_time is not None:

			# TODO move to seperate function (like in MID, MIS)
			#get the earliest arrival time
			earliest_arrival = min([self.vehicles[v]["timestamp"] for v in self.vehicles]) 
			#earliest_arrival = min([v["timestamp"] for v in self.vehicles]) 

			# check if there is already a car in the intersection
			transit_vehicle = None
			for v in self.vehicles:
				if self.vehicles[v]["state"] == IN_TRANSIT:
					transit_vehicle = self.vehicles[v]

			if transit_vehicle:
				self.debug_print("Another vehicle currently in transit.")
				if (transit_vehicle["direction"] in [NORTH,SOUTH] and self.direction in [NORTH, SOUTH]) \
						or (transit_vehicle["direction"] in [EAST,WEST] and self.direction in [EAST, WEST]):
					self.debug_print("In-transit vehicle in parallel lane. ready.")
					self.ready =  True

			# check for arrival time precedence
			elif (self.arrival_time + BUFFER_PERIOD) < earliest_arrival :
				self.debug_print("arrival time + Buffer period is less than next-earliest arrival. going.")
				self.ready = True

			# check for lane precedence (NORTH/SOUTH gets to go first)
			elif abs(self.arrival_time - earliest_arrival) < BUFFER_PERIOD:
				self.debug_print("ambiguous arrival times")
				if self.direction in [NORTH,SOUTH]:
					self.debug_print("we are in priority lane. going.")
					self.ready = True



	# Receiver callback, triggered when a NEW message is received
	# update our vehicle data
	def _on_message_received(self, msg):
		string = ''.join(chr(e) for e in msg)
		params = self._parse_message(str(string))

		if params:
			#self.debug_print("new message received: %s" % (string))
			vehicle_id = str(params[0])
			vehicle_data = {
					"state": int(params[1]),
					"direction": int(params[2]),
					"speed": int(params[3])
			}

			self._update_vehicles(vehicle_id, vehicle_data)

	# validate format, extract vehicle data and remove self-messages
	def _parse_message(self, msg):
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
			message = str(DEVICE_ID) + ":" + str(state) + ":" + str(self.direction) + ":" + str(DUMMY_SPEED)
			self.transmitter.set_message(message)
			self.transmitter.set_enabled(True)


	# reset after leaving intersection
	def _set_idle(self):
		self.direction = None
		self.arrival_time = None
		self.ready = False
		self.state = IDLE
		self._set_transmitter_state(IDLE)

	"""
	public methods called by main vehicle controller 
	"""

	# check if we are allowed to drive into intersection
	def get_transit_permission(self):
		return self.ready

	# 1 arrive at intersection
	def set_stopped(self, direction):
		self.debug_print('Stopped state received from main controller')
		self.direction = direction
		self.arrival_time = time.time()
		self.state = STOPPED

	# 2 driving through intersection after receiving permission
	def set_in_transit(self):
		self.debug_print('In transit state received from main controller')
		self.state = IN_TRANSIT

	# 3 left intersection
	def set_cleared(self):
		self.debug_print('Cleared state received from main controller')
		self.state = CLEARED

	# debugging
	def debug_print(self, string):
		global last_message
		if self.debug_mode and string != last_message:
			last_message = string
			print( str(DEVICE_ID) + ": " + string)

