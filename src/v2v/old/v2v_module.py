# toplevel logic module. Creates messages with signal module and 
# sends them to hardware module for transmission

from enums import *
from transceiver import Transceiver
from signals import *
from vehicle import VehicleData
import sys
import time


class V2V_module:

	def __init__(self, vehicle_id):
		self.bearing = Bearing.NORTH  # EAST, SOUTH, WEST
		
		self.vehicle_id = vehicle_id

		self.state = State.IDLE
		self.transceiver = Transceiver()
		self.cleared_for_transit = False

		self.vehicle_queue = []

		self.message = ""

		self.timestamp = sys.maxint


# methods for setting module state from main controller
# should NOT be called from inside module

	def set_bearing(bearing):
		self.bearing = bearing

	def set_stopped():
		self.state = State.WAITING_AT_STOP

	def set_transiting():
		self.state = State.IN_TRANSIT

	def set_transit_completed():
		self.state = State.TRANSIT_COMPLETED

	def set_idle():
		self.state = State.IDLE

# get state from main module
# e.g., to check when vehicle is OKd to transit

	def is_cleared_for_transit():
		return self.cleared_for_transit

# private methods

	def _current_time_in_millis(): 
		return int(round(time.time() * 1000))

	def _set_timestamp():
		self.timestamp = _current_time_in_millis()
	# set the message to be broadcasted
	# purely a function of module state
	# store it here so we only have to call encode_message when state changes
	def _set_encoded_message():
		self.message = encode_vehicle_data()

	def _updateQueue(vehicle_data):
		self.vehicle_queue.append(vehicle_data)
		# sort by timestamp; then lowest vid in case of conflict
		self.vehicle_queue.sort(key=lambda x: (x.timestamp, x.vid))

	# broadcasts and recieves according to the module's state
	# update car queue and determines when cleared for transit
	def _trancieve():
		self.transceiver.transmit(self.message) #ignore acknowledgments for now
		raw_message = self.transceiver.receive()
		vehicle_data = decode_vehicle_data(raw_message)
		
		# add vehicle to queue if data is valid
		if vehicle_data:
			_updateQueue(vehicle_data)

# one main method that is called repeatedly in main control loop**
# **on a thread TBI
	def run():
		_trancieve()




# NOTE: synchronizing the queue is complicated. 
# There are potentially two sources of vehicle data:
# Received messages,and acknowledgments from cars who have received
# your broadcast.

# For now, acknowledgments are disabled, and queue is only based on
# received messages, in order to avoid complexity of syncing 
# acknowledgments and receivals, and to minimize traffic/clutter on the
# common wireless channel/pipe.

# It may be necessary to impliment some sort of synchronization 
# in the future if communication is too unreliable.