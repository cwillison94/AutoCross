# intermediate message module

from enums import *


def encode_encode_vehicle_data(vehicle):

	vid = vehicle.vid
	timestamp = vehicle.timestamp
	bearing = vehicle.bearing
	status = vehicle.status

	encoded_message = list(vid + timestamp + bearing + status)
	print("vehicle data encoded: " + encoded_message)
	return encoded_message 

def decode_vehicle_data(raw_message):

	if _validate_message(message):
		vid = raw_message[0..8]
		timestamp = int(raw_message[8..16])
		bearing = Bearing.NORTH #todo
		status = int(raw_message[16])

		return VehicleData(vid, timestamp, bearing, status)
	else:
		return False


def _validate_message(message):
	#more needed
	return isinstance(message, lst) and len(message) == 16








