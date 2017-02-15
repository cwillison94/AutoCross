# state broadcasted to other vehicles
class State:
	IDLE,				# not at an intersection				
	WAITING_AT_STOP,	# initial waitwaiting for initial handshake
	IN_TRANSIT,			# crossing intersection
	TRANSIT_COMPLETED   # through intersection
	= range(4)

# direction car is approaching intersection from
class Bearing:
	NORTH,
	SOUTH,
	EAST,
	WEST = range (4)