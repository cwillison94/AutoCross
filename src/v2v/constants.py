	BUFFER_PERIOD = 0.5 
	# minimum wait time (must be at least 2x max transmission delay)
	# actual time will depend on processing delays, start with a conservative value

	DEVICE_ID = binascii.b2a_hex(os.urandom(3)) # 6 digit unique id

	NORTH = 0
	SOUTH = 1
	EAST = 2
	WEST = 3

	IDLE = 0
	STOPPED = 1
	IN_TRANSIT = 2
	CLEARED = 3