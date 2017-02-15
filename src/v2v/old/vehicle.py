from enums import *

class VehicleData:
	def __init__(self, vehicle_id, timestamp, bearing, status):
		self.vid = vehicle_id
		self.timestamp = timestamp
		self.bearing = bearing
		self.status = status # 0: waiting, 1: in transit, 2: completed