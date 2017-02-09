import time
from Signals import *
import transceiver

def current_time_in_millis(): 
	return int(round(time.time() * 1000))

#time to listen for other cars, in msec
WAIT_BUFFER = 3000  

def wait_for_new_handshake():
	while(True):
		idle = transceiver.listen_for_idle() 
		if idle: # ready to start a handshake
			break
		sleep(0.5)


broadcasted_to = []
received_from = []

queue = []



# send out a handshake signal
# wait until acknowledgement is receieved from every car
# i.e., acknowledge every car that transmits handshake signal, and recieve acknowledgment in turn
def start_new_handshake():


	start_time = current_time_in_millis()
	end_time = start_time + WAIT_BUFFER

	while (True):
		time = current_time_in_millis()
		if time > end_time:
			break

		# continuously send and recieve handshakes until time elapses
		# update end time if a new one if recieved 
		receiver = transceiver.broadcast_handshake()
		sender = transceiver.receive_handshake()

		if (receiver not in broadcasted_to):
			broadcasted_to.push(receiver)
			end_time = time + WAIT_BUFFER

		if (sender not in received_from):
			received_from.push(sender)
			end_time = time + WAIT_BUFFER
	
		sleep(0.1)
	# handshake is complete
	if (broadcasted_to == received_from):
		# for car in broadcasted_to:
		# 	if car.timestamp < start_time
		print("handshake complete")
	else:
		print("error.. not all members in handshake")




def wait_for_turn():
	while (True):
		if position > 0:
			req = transceiver.receive_and_acknowledge_transit_request()
			position = position - 1
			# if (req):
			# 	sleep(0.5) # give time for other car to start transit
			# 	transceiver.recieve_in_transit_signal()


def finish_transit():
	print("finished.")


# standalone module for testing broadcast, receieve, handshake 
# initial conditions: stopped at intersection, waiting for clearance
if __name__ == '__main__':

	# globals for communicating with main control module
	# will not be defined here; test purposes only

	# inform controller we are clear for transit
	TRANSIT_REQUEST_APPROVED = False

	# monitor state of car to broadcast/recieve
	STOPPED_AT_INTERSECTION = True
	TRANSITING_INTERSECTION = False

	init_transceiver()
	print('success')
	while(True):

		if (STOPPED_AT_INTERSECTION):

			# wait for any car already in transit to finish
			# also wait for any cars already in the process of handshaking
			# we do this by listening for any NOT_IDLE signals being transmitted
			wait_for_new_handshake()

			# establish contact with other cars waiting to transit
			# we have to restart the timer every time a new car enters the handshake
			# otherwise a car might arrive inside one car's waiting buffer, 
			# but outside another, and the handshake will be incomplete
			start_new_handshake()

			# all cars involved in transaction have been mutually identified along with their arrival times and serial #
			# now each car proceeds to request transit in order of arrival (or serial # in case of conflict)
			# wait until all handshaked cars with higher priority have requested and completed transit
			wait_for_turn()

			# send a request for transit out to all the handshaked cars
			if request_transit_approval():
				emit_signal(Signal_Codes.IN_TRANSIT)
				TRANSIT_REQUEST_APPROVED = True

				# car begins transit...
				# wait for controller to start moving car
				while (STOPPED_AT_INTERSECTION):
					sleep(0.1)
				# wait for car to finish transiting intersection
				while (TRANSITING_INTERSECTION):
					sleep(1)

				# signal other handshaked cars that transit is complete and next car can request transit
				finish_transit()
				break



			#error, did not get approval from everyone
			else:
				print("error receiving approval")

	print('exited')