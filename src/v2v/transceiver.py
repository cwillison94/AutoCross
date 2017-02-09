# hardware module

import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import time
import spidev

class Transceiver:

	def __init__(self):

		# boilerplate to initialize radio

		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)

		# NRF24L01 has 6 pipes
		# all of our vehices read and write to only one pipe
		pipes = [[0xe7, 0xe7, 0xe7, 0xe7, 0xe7], [0xc2, 0xc2, 0xc2, 0xc2, 0xc2]]

		self.radio = NRF24(GPIO, spidev.SpiDev())
		self.radio.begin(0, 17)
		self.radio.setPayloadSize(32)

		# We can use any channel, but all cars must use the same one
		self.radio.setChannel(0x60) 

		self.radio.setDataRate(NRF24.BR_2MBPS)
		self.radio.setPALevel(NRF24.PA_MIN)

		# disable automatic acknowledgment
		self.radio.setAutoAck(False) 

		# allow variable message size
		# may consider using static payload size once message protocol is established
		self.radio.enableDynamicPayloads()

		# read and write on same pipe for everyone
		self.radio.openWritingPipe(pipes[1])
		self.radio.openReadingPipe(0, pipes[1])

		#self.radio.printDetails()
		time.sleep(0.1)
		self.radio.flush_tx()
		self.radio.flush_rx()
		time.sleep(0.1)
		print('radio initialized')


	# send message in form of character array
	def transmit(self, message):
		char_arr = list(message)
		self.radio.write(char_arr)


	def receive(self):
		# chip can only be in transmit or receive mode at a time
	    self.radio.startListening()  # switch to receive mode

	    # number of tries and sleep interval may have to be tweaked
	    # this gives good results (minimal packet loss) for 3 cars
	    tries = 5
	    while(tries > 0):
	        if self.radio.available(0):
	            receivedMessage = []
	            self.radio.read(receivedMessage, self.radio.getDynamicPayloadSize())
	            return receivedMessage

	        tries = tries - 1
	        time.sleep(0.01)

	    self.radio.stopListening() # disable receive mode
	    return False

