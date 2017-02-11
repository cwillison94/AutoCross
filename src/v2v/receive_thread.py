import time
from threading import Thread
from nrf24_transceiver import Transceiver

class ReceiveThread(Thread):

	def __init__(self, callback):
		super(ReceiveThread, self).__init__()
		self.transceiver = Transceiver(False)
		self.callback = callback


	def run(self):
		while True:
			msg = self.transceiver.receive()
			if msg:
				self.callback(msg)



