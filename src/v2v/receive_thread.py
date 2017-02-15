import time
from threading import Thread
from nrf24_transceiver import Transceiver

#callback when a message is received
class ReceiveThread(Thread):

	def __init__(self, callback):
		super(ReceiveThread, self).__init__()
		self.transceiver = Transceiver(False)
		self.callback = callback
		self.running = True

	def run(self):
		while self.running:
			msg = self.transceiver.receive()
			if msg:
				self.callback(msg)
		print('receive thread done')


