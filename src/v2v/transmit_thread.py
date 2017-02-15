import time
from threading import Thread
from nrf24_transceiver import Transceiver

# callback when a message is transmitted
class TransmitThread(Thread):

    def __init__(self, message, callback=None):
        super(TransmitThread, self).__init__()
        self.transceiver = Transceiver(True)
        self.message = message
        self.callback = callback
	self.running = True

    def get_message(self):
    	return self.message

    def set_message(self, message):
        self.message = message

    def run(self):
        while self.running:
            self.transceiver.transmit( self.message )
            self.callback and self.callback(self.message)
	print('transmit thread done')
