import time
from threading import Thread
from nrf24_transceiver import Transceiver

# callback when a message is transmitted
class TransmitThread(Thread):

    def __init__(self, message, callback):
        super(TransmitThread, self).__init__()
        self.transceiver = Transceiver(True)
        self.message = message
        self.callback = callback

    def get_message():
    	return self.message

    def set_message(message):
        self.message = message

    def run(self):
        while True:
            self.transceiver.transmit( self.message )
            self.callback(self)






