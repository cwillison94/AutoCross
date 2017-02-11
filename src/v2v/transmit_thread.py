import time
from threading import Thread
from nrf24_transceiver import Transceiver


class TransmitThread(Thread):

    def __init__(self, message):
        super(TransmitThread, self).__init__()
        self.message = message
        self.transceiver = Transceiver(True)

    def set_message(message):
        self.message = message

    def run(self):
        while True:
            self.transceiver.transmit( self.message )






