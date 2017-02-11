import sys
sys.path.insert(0, "../")
from receive_thread import *
from transmit_thread import *

def on_message_received(message):
    print('message received: ', message)
    time.sleep(0.01)

if __name__ == "__main__":

    message = "Hello!"

    receiver = ReceiveThread(on_message_received)
    transmitter = TransmitThread(message, None)

    transmitter.start()
    receiver.start()
