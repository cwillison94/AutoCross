import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import time
import spidev

GPIO.setmode(GPIO.BCM)

pipes = [[0xe7, 0xe7, 0xe7, 0xe7, 0xe7], [0xc2, 0xc2, 0xc2, 0xc2, 0xc2]]

radio = NRF24(GPIO, spidev.SpiDev())
radio.begin(0, 17)
radio.setPayloadSize(32)
radio.setChannel(0x60)

radio.setDataRate(NRF24.BR_2MBPS)
radio.setPALevel(NRF24.PA_MIN)
#radio.setAutoAck(False)
radio.enableDynamicPayloads()
#radio.enableAckPayload()

# read pipe 1
# write pipe 0. in transmit mode, pipe 0 is automatically switched to a write pipe!!!

radio.openWritingPipe(pipes[1])
radio.openReadingPipe(0, pipes[1])

radio.printDetails()


device_id = "PI_2"


def transmit():
    message = list("Hello from" + device_id)
    radio.write(message)

    # Check if it returned ackPL
    #if radio.isAckPayloadAvailable():
    #    returnedPL = []
    #    radio.read(returnedPL, radio.getDynamicPayloadSize())
    #    print("Payload acknowledged: {}".format(returnedPL))


def receive():
    radio.startListening()
    #ackPL = [1]
    tries = 5
    while(tries > 0):
        if radio.available(0):
            receivedMessage = []
            radio.read(receivedMessage, radio.getDynamicPayloadSize())
            print("Received: {}".format(receivedMessage))
            string = ""
            for n in receivedMessage:
                # Decode into standard unicode set
                if (n >= 32 and n <= 126):
                    string += chr(n)
            print("recieved: " + string)
            #radio.writeAckPayload(1, ackPL, len(ackPL))
            #print("Loaded payload reply of {}".format(ackPL))

            break
        tries = tries - 1
        time.sleep(0.01)
        #print('durr')

            #timesleep(1)
    radio.stopListening()



# radio.startListening()
# message = list(input("Enter a message to send: "))
while(1):
  transmit()
#    radio.flush_tx()
#    radio.flush_rx()
  receive()
#    radio.flush_tx()
#    radio.flush_rx()
    #time.sleep(1)
