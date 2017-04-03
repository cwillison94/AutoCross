import sys
sys.path.insert(0, "../")

from nrf24_transceiver import *


def main():

	transceiver = Transceiver(False)
	transceiver.radio.flush_rx()
	last_message = None

	total_message_count = 0

	print('listening for messages')
	while(1):
		msg = transceiver.receive()
		if msg:
			total_message_count+=1
			if msg != last_message:
				string = ''.join(chr(e) for e in msg)
				last_message = msg
				print("%s (%d total messages)" % (string, total_message_count) )


if __name__ == "__main__":
	main()