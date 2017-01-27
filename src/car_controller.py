import motor
import steering
import ir_receiver
import time



class CarController:

	def __init__(self):
		self.car_steering = steering.Steering()
		self.car_motor = motor.Motor()
		self.auto_running = False

	def ir_callback(self, key_pressed):
		print "Key Pressed", key_pressed

		if key_pressed == ir_receiver.KEY_2:
			self.car_motor.setPower(100)

		if key_pressed == ir_receiver.KEY_5:
			self.car_steering.reset()

		if key_pressed == ir_receiver.KEY_8:
			self.car_motor.setPower(0)

		if key_pressed == ir_receiver.KEY_4:
			self.car_steering.setPercentDirection(steering.DIRECTION_LEFT, 100)

		if key_pressed == ir_receiver.KEY_6:
			self.car_steering.setPercentDirection(steering.DIRECTION_RIGHT, 100)


	def start_ir_listener(self):

		ir_receiver.ir_start_listener(self.ir_callback)

	def start_auto(self):
		print "Auto Running Ctrl+C to stop"
		self.auto_running = True
		while self.auto_running:

			try:
				time.sleep(0.05)
			except KeyboardInterrupt:
				self.auto_running = False


if __name__ == "__main__":
	car = CarController()
	car.start_ir_listener()
	car.start_auto()

