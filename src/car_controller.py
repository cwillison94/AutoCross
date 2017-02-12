import car_motor
import steering
import ir_receiver
import time

DEFAULT_POWER_INCREMENT = 10
DEFAULT_STEERING_INCREMENT = 25

class CarController:

	def __init__(self, power_increment = DEFAULT_POWER_INCREMENT, steering_increment = DEFAULT_STEERING_INCREMENT):
		self.car_steering = steering.Steering()
		self.car_motor = car_motor.CarMotor()
		self.auto_running = False
		self.power = 0
		self.power_increment = power_increment
		self.steering = 0
		self.steering_increment = steering_increment

	def ir_callback(self, key_pressed):
		print "Key Pressed", key_pressed

		if key_pressed == ir_receiver.KEY_100_PLUS:
			if self.power + self.power_increment <= 100:
				self.power += self.power_increment
				self.car_motor.set_percent_power(self.power)
		if key_pressed == ir_receiver.KEY_2:
			if self.power >= self.power_increment:
				self.power -= self.power_increment
				self.car_motor.set_percent_power(self.power)

		if key_pressed == ir_receiver.KEY_5:
			self.steering = 0
			self.car_steering.reset()

		if key_pressed == ir_receiver.KEY_8:
			self.power = 0
			self.car_motor.set_percent_power(0)

		if key_pressed == ir_receiver.KEY_4:
			if self.steering - self.steering_increment >= -100:
				self.steering -= self.steering_increment

			self.car_steering.set_percent_direction(self.steering)

		if key_pressed == ir_receiver.KEY_6:
			if self.steering + self.steering_increment <= 100:
				self.steering += self.steering_increment

			self.car_steering.set_percent_direction(self.steering)

		if key_pressed == ir_receiver.KEY_CH:
			print "Reset"
			self.power = 0
			self.steering = 0
			self.car_motor.set_percent_power(0)
			self.car_steering.reset()


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

