
import time
import motor
import steering

#[help string, method name, command name, alias]
commands = [
	["test-cycle]\t\t-(tc)runs a test cycle on the car. forward + left/right","testCycle","test-cycle",'tc'], 
	["auto\t\t-(a)run the car in fully autonmous mode with everythin ON", "startAuto","auto", "a"], 
	["drive <motor_percent>\t\t-(d)set the motor to drive at a percent of 0 to 100","drive", "drive", "d"],
	["direction <1:L 2:R> <percent_of_direction>\t-set steering servo percentage left or right of max", "steerDirection", "direction"],
	["stop\t\t-stops everything","stopCar", "stop"],
	["exit\t\t-stops and exits", "stopProgram", "exit"],
	["help\t\t-brings up help", "help", "help", "h"]
]

carSteering = steering.Steering()
carMotor = motor.Motor()

def mainCommandInterface():

	printWelcome()
	printCommandList()

def printWelcome():
	print "Welcome to AutoCross autonomous talking RC cars!"
	print "############################################################"
	print "  ____  __ __  ______   ___     __  ____   ___   _____ _____"
	print " /    ||  |  ||      | /   \   /  ]|    \ /   \ / ___// ___/"
	print "|  o  ||  |  ||      ||     | /  / |  D  )     (   \_(   \_ "
	print "|     ||  |  ||_|  |_||  O  |/  /  |    /|  O  |\__  |\__  |"
	print "|  _  ||  :  |  |  |  |     /   \_ |    \|     |/  \ |/  \ |"
	print "|  |  ||     |  |  |  |     \     ||  .  \     |\    |\    |"
	print "|__|__| \__,_|  |__|   \___/ \____||__|\_|\___/  \___| \___|"
	print "############################################################"
	
	
def printCommandList():
	print "Type the one of the following commands"
	
	
	for i in range(len(commands)):
		print ""
		print commands[i][0]
		
	running = True
	
	try:
		while running:
			commandInput = raw_input(">>> ")
			
			commandSplit = commandInput.split()			
			commandIndex = findCommandIndex(commandSplit[0])
			
			
			#sanity check
			if commandIndex == -1:
				print "Invalid command please! Please try again"
			else:			
				# call defined command function
				method = globals()[commands[commandIndex][1]]
				
							
				try:
					if len(commandSplit) == 1:
						method()
					elif len(commandSplit) == 2:
						method(commandSplit[1])
					elif len(commandSplit) == 3:
						method(commandSplit[1], commandSplit[2])
					elif len(commandSplit) == 4:
						method(commandSplit[1], commandSplit[2], commandSplit[3])
				except TypeError as e:
					print e
					
			
	except KeyboardInterrupt:
		stopProgram()
		 
def findCommandIndex(commandName):
	for i in range(len(commands)):
		#check full command name and alias
		if commands[i][2] == commandName or len(commands[i]) > 4 and commands[i][3] == commandName:
			return i
	
	return -1

def testCycle():
	print "Not yet implemented"	
	
def drive(motorPercent):
	motorPercent = int(motorPercent)
	
	carMotor.setPower(motorPercent)
	
def steerDirection(direction, percent):
	direction = int(direction)
	percent = int(percent)
	
	carSteering.setPercentDirection(direction, percent)
	
	
def startAuto():
	print "Not yet impleneted"
	
def stop():
	#probable make carMotor a global object to avoid writting to it so much 

	carMotor.stop()
	
def stopProgram():
	#call stop car
	stop() 
	quit()

def help():
	
	printCommandList()
	

	
	
if __name__=="__main__": mainCommandInterface()
