
import time

#[help string, method name, command name, alias]
commands = [
	["test-cycle]\t\t-(tc)runs a test cycle on the car. forward + left/right","testCycle","test-cycle",'tc'], 
	["auto\t\t-(a)run the car in fully autonmous mode with everythin ON", "startAuto","auto", "a"], 
	["drive <motor_percent>		-(d)set the motor to drive at a percent of 0 to 100","drive", "drive", "d"],
	["stop\t\t-stops everything","stopCar", "stop"],
	["exit\t\t-stops and exits", "stopProgram", "exit"]
]

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
		print commands[i][0]
		
	running = True
	
	try:
		while running:
			commandInput = raw_input(">>>")
			
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
		print "Press Ctrl-C again to exit"
		 
def findCommandIndex(commandName):
	for i in range(len(commands)):
		if commands[i][2] == commandName:
			return i
	
	return -1

def testCycle():
	print "Not yet implemented"	
	
def drive(motorPercent):
	print "Not yet implemented"
	motorPercent = int(motorPercent)
	print "Drive value of " + str(motorPercent)

def stopProgram():
	
	#call stop car 
	print "program exiting"
	
	quit()

	
	
if __name__=="__main__": mainCommandInterface()
