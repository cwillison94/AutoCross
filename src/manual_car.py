import pygame
import sys
import motor
import steering

pygame.init()
pygame.display.set_caption('Manual AutoCross Car')
screen = pygame.display.set_mode([400, 400])
screen.fill((255, 255, 255))
font = pygame.font.SysFont('Arial', 25)
pygame.display.update()

clock = pygame.time.Clock()
carMotor = motor.Motor()
carSteering = steering.Steering()

up_pressed = False

while True:
	try:
		
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
				sys.exit()
				
		keys_pressed = pygame.key.get_pressed()
		
		if keys_pressed[pygame.K_UP]:
			
			if not(up_pressed):	
				print "Motor forward"
				screen.fill((255,255,255))
				screen.blit(font.render('FORWARD', True, (255,0,0)), (100, 100))
				up_pressed = True
				carMotor.setPower(100)
		else:
			up_pressed = False
			carMotor.setPower(0)

				
		if keys_pressed[pygame.K_LEFT]:
			print "Wheel Reft"
			screen.fill((255,255,255))
			screen.blit(font.render('LEFT', True, (0,255,0)), (100, 100))
			carSteering.setPercentDirection(steering.DIRECTION_LEFT, 100)
		if keys_pressed[pygame.K_RIGHT]:
			print "Wheel Right"		
			screen.fill((255,255,255))
			screen.blit(font.render('RIGHT', True, (0,0,255)), (100, 100))
			carSteering.setPercentDirection(steering.DIRECTION_RIGHT, 100)
		else:
			#reset everything to normal
			up_pressed = False
			carMotor.setPower(0)
			carSteering.reset()
		
	
		pygame.display.update()
			
		clock.tick(60)
	except KeyboardInterrupt:
		pygame.quit()
		sys.exit()
		

		
		
		
