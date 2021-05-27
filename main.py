# Author: Gabriel Campa
# Project: 3 Wheel Blind Robot
# Main file
#------------------------------------------------------------------------------------------------------
from POLOfunct import *
#from POLOmoves import *
#------------------------------------------------------------------------------------------------------
#Program variables:
# ------------------------------------------------------------------------------------------------------

#------------------------------------------------------------------------------------------------------
#Initial Setup:
# Basic setup of pins.
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
global Bot
Bot = Wheels()
global Marco
Marco = POLOSpeed()
global Find
Find = Nav()
#------------------------------------------------------------------------------------------------------
#Main Loop:

print ("PoloBot On!")
print ("Hello!")
while True:
	print ("What would you like me to do?")
	print ("1.Spin in place")
	print ("2.Rotate around a certain radius")
	print ("3.Upload custom trajectory")
	print ("4.Enter desired Speed")
	menuopt=input("Select from the above options: ")
#------------------------------------------------------------------------------------------------------
#Option 1: Rotate in place
#------------------------------------------------------------------------------------------------------
	if (menuopt=='1'):
		print ("Ok!")
		vm=0
		wm=input("How fast would you like to spin? In units or rotations per second: ")
		dir=input("In which direction? Enter cw for CLOCKWISE and ccw for COUNTER CLOCKWISE: ")
		if dir=='ccw':
			#Get Wheel speed
			#Bot.getready()
			rd, ld = Marco.POLOspeed(float(vm), float(wm)*360)
			Bot.sendspeed(Marco.WRpul, Marco.WLpul, rd, ld)
			time.sleep(30)
			Bot.stop()
		elif dir=='cw':
			#Bot.getready()
			rd, ld = Marco.POLOspeed(float(vm), -float(wm)*360)
			Bot.sendspeed(Marco.WRpul, Marco.WLpul, rd, ld)
			time.sleep(30)
			Bot.stop()
#------------------------------------------------------------------------------------------------------
#Option 2: Move in circle
#------------------------------------------------------------------------------------------------------
	elif menuopt=='2':
		print ("Great!")
		rads=input("What is the radius? Should be in inches!: ")
		wm=input("How many degrees should I rotate in a second: ")
		vm=Find.circle(float(rads),float(wm))
		#Bot.getready()
		rd,ld=Marco.POLOspeed(vm,float(wm))
		Bot.sendspeed(Marco.WRpul,Marco.WLpul,rd,ld)
		time.sleep(30)
		Bot.stop()
#------------------------------------------------------------------------------------------------------
#Option 3: Custom speed
#------------------------------------------------------------------------------------------------------
	elif menuopt=='3':
		vm=input('Linear Velovity (m/s): ')
		wm=input('Angular Velocity (degrees/s): ')
		tm=input('Time Duration: ')
		#Bot.getready()
		rd,ld=Marco.POLOspeed(float(vm),float(wm))
		Bot.sendspeed(Marco.WRpul,Marco.WLpul,rd,ld)
		time.sleep(float(tm))
		Bot.stop()
#------------------------------------------------------------------------------------------------------
#Option 4: Distance
#------------------------------------------------------------------------------------------------------
	elif menuopt=='4':
		d=input('Speed Vm: ')
		wm=0
		#Bot.getready()
		rd,ld=Marco.POLOspeed(float(d),wm)
		Bot.sendspeed(Marco.WRpul,Marco.WLpul,rd,ld)
		time.sleep(30)
		Bot.stop()
	time.sleep(1)

