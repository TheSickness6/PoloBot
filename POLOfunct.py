# Using Rpi GPIO, time, and gpiozero libraries.
import RPi.GPIO as GPIO
import numpy as np
import time
import math
import csv
import smbus
# ------------------------------------------------------------------------------------------------------
# ------------------------------------------------------------------------------------------------------
# My Functions:

class POLOSpeed:
	def __init__(self):
		# POLO Dimensions
		self.Radiusm=0.0333375  # Wheel Radius
		self.Lengthm=0.132  # Distance between wheels
		self.Radiusin=0.0333375 * 39.37
		self.Lengthin=0.132 * 39.37
		self.MperStep=0.010524 #Meter per step
		self.InperStep=.000414344 #Inch per step
		# POLO Speed
		self.Vm=0
		self.Wm=0
		self.WR=0
		self.WL=0
		self.WRpul=0
		self.WLpul=0


    # Wheel speed calculation
    # MAKE SURE TRAJECTORY IS IN Degrees!!!!!
    # m/s and deg/s
	def angvel(self):
		jacob = np.matrix([[1 / self.Radiusm, self.Lengthm / (2 * self.Radiusm)],
                           [1 / self.Radiusm, -self.Lengthm / (2 * self.Radiusm)]])  # Precalculated jacobian
		speed = np.matrix([[self.Vm], [self.Wm]])
		self.WR, self.WL = jacob * speed  # Individual wheel angular velocities calculated deg/s

	# Main Function
	#Input linear and angular velocities
	#Saves values and calculates pulses required and direction
	def POLOspeed(self, vel, ang):
		self.Vm = vel
		self.Wm = ang
		self.angvel()
		if self.WR<0:
			rdir=1
		else:
			rdir=0
		if self.WL<0:
			ldir=1
		else:
			ldir=0
		self.WRpul=abs(self.WR)*0.166667
		self.WLpul=abs(self.WL)*0.166667
		return rdir,ldir



class Nav:
	def __init__(self):
		# POLO Dimensions
		self.Radiusm = 0.0333375  # Wheel Radius
		self.Lengthm = 0.132  # Distance between wheels
		self.Radiusin = .0333375 * 39.37
		self.Lengthin = .132 * 39.37
		self.thetaDeg = 0
		self.length = 0

	# Calculates Transfomation matrix from origin to current XY position.
	def oTn(self, x, y):
		self.thetaDeg = math.atan2(y, x)
		self.length = np.sqrt(x * x + y * y)
		tran = np.matrix([[math.cos(self.thetaDeg), -math.sin(self.thetaDeg), 0, self.length * math.cos(self.thetaDeg)],
                      [math.sin(self.thetaDeg), math.cos(self.thetaDeg), 0, self.length * math.sin(self.thetaDeg)], [0, 0, 1, 0], [0, 0, 0, 1]])
		return tran

	def circle(self,radi,wm):
		vdif=wm/39
		vsum=vdif*(2*(radi/39.37))/self.Lengthm
		V=vsum/2
		return V

    # #Calculate Linear and Angular velocities
    # def VelCalc(self):
            # #Import position plot code
            # #Convert to double
            # #Find length of x
            # dt=t/1000
            # dy=[]
            # dx=[]
            # dtheta=[]
            # for n in range(1,1000):
            # a=self.oTn(x(n),y(n))
            # b=self.oTn(x(n-1),y(n-1))
            # dx.append(a[0][3]-b[0][3])
            # dy.append(a[1][3]-b[1][3])
            # dtheta.append(math.atan2(dy[n],dx[n]))
            # self.Vx=dx/dt
            # self.Vx.append(0)
            # self.Vy=dy/dt
            # self.Vy.append(0)
            # self.Vm=math.sqrt((Vx*Vx)+(Vy*Vy))
            # self.Wm=dtheta/dt
            # self.Wm.append(0)

    # def TrajCSV(self):
    # self.VelCalc()
    # for n in range(0,1000):
        # wr,wl=self.angveln(self.Vm[n],self.Wm[n])
        # self.WRm.append(wr)
        # self.WLm.append(wl)

class Wheels:
	def __init__(self):
		self.RAddr=0x66
		self.LAddr=0x67
		self.bus = smbus.SMBus(1)
		self.WRm = []
		self.WLm = []

	def stop(self):
		self.bus.write_byte_data(self.RAddr,0, 255)
		self.bus.write_byte_data(self.LAddr,0, 255)

	#def getready(self):
	#	self.bus.write_byte_data(self.RAddr,0, 66)
	#	self.bus.write_byte_data(self.LAddr,0, 66)

	#def getspeed(self):
	#	self.WRm.append(self.bus.read_byte(self.RAddr)*12)
	#	self.WLm.append(self.bus.read_byte(self.LAddr)*12)


	def sendspeed(self,Wr,Wl,r,l):
		print(" Left RPM ",int(Wl),"Right RPM ",int(Wr)," LDIR ",l," RDIR ",r)
		try:
			self.bus.write_byte_data(self.RAddr,int(Wr), r)
		except:
			self.bus.write_byte_data(self.RAddr,int(Wr), r)
		try:
			self.bus.write_byte_data(self.LAddr,int(Wl), l)
		except:
			self.bus.write_byte_data(self.LAddr,int(Wl), l)
