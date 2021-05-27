class Traj:
	def __init__(self):
        self.dt = t/1000
		self.dy = []
		self.dx = []
        self.dtheta = []
        self.Vm = 0
        self.Wm = 0
		
	#Calculate Linear and Angular velocities
    def VelCalc(self):
		#Import position plot code
		#Convert to double
		#Find length of x
		for n in range(1,1000):
            a=self.oTn(x(n),y(n))
            b=self.oTn(x(n-1),y(n-1))
            self.dx.append(a[0][3]-b[0][3])
            self.dy.append(a[1][3]-b[1][3])
            dtheta.append(math.atan2(self.dy[n],self.dx[n]))
            self.Vx=self.dx/self.dt
            self.Vx.append(0)
            self.Vy=self.dy/self.dt
            self.Vy.append(0)
            self.Vm=math.sqrt((self.Vx*self.Vx)+(self.Vy*self.Vy))
            self.Wm=self.dtheta/self.dt
            self.Wm.append(0)

     def TrajCSV(self):
     self.VelCalc()
		for n in range(0,1000):
			wr,wl=self.angveln(self.Vm[n],self.Wm[n])
			self.WRm.append(wr)
			self.WLm.append(wl)