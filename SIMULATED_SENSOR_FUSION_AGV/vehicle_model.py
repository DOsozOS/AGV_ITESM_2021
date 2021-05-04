import numpy as np
import math
#Useful Constants
g = 9.81
PI = 3.14159
class vehicle_model:
	def __init__(self,vehicle_mass,vehicle_l1,vehicle_l2,
		vehicle_width,height_iz,vehicle_wheel_radius,
		vehicle_initial_theta,vehicle_initial_vx,vehicle_initial_vy,
		vehicle_initial_x,vehicle_initial_y):
		#print("Preparing the vehicle model")
		self._mass = float(vehicle_mass)
		self._l1 = float(vehicle_l1)
		self._l2 = float(vehicle_l2)
		self._hg = float(height_iz)
		self._iz = float(1.0/12.0*self._mass*(math.pow(vehicle_width,2)+math.pow(self._l1+self._l2,2)))
		self._rw = float(vehicle_wheel_radius)
		self._x_position = float(vehicle_initial_x)
		self._y_position = float(vehicle_initial_y)
		self._vx = float(vehicle_initial_vx)
		self._vy = float(vehicle_initial_vy)
		self._theta = float(vehicle_initial_theta)
		self._c = 1.49
		self._b = 0.244
		self._e = 0.00
		self._vx = 0.00
		self._vy = 0.00
		self._wz = 0.00
		#print("Success!")
	def x0dot(self,omega):
		ret = omega
		#print("x0dot:"+str(ret))
		return ret
	def x1dot(self,fxf,fyf,fyr,st):
		ret = (self._l1*fyf*math.cos(st)-self._l2*fyr+self._l1*fxf*math.sin(st))/self._iz
		#print("x1dot:"+str(ret))
		return ret
	def x2dot(self,fxf,fyf,fyr,st,vx,omega):
		ret = (fyr + fyf*math.cos(st)+fxf*math.sin(st))/self._mass - vx*omega
		#print("x2dot:"+str(ret))
		return ret
	def x3dot(self,fxf,fyf,fxr,st,vy,omega):
		ret = (fxf + math.cos(st)+fxr-fyf*math.sin(st))/self._mass + vy*omega
		#print("x3dot:"+str(ret))
		return ret
	def x4dot(self,vy,theta,vx):
		ret = vx*math.cos(2*PI-theta)+vy*math.cos(3.0/4.0*PI-theta)
		#print("x4dot:"+str(ret))
		return ret
	def x5dot(self,vy,theta,vx):
		ret = vx*math.sin(2*PI-theta)+vy*math.sin(3.0/4.0*PI-theta)
		#print("x5dot:"+str(ret))
		return ret
	def fxr(self,acceleration,vx):
		ret = acceleration*self._mass #- 0.4*vx
		#print("fxr:"+str(ret))
		return ret
	def fzr(self,fxr):
		ret = (self._mass*g*self._l1)/(self._l1+self._l2) + (fxr*self._hg)/(self._l1+self._l2)
		#print("fzr:"+str(ret))
		return ret
	def fzf(self,fxr):
		ret = (self._mass*g*self._l1)/(self._l1+self._l2) - (fxr*self._hg)/(self._l1+self._l2)
		#print("fzf:"+str(ret))
		return ret
	def alpha_r(self,omega,vy,vx):
		if(vx==0):
			return 0
		else:
			ar = (self._l2*omega-vy)/vx 
			if math.isnan(ar):
				return 0
			else:
				#print("ar:"+str(ar))
				return ar
	def alpha_f(self,omega,vy,vx,st):
		if(vx==0):
			return 0
		else:
			af =  st-(self._l1*omega+vy)/vx
			if math.isnan(af):
				return 0
			else:
				#print("af:"+str(af))
				return af
	def fyf(self,fzf,af):
		ret = fzf*math.sin(self._c*math.atan(self._b*af - self._e*(self._b*af - math.atan(self._b*af)))) 
		#print("fyf:"+str(ret))
		return ret
	def fyr(self,fzr,ar):
		ret = fzr*math.sin(self._c*math.atan(self._b*ar - self._e*(self._b*ar - math.atan(self._b*ar))))
		#print("fyr:"+str(ret))
		return ret

	@property
	def mass(self):
		return self._mass
	@property
	def l1(self):
		return self._l1
	@property
	def l2(self):
		return self._l2
	@property
	def hg(self):
		return self._hg
	@property
	def iz(self):
		return self._iz
	@property
	def rw(self):
		return self._rw
	@property
	def x_position(self):
		return self._x_position
	@property
	def y_position(self):
		return self._y_position
	@property
	def vx(self):
		return self._vx
	@property
	def vy(self):
		return self._vy
	@property
	def wz(self):
		return self._wz
	
	
	
	
	