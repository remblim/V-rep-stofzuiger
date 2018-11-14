import numpy as np
import nudged

class Slam():
	def __init__(self):
		#parameters
		self.rejection_distance = 0.2
		self.sensors = ['Laser_Sensor','Laser_Sensor0','Laser_Sensor1','Laser_Sensor2']
		print('init slam')
		
		
		#initialisatie
		self.op_cilinder = False
		self.measured_distance = []
		self.derivate_measured_distance = []
		self.raycount = 0
		self.landmark_ray_center = []
		self.landmark_ray_distance = []
		self.totale_afstand = 0
		self.totale_ray = 0
		self.measured_points_x = []
		self.measured_points_y = []
		self.s = []
		self.r = []
		self.tx = []
		self.ty = []
		self.angle_per_scan = 2 * np.pi / len(measured_distance)
		
	def slam(self,distance):
				
		i = 0
		
		while i < len(self.sensors):
			self.measured_distance.extend(self.distance[sensors[i]])
			i+=1
		i = 0
		
		
		while i < len(self.measured_distance)-2:
			derivate_measured_distance.append((self.measured_distance[i-1]-self.measured_distance[i+1])/2)
			if self.derivate_measured_distance[i] > 0.4:
				op_cilinder = True
			if self.derivate_measured_distance[i] < -0.4:
				if 5 < self.raycount:
					landmark_ray_center.append(self.totale_ray/self.raycount + 1)
					landmark_ray_distance.append(self.totale_afstand/self.raycount+0.25)
				op_cilinder = False
				raycount = 0
				totale_afstand = 0
				totale_ray = 0
			if op_cilinder == True:
				raycount += 1
				totale_afstand += self.measured_distance[i+1]
				totale_ray += i
			i += 1

		heading_landmarks = np.multiply(self.angle_per_scan,landmark_ray_center) + position[2]
		landmark_x = np.cos(heading_landmarks) * landmark_ray_distance
		landmark_y = np.sin(heading_landmarks) * landmark_ray_distance
		
		i = 0
		new_map = []
		while i < len(landmark_x):
			new_map.append(landmark_x[i],landmark_y[i])
			i += 1
		
		trans = nudged.estimate(new_map,old_map)
		self.s.append(trans.s)
		self.r.append(trans.r)
		self.tx.append(trans.tx)
		self.ty.append(trans.ty)
		position = [0,0]
		i = 0
		while i < len(s):
			position = transform_one(position,self.s[i],self.r[i],self.tx[i],self.ty[i])
			i += 1
			
		heading_measured_points = np.multiply(angle_per_scan,list(range(0,len(measured_distance)))) + new_location[2]
		i = 0
		while i < len(measured_distance):
			measured_points_x.append(np.cos(heading_measured_points[i]) * measured_distance[i])
			measured_points_y.append(np.sin(heading_measured_points[i]) * measured_distance[i])
			i += 1
			
		
		
		return measured_points_x,measured_points_y,position

	def transform_one(q,s,r,tx,ty):
		return [self.s * q[0] - self.r * q[1] + self.tx, self.r * q[0] + self.s * q[1] + self.ty]