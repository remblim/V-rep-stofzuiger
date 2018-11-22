import numpy as np
import math

class Slam():
	def __init__(self):
		#parameters
		self.rejection_distance = 0.2
		self.sensors = ['Laser_Sensor','Laser_Sensor0','Laser_Sensor1','Laser_Sensor2']
		print('init slam')
		
		
		#initialisatie
		self.measured_distance = []
		self.landmark_ray_center = []
		self.landmark_ray_distance = []
		self.measured_points_x = []
		self.measured_points_y = []
		self.old_map = False
		
	def calculations(self,distance,oude_locatie):
		#distance is de afstand gescand door de sensors
		#oude_locatie is de positie van de robot [x,y,hoek(radialen)]
		#variabele initialisatie
		derivate_measured_distance = []
		raycount = 0
		totale_afstand = 0
		totale_ray = 0
		op_cilinder = False
		measured_points_x = []
		measured_points_y = []
		i = 0
		
		#alle afstanden van de sensoren achterelkaar plaatsen
		while i < len(self.sensors):
			self.measured_distance.extend(distance[self.sensors[i]])
			i+=1
			
		i = 0
		
		#hoek per scan
		self.angle_per_scan = 2 * np.pi / len(self.measured_distance)	
		
		#Bepalen van landmarks
		self.landmark_ray_center,self.landmark_ray_distance = retrieve_landmarks(measured_distance)
		
		#hoek van landmarks bepalen door vermenigvuldigen van hoek per scan en ray_center
		heading_landmarks = np.multiply(self.angle_per_scan,self.landmark_ray_center)
		
		#robot kaart maken
		landmark_x = np.cos(heading_landmarks) * self.landmark_ray_distance
		landmark_y = np.sin(heading_landmarks) * self.landmark_ray_distance
		
		if self.old_map is False:
			#als self_map false is is er nog geen kaart aangemaakt (eerste cyclus).
			#return de oude locatie omdat er nog geen betere berekend kon worden.
			self.old_map = np.column_stack((landmark_x,landmark_y))
			return oude_locatie
			
		else:
			#tweede cyclus, positie berekenen
			#Eerst landmarks koppelen waarvoor de translatie nodig is van de robot nulpunt naar kaart nulpunt.
			#hiervoor twee punten, robot punt en 1m voor de robot en deze transformeren naar de kaart.
			s,r,tx,ty = robot_map_translation_calculation(positie)
			
			#nu de robot kaar transformeren over de berekende translatie, eerst in [,][,] formaat brengen
			robot_map = np.column_stack((landmark_x,landmark_y))
			
			#Robot_map transformeren naar echte kaart
			transformed_robot_map = transform(robot_map,s,r,tx,ty)
			
			#Landmarks van robot en kaart bij elkaar brengen voor slam
			connection = connect_landmark(self.old_map,transformed_robot_map)
			
			#Voorbereiden van maps voor slam, de volgorde van punten goedzetten
			robot_map_sorted = []
			for item in connection:
				robot_map_sorted.append(robot_map[item[1]])
			
			#slam tijd!!!
			s,r,tx,ty = estimate(robot_map_sorted,self.old_map)
			
			#positie bepalen
			nieuwe_positie = transform([0,0],s,r,tx,ty)

		heading_measured_points = np.multiply(self.angle_per_scan,list(range(0,len(self.measured_distance))))

		i = 0
		while i < len(self.measured_distance):
			measured_points_x.append(np.cos(heading_measured_points[i]) * self.measured_distance[i])
			measured_points_y.append(np.sin(heading_measured_points[i]) * self.measured_distance[i])
			i += 1
			
		
		
		return measured_points_x,measured_points_y,nieuwe_positie

	def transform(p,s,r,tx,ty):
		'''
		Parameter
			p
				point [x, y] or list of points [[x1,y1], [x2,y2], ...]
		'''
		def transform_one(q):
			return [s * q[0] - r * q[1] + tx,
					r * q[0] + s * q[1] + ty]

		if not isinstance(p[0], list):
			# Single point
			return transform_one(p)
		# else
		return list(map(transform_one, p))
		
	def connect_landmark(map1,map2,rejection_distance):
		#map1x,map1y are the map wich should be mapped to. map 2 will be mapped to map 1
		#return [,][,]
		connecting = []
		for i, row2 in map2x:
			diff = []
			for row1 in map1x:
				diff.append(np.sqrt(np.square(row1[0] - row2[0]) + np.square(row1[1] - row2[1])))
			if np.minimum(diff) < rejection_distance:
				connecting.append([i,np.argmin(diff)])	
		
		return connecting
	
	def retrieve_landmarks(measured_distance):
		#measured_distance is a list of points around the robot scanned by lidar scanner. list is pure distance		
		landmark_ray_distance, landmark_ray_center = []
		derivate_measured_distance = []
		op_cilinder = False
		raycount = 0
		totale_afstand = 0
		totale_ray = 0
		while i < len(measured_distance)-2:
			derivate_measured_distance.append((measured_distance[(i-1)]-measured_distance[(i+1)])/2)
			if derivate_measured_distance[i] > 0.4:
				op_cilinder = True
			if derivate_measured_distance[i] < -0.4:
				if 5 < raycount:
					landmark_ray_center.append(totale_ray/raycount + 1)
					landmark_ray_distance.append(totale_afstand/raycount+0.25)
				op_cilinder = False
				raycount = 0
				totale_afstand = 0
				totale_ray = 0
			if op_cilinder == True:
				raycount += 1
				totale_afstand += measured_distance[i+1]
				totale_ray += i
			i += 1
		return landmark_ray_center, landmark_ray_distance
	
	def robot_map_translation_calculation(positie):
		#input [x,y,hoek[radialen]]
		#output s,r,tx,ty
		robot_positie = [[positie[0],positie[1]],[np.cos(positie[2]),np.sin(positie[2])]]
		
		#kaart positie 0 punt altijd op 0,0 met aftand 1 in y richting
		map_positie = [[0,0],[0,1]]
		
		#transformatie berekenen
		Objtrans = Transform
		Objtrans.estimate(robot_positie,map_positie)
		
		return Objtrans.s,Objtrans.r,Objtrans.tx,Objtrans.ty
		
	def estimate(domainpoints, rangepoints):
		'''
		Parameters
			domainpoints
				list of [x, y] 2D lists
			rangepoints
				list of [x, y] 2D lists
		'''

		# Alias
		X = domainpoints
		Y = rangepoints

		# Allow arrays of different length but
		# ignore the extra points.
		N = min(len(X), len(Y))

		a1 = b1 = c1 = d1 = 0.0
		a2 = b2 = 0.0
		ad = bc = ac = bd = 0.0
		for i in range(N):
			a = X[i][0]
			b = X[i][1]
			c = Y[i][0]
			d = Y[i][1]
			a1 += a
			b1 += b
			c1 += c
			d1 += d
			a2 += a * a
			b2 += b * b
			ad += a * d
			bc += b * c
			ac += a * c
			bd += b * d

		# Denominator.
		# It is zero iff X[i] = X[j] for every i and j in [0, n).
		# In other words, iff all the domain points are the same.
		den = N * a2 + N * b2 - a1 * a1 - b1 * b1

		if abs(den) < 1e-8:
			# The domain points are the same.
			# We assume the translation to the mean of the range
			# to be the best guess. However if N=0, assume identity.
			if N == 0:
				return Transform(1.0, 0.0, 0.0, 0.0)
			else:
				return Transform(1.0, 0.0, (c1 / N) - a, (d1 / N) - b)

		# Estimators
		s = (N * (ac + bd) - a1 * c1 - b1 * d1) / den
		r = (N * (ad - bc) + b1 * c1 - a1 * d1) / den
		tx = (-a1 * (ac + bd) + b1 * (ad - bc) + a2 * c1 + b2 * c1) / den
		ty = (-b1 * (ac + bd) - a1 * (ad - bc) + a2 * d1 + b2 * d1) / den

		return Transform(s, r, tx, ty)