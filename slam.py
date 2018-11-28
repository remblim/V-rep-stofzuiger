import numpy as np

class Slam():
	def __init__(self):
		#parameters
		self.rejection_distance = 0.2
		self.sensors = ['Laser_Sensor','Laser_Sensor0','Laser_Sensor1','Laser_Sensor2']
		print('init slam')
		
		
		#initialisatie
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
		measured_distance = []
		i = 0
		
		#alle afstanden van de sensoren achterelkaar plaatsen
		while i < len(self.sensors):
			measured_distance.extend(distance[self.sensors[i]])
			i+=1
			
		i = 0
		
		#hoek per scan
		self.angle_per_scan = 2 * np.pi / len(measured_distance)	
		
		#punten bepalen voor plot
		heading_measured_points = np.multiply(self.angle_per_scan,list(range(0,len(measured_distance))))
		i = 0
		while i < len(measured_distance):
			measured_points_x.append(np.cos(heading_measured_points[i]) * measured_distance[i])
			measured_points_y.append(np.sin(heading_measured_points[i]) * measured_distance[i])
			i += 1
		
		#Bepalen van landmarks
		landmark_ray_center,landmark_ray_distance = retrieve_landmarks(measured_distance)
		#hoek van landmarks bepalen door vermenigvuldigen van hoek per scan en ray_center
		heading_landmarks = np.multiply(self.angle_per_scan,landmark_ray_center)
		
		#robot kaart maken
		landmark_x = np.cos(heading_landmarks) * landmark_ray_distance
		landmark_y = np.sin(heading_landmarks) * landmark_ray_distance
		
		if self.old_map is False:
			print('map maken')
			#als self_map false is is er nog geen kaart aangemaakt (eerste cyclus).
			#return de oude locatie omdat er nog geen betere berekend kon worden.
			self.old_map = np.column_stack((landmark_x,landmark_y)).tolist()
			self.old_map_x = landmark_x
			self.old_map_y = landmark_y
			return measured_points_x,measured_points_y,self.old_map_x,self.old_map_y,[],[],False
			
		else:
			print('map gebruiken')
			#tweede cyclus, positie berekenen
			#Eerst landmarks koppelen waarvoor de translatie nodig is van de robot nulpunt naar kaart nulpunt.
			transformed_robot_map_x = np.cos(oude_locatie[2]+heading_landmarks) * landmark_ray_distance + oude_locatie[0]
			transformed_robot_map_y = np.sin(oude_locatie[2]+heading_landmarks) * landmark_ray_distance - oude_locatie[1]
			#Landmarks van robot en kaart bij elkaar brengen voor slam
			transformed_robot_map = np.column_stack((landmark_x,landmark_y)).tolist()
			connection = connect_landmark(self.old_map_x,self.old_map_y,transformed_robot_map_x,transformed_robot_map_y,self.rejection_distance)
			#Voorbereiden van maps voor slam, de volgorde van punten goedzetten
			robot_map_sorted = []
			for item in connection:
				robot_map_sorted.append([landmark_x[item[0]],landmark_y[item[1]]])
			print(robot_map_sorted)
			
			#slam tijd!!!
			s,r,tx,ty = estimate(robot_map_sorted,self.old_map)
			print(s,r,tx,ty)
			
			#positie bepalen
			nieuwe_positie = transform([0,0],s,r,tx,ty)
			print(nieuwe_positie)
			#for item in transformed_robot_map:
			#	transformed_robot_map_x.append(item[0])
			#	transformed_robot_map_y.append(item[1])
		
		return measured_points_x,measured_points_y,self.old_map_x,self.old_map_y,transformed_robot_map_x,transformed_robot_map_y,nieuwe_positie

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
	#	# Single point
		return transform_one(p)
	# else
	transformed = []
	for item in p:
		transformed.append(transform_one(item))
	return transformed
		
def connect_landmark(map1x,map1y,map2x,map2y,rejection_distance):
	#map1x,map1y are the map wich should be mapped to. map 2 will be mapped to map 1
	#return [,][,]
	connecting = []
	i = 0
	while i < len(map1x):
		diff = []
		n = 0
		while n < len(map2x):
			diff.append(np.sqrt(np.square(map1x[i]-map2x[n])+np.square(map1y[i]-map2y[n])))
			
			
			n += 1
		if rejection_distance > np.amin(diff):
			connecting.append([i,np.argmin(diff)])
		print(diff)
		i += 1
	print(connecting)
	return connecting
	
def retrieve_landmarks(measured_distance):
	#measured_distance is a list of points around the robot scanned by lidar scanner. list is pure distance		
	landmark_ray_distance, landmark_ray_center = [],[]
	derivate_measured_distance = []
	op_cilinder = False
	raycount = 0
	totale_afstand = 0
	totale_ray = 0
	i = 0
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
	
def robot_map_translation_calculation(locatie):
	#input [x,y,hoek[radialen]]
	#output s,r,tx,ty
	robot_positie = [[locatie[0],locatie[1]],[np.cos(locatie[2]+locatie[0]),np.sin(locatie[2])+locatie[1]]]
	
	#kaart positie 0 punt altijd op 0,0 met aftand 1 in y richting
	map_positie = [[0,0],[0,1]]
	
	#transformatie berekenen
	s,r,tx,ty = estimate(map_positie,robot_positie)
	
	return s,r,tx,ty
		
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

	return s, r, tx, ty