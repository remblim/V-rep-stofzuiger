import numpy as np
import vrep
import sys
import matplotlib.pyplot as plt
import time

Robot_parts = ['linkermotor','rechtermotor','Laser_Sensor','Laser_Sensor0','Laser_Sensor1','Laser_Sensor2','Lidar_Motor']

vrep.simxFinish(-1) #close all open connections
clientID = vrep.simxStart('127.0.0.1',19997,True,True,5000,5) #v-rep connections
vrep.simxSynchronous(clientID,True)
# start the simulation:
vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking)
if clientID == -1:
	print('connection failed')
	exit()
else:
	print('connection succeeded')	

def get_handle (clientID,part_name):
	errorCode,current_handle = vrep.simxGetObjectHandle(clientID,part_name,vrep.simx_opmode_blocking)
	handle[part_name] = current_handle
	if errorCode > 0:
		print(part_name + ' has no handle')
		exit()
	else:
		print(part_name + ' has handle')
		
def move_motor(part_name, speed):
	vrep.simxSetJointTargetVelocity(clientID,handle[part_name],speed,vrep.simx_opmode_oneshot)

def move_lidar_motor(locatie):
	vrep.simxSetJointPosition(clientID, handle['Lidar_Motor'], locatie, vrep.simx_opmode_oneshot)
	
def position_motor(part):		
	errorCode,position = vrep.simxGetJointPosition(clientID,handle[part],vrep.simx_opmode_blocking)
	if position > 0:
		positie[part] = position
	else:
		positie[part] = position + np.pi

def position_lidar_motor(part):
	errorCode,positie[part] = vrep.simxGetJointPosition(clientID,handle[part],vrep.simx_opmode_blocking)

def distance_sensor(part_name):
	return vrep.simxReadProximitySensor(clientID,handle[part_name],vrep.simx_opmode_blocking)

def rotation_calculation(part):
	old_position = positie[part]
	position_motor(part)
	rotation_difference[part] = positie[part] - old_position
	if rotation_difference[part] < -2:
		rotation_difference[part] += np.pi
	if rotation_difference [part] < 0.05 or rotation_difference[part] > np.pi -0.05:
		rotation_difference[part] = 0
		
def position_change_calculation():
	distance_difference['linkermotor'] = (rotation_difference['linkermotor']*correction_factor/(np.pi*2))*(omtrek_wiel)
	distance_difference['rechtermotor'] = (rotation_difference['rechtermotor']*correction_factor/(np.pi*2))*(omtrek_wiel)
	
	alpha = (distance_difference['rechtermotor']-distance_difference['linkermotor'])/afstand_wielen
	if alpha == 0:
		locatie[0] = locatie[0] + distance_difference['linkermotor']*np.cos(locatie[2])
		locatie[1] = locatie[1] + distance_difference['linkermotor']*np.sin(locatie[2])
	
	else:
		if distance_difference['linkermotor']<distance_difference['rechtermotor']:
			radius = distance_difference['linkermotor']/alpha
		else:
			radius = distance_difference['rechtermotor']/alpha
			
		if radius > 0:
			robot_radius = radius + afstand_wielen/2
		else:
			robot_radius = radius - afstand_wielen/2
			
		rotatie_centrum[0]=locatie[0]+(robot_radius)*np.cos(locatie[2]) #x
		rotatie_centrum[1]=locatie[1]+(robot_radius)*np.sin(locatie[2]) #y
		locatie[2] = (locatie[2] + alpha)%(2*np.pi)
		locatie[0] = rotatie_centrum[0] - np.cos(locatie[2]) * (robot_radius)
		locatie[1] = rotatie_centrum[1]- np.sin(locatie[2]) * (robot_radius)
		afgelegde_weg_x.append(locatie[0])
		afgelegde_weg_y.append(locatie[1])
		
def proximity_sensor_calcultion(part_name):
	errorCode,prox_status,prox_mesured_distance,detected_handle,normal_vector = distance_sensor(part_name)
	sensor_heading = locatie[2] + positie['Lidar_Motor'] + hoeken_scanners[part_name]
	if prox_status == True:
		sensor_location_x = locatie[0] - 0.15 * np.sin(locatie[2])
		sensor_location_y = locatie[1] + 0.15 * np.cos(locatie[2])
		meetpunten[part_name].append(prox_mesured_distance[2])		
		measured_points_x.append(sensor_location_x - prox_mesured_distance[2] * np.sin(sensor_heading))
		measured_points_y.append(sensor_location_y + prox_mesured_distance[2] * np.cos(sensor_heading))
		
def landmark_generation():
	global gemeten_afstanden
	global meetpunten
	gemeten_afstanden = []
	gemeten_afstanden.extend(meetpunten['Laser_Sensor1'])
	gemeten_afstanden.extend(meetpunten['Laser_Sensor2'])
	gemeten_afstanden.extend(meetpunten['Laser_Sensor'])
	gemeten_afstanden.extend(meetpunten['Laser_Sensor0'])
	meetpunten['Laser_Sensor'] = []
	meetpunten['Laser_Sensor0'] = []
	meetpunten['Laser_Sensor1'] = []
	meetpunten['Laser_Sensor2'] = []
	n = 0
	global afgeleide_gemeten_afstand
	afgeleide_gemeten_afstand = []
	raycount = 0
	gemiddelde_afstand = 0
	op_cilinder = False
	cilinders = 0
	gemiddelde_ray = 0
	global landmark_ray_center
	global landmark_distance
	global landmark_heading
	global landmark_x_robot
	global landmark_y_robot
	landmark_ray_center = []
	landmark_distance = []
	while n < len(gemeten_afstanden)-2:
		afgeleide_gemeten_afstand.append((gemeten_afstanden[n] - gemeten_afstanden[n+2]) / 2)
		if afgeleide_gemeten_afstand[n] > 0.4:
			op_cilinder = True
		if afgeleide_gemeten_afstand[n] < -0.4:
			if raycount > 0:
				cilinders = cilinders + 1
				landmark_ray_center.append(gemiddelde_ray/raycount+1)
				landmark_distance.append(gemiddelde_afstand/raycount + 0.2)
				landmark_heading.append(((hoek_scannen*landmark_ray_center[-1])/365)*2*np.pi+np.pi+locatie[2])
				landmark_x_robot.append(-np.sin(landmark_heading[-1])*landmark_distance[-1]+locatie[0])
				landmark_y_robot.append(np.cos(landmark_heading[-1])*landmark_distance[-1]+locatie[1])
			op_cilinder = False
			raycount = 0
			gemiddelde_afstand = 0
			gemiddelde_ray = 0
		if op_cilinder == True:
			raycount = raycount + 1
			gemiddelde_afstand = gemiddelde_afstand + gemeten_afstanden[n+1]
			gemiddelde_ray = gemiddelde_ray + n
		n = n+1

def landmark_comparison(landmark_x_robot,landmark_y_robot,landmark_x_map,landmark_y_map,rejection_distance): 
	robot_x = []
	robot_y = []
	map_x = []
	map_y = []
	if len(landmark_x_map) == 0:
		landmark_x_map.extend(landmark_x_robot)
		landmark_y_map.extend(landmark_y_robot)
		landmark_x_robot = []
		landmark_y_robot = []
	else:
		n=0
		kolom = []
		while n < len(landmark_x_map):
			i = 0
			kolom = []
			while i < len(landmark_x_robot):
				distance = np.sqrt(np.square(landmark_x_map[n]-landmark_x_robot[i])+np.square(landmark_y_map[n]-landmark_y_robot[i]))
				kolom.append(distance)
				i = i + 1
			if np.min(kolom) < rejection_distance:
				map_x.append(landmark_x_map[n])
				map_y.append(landmark_y_map[n])
				robot_x.append(landmark_x_robot[np.argmin(kolom)])
				robot_y.append(landmark_y_robot[np.argmin(kolom)])
			n=n+1
		landmark_x_robot = []
		landmark_y_robot = []
	return map_x,map_y,robot_x,robot_y
		
def assenstelsel_transformeren_abs_locaal(lijst_x,lijst_y):
	zwaartepunt_x = np.aveg(lijst_x)
	zwaartepunt_y = np.aveg(lijst_y)
	local_x = lijst_x - zwaartepunt_x
	local_y = lijst_y - zwaartepunt_y
	return zwaartepunt_x,zwaartepunt_y,local_x,local_y
	
def assenstelsel_transformeren_locaal_abs(lijst_x,lijst_y,zwaartepunt_x,zwaartepunt_y):
	absoluut_x = lijst_x + zwaartepunt_x
	absoluut_y = lijst_y + zwaartepunt_y
	return absoluut_x,absoluut_y
	
def slam_berekeningen(robot_x,robot_y,map_x,map_y,zwaartepunt_robot_x,zwaartepunt_robot_y,zwaartepunt_map_x,zwaartepunt_map_y):
	for i in robot_x:
		rr += np.square(robot_x[i]) + np.square(robot_y[i])
		ll += np.square(map_y[i]) + np.square(map_y[i])
		cs += robot_x[i]*map_x[i] + robot_y[i] * map_y[i]
		ss += -robot_x[i] * map_x[i] + robot_y[i] * map_y[i]
	if ll == 0 and rr == 0:
		return None,None,None,None
	else:
		scale = np.sqrt(rr/ll)
		cos = cs / np.sqrt(np.square(cs)+np.square(ss))
		sin = ss / np.sqrt(np.square(cs)+np.square(ss))
		translate_x = zwaartepunt_robot_x - scale * (cos * zwaartepunt_map_x - sin * zwaartepunt_map_y)
		translate_y = zwaartepunt_robot_y - scale * (sin * zwaartepunt_map_x + cos * zwaartepunt_map_y)
		angle = arctan(sin/cos)
		return translate_x,translate_y,angle,scale

def transformatie_punten(robot_x,robot_y,translate_x,translate_y,angle,scale):
	lac = scale * np.cos(angle)
	las = scale * np.sin(angle)
	x = lac * robot_x - las * robot_y + translate_x
	y = las * robot_y - lac * robot_x + translate_y
	return (x,y)
						
def visual_map_making(map_x,map_y,robot_x,robot_y):
	plt.gcf().clear()
	plt.subplot(221)
	plt.plot(afgelegde_weg_x, afgelegde_weg_y,color='red')
	plt.scatter(map_measured_points_x, map_measured_points_y,color='green')
	plt.axis('equal')
	plt.subplot(222)
	plt.scatter(map_x,map_y,color='blue')
	plt.scatter(robot_x,robot_y,color='red')
	i = 0
	while i < len(map_x):
		plt.text(map_x[i],map_y[i],str(i),color='blue')
		plt.text(robot_x[i]+0.5,robot_y[i],str(i),color='red')
		i+=1
	plt.axis('equal')
	plt.subplot(212)
	global gemeten_afstanden
	plt.plot(gemeten_afstanden, color='green')
	plt.scatter(landmark_ray_center, landmark_distance,color = 'purple')
	plt.plot(afgeleide_gemeten_afstand, color='red')
	plt.draw()
	plt.pause(0.001)
	
#onderdelen aansturen
def wielen_berekeningen():
	rotation_calculation('linkermotor')
	rotation_calculation('rechtermotor')
	position_change_calculation()

def prox_sensor_berekeningen():
	position_lidar_motor('Lidar_Motor')
	proximity_sensor_calcultion('Laser_Sensor')
	proximity_sensor_calcultion('Laser_Sensor0')
	proximity_sensor_calcultion('Laser_Sensor1')
	proximity_sensor_calcultion('Laser_Sensor2')
	
def landmark_berekeningen():
	map_x,map_y,robot_x,robot_y = [],[],[],[]
	global rejection_distance
	
	landmark_generation()
	map_x,map_y,robot_x,robot_y = landmark_comparison(landmark_x_robot,landmark_y_robot,landmark_x_map,landmark_y_map,rejection_distance)
	visual_map_making(map_x,map_y,robot_x,robot_y)
	
#beweeg modusen
def rond_scannen():
	print('Rondje scannen')
	while True:
		prox_sensor_berekeningen()
		move_motor('linkermotor', 0.0)
		move_motor('rechtermotor', 0.0)
		lidar_hoek = positie['Lidar_Motor'] + (hoek_scannen / 365) *2*np.pi
		if lidar_hoek > np.pi/2:
			lidar_hoek = 0
			landmark_berekeningen()
			global measured_points_x
			global measured_points_y
			map_measured_points_x.extend(measured_points_x)
			map_measured_points_y.extend(measured_points_y)
			measured_points_x = []
			measured_points_y = []
			move_lidar_motor(0)
			break
		move_lidar_motor(lidar_hoek)
		vrep.simxSynchronousTrigger(clientID)
	
def rijden():
	print('Rijden')
	n = 0
	while True:
		n += 1
		if n % 10 == 0:
			wielen_berekeningen()
			move_motor('linkermotor', 0.5)
			move_motor('rechtermotor', 1)
			vrep.simxSynchronousTrigger(clientID)
			if n % 1000 == 0:
				break

#parameters initializing
hoeken_scanners = {'Laser_Sensor':0,'Laser_Sensor0':np.pi/2,'Laser_Sensor1':-np.pi,'Laser_Sensor2':-np.pi/2}
hoek_scannen = 365/4/60 #altijd deelbaar door 365 en 4
omtrek_wiel = 0.08 * np.pi #omtrek wiel berekenen
Linker_Wiel_Oude_Positie = 0 #linker wiel voor begin loop
Rechter_Wiel_Oude_Positie = 0 #rechter wiel voor begin loop
afstand_wielen = 0.26 #afstand tussen wielen
correction_factor = 0.98 #Correctie voor afstand
rejection_distance = 99




#variables initializing
afgelegde_weg_x = [] #x-coördinaten afgelegd door robot
afgelegde_weg_y = [] #y-coördinaten afgelegd door robot
handle = {} #handle boek van onderdelen
positie = {} #positie van onderelen
rotation_difference = {} #verdraaiing verschil tov vorige stap
distance_difference = {} #afstand verschil tov vorige stap
locatie = [0,0,0] #x,y,teta (hoekverdraaiing)
rotatie_centrum = [0,0] #rotatie_centrum waar robot omheendraait in het kaart frame
measured_points_x = [] #gemeten punten x
measured_points_y = [] #gemeten punten y
map_measured_points_x = []
map_measured_points_y = []
lijn_x = [0,0] #lijn voor plotten, [[x1,x2],[y1,y2]]
lijn_y = [0,0]
modus = 0 #0=scannen op de plaats, 1 is rijden
meetpunten = {'Laser_Sensor':[],'Laser_Sensor0':[],'Laser_Sensor1':[],'Laser_Sensor2':[]} #gemeten afstanden, niet gesorteerd
gemeten_afstanden = [] #gesorteerde afstanden
landmark_ray_center = []
landmark_distance = []
landmark_heading = []
landmark_x_robot = []
landmark_y_robot = []
landmark_x_map = []
landmark_y_map = []
rejection_distance = 0.1


#get component handles
n = 0
while n < len(Robot_parts):
	get_handle(clientID,Robot_parts[n])
	n +=1
	
#map initializing
plt.ion()
plt.show()

position_motor('linkermotor')
position_motor('rechtermotor')
try:
	print('simulation started')
	while True:
		rond_scannen()
		rijden()

except KeyboardInterrupt:
	vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
	print('Simulation stopped')
	print('plots closed')
	sys.exit()

	

