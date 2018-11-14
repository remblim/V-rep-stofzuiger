import numpy as np
import vrep
import sys
import matplotlib.pyplot as plt
import time

Robot_parts = ['linkermotor','rechtermotor','Laser_Sensor','Lidar_Motor']

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
			
		rotatie_centrum[0]=locatie[0]-(robot_radius)*np.cos(locatie[2]) #x
		rotatie_centrum[1]=locatie[1]-(robot_radius)*np.sin(locatie[2]) #y
		locatie[2] = (locatie[2] + alpha)%(2*np.pi)
		locatie[0] = rotatie_centrum[0] + np.cos(locatie[2]) * (robot_radius)
		locatie[1] = rotatie_centrum[1] + np.sin(locatie[2]) * (robot_radius)
		afgelegde_weg_x.append(locatie[0])
		afgelegde_weg_y.append(locatie[1])
		
def proximity_sensor_calcultion(part_name):
	errorCode,prox_status,prox_mesured_distance,detected_handle,normal_vector = distance_sensor(part_name)
	
	if prox_status == True:
		sensor_location_x = 0.15 * np.sin(locatie[2])
		sensor_location_y = 0.15 * np.cos(locatie[2])
		sensor_heading = locatie[2] + positie['Lidar_Motor']
		measured_points_x.append((sensor_location_x - prox_mesured_distance[2] * np.sin(sensor_heading)))
		measured_points_y.append((sensor_location_y + prox_mesured_distance[2] * np.cos(sensor_heading)))

def connecting_points():
	if len(measured_points_x) > 2:
		x_aveg, y_aveg = np.mean(measured_points_x), np.mean(measured_points_y)
		x_min, y_min = measured_points_x - x_aveg, measured_points_y - y_aveg
		Xmin, Xmax = np.min(measured_points_x), np.max(measured_points_x)
		m = np.sum(x_min*y_min)/np.sum(x_min**2)
		b = y_aveg - m * x_aveg
		lijn_x[0], lijn_x[1] = np.min(measured_points_x), np.max(measured_points_x)
		lijn_y[0], lijn_y[1] = b + m * lijn_x[0], b + m * lijn_x[1]

def visual_map_making():
	plt.gcf().clear()
	if not lijn_x[0] == 0:
		plt.plot(lijn_x, lijn_y, color='yellow')
	plt.plot(afgelegde_weg_x, afgelegde_weg_y,color='red')
	plt.scatter(measured_points_x,measured_points_y,color='green')
	plt.axis('equal')
	plt.draw()
	plt.pause(0.001)
	
#onderdelen aansturen
def move_motor(part_name, speed):
	vrep.simxSetJointTargetVelocity(clientID,handle[part_name],speed,vrep.simx_opmode_oneshot)

def move_lidar_motor(part_name, location):
	vrep.simxSetJointPosition(clientID, handle[part_name],location , vrep.simx_opmode_oneshot)
		
#beweeg modusen
def rond_scannen():
	global scannen_stappen
	move_motor('linkermotor', 0.0)
	move_motor('rechtermotor', 0.0)
	lidar_hoek = positie['Lidar_Motor'] + (hoek_scannen / 365) *2*np.pi 
	scannen_stappen = scannen_stappen + 1
	if scannen_stappen == 365/hoek_scannen:
		modus = 1
		lidar_hoek = 0
		scannen_stappen = 0
	print(scannen_stappen)	
	move_lidar_motor('Lidar_Motor', lidar_hoek)

def rijden():
	print('rijden')
	move_motor('linkermotor', 0.3)
	move_motor('rechtermotor', 0.6)
	
#parameters initializing
hoek_scannen = 3.65 #altijd deelbaar door 365
omtrek_wiel = 0.08 * np.pi #omtrek wiel berekenen
Linker_Wiel_Oude_Positie = 0 #linker wiel voor begin loop
Rechter_Wiel_Oude_Positie = 0 #rechter wiel voor begin loop
afstand_wielen = 0.26 #afstand tussen wielen
afgelegde_weg_x = [] #x-coördinaten afgelegd door robot
afgelegde_weg_y = [] #y-coördinaten afgelegd door robot
handle = {} #handle boek van onderdelen
positie = {} #positie van onderelen
rotation_difference = {} #verdraaiing verschil tov vorige stap
distance_difference = {} #afstand verschil tov vorige stap
locatie = [0,0,0] #x,y,teta (hoekverdraaiing)
rotatie_centrum = [0,0] #rotatie_centrum waar robot omheendraait
measured_points_x = [] #gemeten punten x
measured_points_y = [] #gemeten punten y
correction_factor = 0.98 #Correctie voor afstand
lijn_x = [0,0] #lijn voor plotten, [[x1,x2],[y1,y2]]
lijn_y = [0,0]
modus = 0 #0=scannen op de plaats, 1 is rijden
scannen_stappen = 0

#get component handles
n = 0
while n < len(Robot_parts):
	get_handle(clientID,Robot_parts[n])
	n +=1
	
#map initializing
plt.ion()
plt.show()

n = 0
position_motor('linkermotor')
position_motor('rechtermotor')
try:
	print('simulation started')
	while True:
		if n % 10 == 0:
			rotation_calculation('linkermotor')
			rotation_calculation('rechtermotor')
			position_lidar_motor('Lidar_Motor')
			position_change_calculation()
			proximity_sensor_calcultion('Laser_Sensor')
			#connecting_points()
			if n % 300 == 0:
				visual_map_making()
			if modus == 0:
				rond_scannen()
			if modus == 0:
				rijden()
			vrep.simxSynchronousTrigger(clientID)
		n+=1

except KeyboardInterrupt:
	vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
	print('Simulation stopped')
	print('plots closed')
	sys.exit()

