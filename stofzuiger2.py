from startup import *
from robot_controle import *
from driving_calc import *
from slam import *
import numpy as np
import sys

#robot parameters				
Robot_parts = ['linkermotor','rechtermotor','Laser_Sensor','Laser_Sensor0','Laser_Sensor1','Laser_Sensor2','Lidar_Motor']				
motors = ['linkermotor','rechtermotor']
sensors = ['Laser_Sensor','Laser_Sensor0','Laser_Sensor1','Laser_Sensor2']
omtrek_wiel = 0.08 * np.pi #omtrek wiel berekenen
afstand_wielen = 0.26 #afstand tussen wielen
correctiefactor = 0.95 #correctiefactor voor de afgelegde afstand in driving_calc
scan_hoek = 365/4/60 #altijd deelbaar door 365 en 4

#startup parameters
locatie = [0,0,0] #startlocatie robot x,y,theta
lidar_positie = []
distance = []

def start_up(Robot_parts):		
	clientID = make_connection()
	handle = get_handle(clientID,Robot_parts)
	motor_position = position_motor(handle,motors,clientID)
	vrep.simxSynchronousTrigger(clientID)
	return motor_position,clientID,handle

def driving(locatie,omtrek_wiel,afstand_wielen,Robot_parts, motors, old_motor_position, correctiefactor, handle, clientID):
	move_motor(handle,'linkermotor',0.5,clientID)
	move_motor(handle,'rechtermotor',1,clientID)
	new_motor_position = position_motor(handle,motors,clientID)
	locatie,positie = drive_calc(new_motor_position,old_motor_position,motors,omtrek_wiel,afstand_wielen,locatie,correctiefactor)
	return locatie, positie
	
def scanning(handle,clientID,scan_hoek,position,sensors,distance):
	move_motor(handle,'linkermotor',0,clientID)
	move_motor(handle,'rechtermotor',0,clientID)
	position.append(position_lidar_motor(handle,'Lidar_Motor',clientID))
	position[-1] += (scan_hoek/365)*2*np.pi
	if position[-1] > np.pi/2:
		slam(position)
	distance.append(distance_sensor(handle,sensors,clientID))
	move_lidar_motor(handle,'Lidar_Motor',position,clientID)
	return position,distance
	

#start-up
old_motor_position,clientID,handle = start_up(Robot_parts)

#drive
try:
	while True:
		#locatie,old_motor_position = driving(locatie,omtrek_wiel,afstand_wielen,Robot_parts, motors,old_motor_position, correctiefactor, handle, clientID)
		lidar_positie,distance = scanning(handle,clientID,scan_hoek,lidar_positie,sensors,distance)
		vrep.simxSynchronousTrigger(clientID)

except KeyboardInterrupt:
	vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
	print('Simulation stopped')
	sys.exit()