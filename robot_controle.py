import vrep
import numpy as np

def move_motor(handle,part_name, speed,clientID):
	vrep.simxSetJointTargetVelocity(clientID,handle[part_name],speed,vrep.simx_opmode_oneshot)

def move_lidar_motor(handle,part,locatie,clientID):
	vrep.simxSetJointPosition(clientID, handle[part], locatie, vrep.simx_opmode_oneshot)

def position_motor(handle,part_name,clientID):
	position = {}
	i = 0
	while i < len(part_name):
		errorCode,positie = vrep.simxGetJointPosition(clientID,handle[part_name[i]],vrep.simx_opmode_blocking)
		if positie > 0:
			position[part_name[i]] = positie
		else:
			position[part_name[i]] = positie + np.pi
		i+=1
	return position

def position_lidar_motor(handle,part,clientID):
	errorCode,position = vrep.simxGetJointPosition(clientID,handle[part],vrep.simx_opmode_blocking)
	return position

def distance_sensor(handle,sensors,clientID,distance):
	i = 0
	while i < len(sensors):
		errorCode,prox_status,prox_mesured_distance,detected_handle,normal_vector=vrep.simxReadProximitySensor(clientID,handle[sensors[i]],vrep.simx_opmode_blocking)
		distance[sensors[i]].append(prox_mesured_distance[2])
		i +=1
	return distance
	
def get_position(handle,clientID,part):
	return vrep.simxGetObjectPosition(clientID,handle[part],-1,vrep.simx_opmode_blocking)

def get_orrientation(handle,clientID,part):
	return vrep.simxGetObjectOrientation(clientID,handle[part],-1,vrep.simx_opmode_blocking)
	