import numpy as np

def drive_calc(positie,oude_positie,motor_part,omtrek_wiel,afstand_wielen,locatie,correction_factor):
	i = 0
	rotation_difference = {}
	while i < len(motor_part):
		rotation_difference[motor_part[i]] = positie[motor_part[i]] - oude_positie[motor_part[i]]
		if rotation_difference[motor_part[i]] < -2:
			rotation_difference[motor_part[i]] += np.pi
		if rotation_difference [motor_part[i]] < 0.001 or rotation_difference[motor_part[i]] > np.pi -0.001:
			rotation_difference[motor_part[i]] = 0
		i += 1
		
	#locatie_verandering berekeningen
	distance_difference = {}
	distance_difference['linkermotor'] = (rotation_difference['linkermotor']*correction_factor/(np.pi*2))*(omtrek_wiel)
	distance_difference['rechtermotor'] = (rotation_difference['rechtermotor']*correction_factor/(np.pi*2))*(omtrek_wiel)
	
	
	alpha = (distance_difference['rechtermotor']-distance_difference['linkermotor'])/afstand_wielen
	if alpha == 0:
		locatie[0] = locatie[0] + distance_difference['linkermotor']*np.sin(locatie[2])
		locatie[1] = locatie[1] - distance_difference['linkermotor']*np.cos(locatie[2])
	else:
		if distance_difference['linkermotor']<distance_difference['rechtermotor']:
			radius = distance_difference['linkermotor']/alpha
		else:
			radius = distance_difference['rechtermotor']/alpha
			
		if radius > 0:
			robot_radius = radius + afstand_wielen/2
		else:
			robot_radius = radius - afstand_wielen/2
		rotatie_centrum = [0,0]
		rotatie_centrum[0]=locatie[0]-(robot_radius)*np.cos(locatie[2]) #x
		rotatie_centrum[1]=locatie[1]-(robot_radius)*np.sin(locatie[2]) #y
		locatie[2] = (locatie[2] + alpha)%(2*np.pi)
		locatie[0] = rotatie_centrum[0] + np.cos(locatie[2]) * (robot_radius)
		locatie[1] = rotatie_centrum[1] + np.sin(locatie[2]) * (robot_radius)
	return locatie,positie