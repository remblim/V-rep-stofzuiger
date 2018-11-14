import numpy as np


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
			print(np.argmin(kolom))
			if np.min(kolom) < rejection_distance:
				map_x.append(landmark_x_map[n])
				map_y.append(landmark_y_map[n])
				robot_x.append(landmark_x_robot[np.argmin(kolom)])
				robot_y.append(landmark_y_robot[np.argmin(kolom)])
			n=n+1
		landmark_x_robot = []
		landmark_y_robot = []
	print(map_x,map_y,robot_x,robot_y)
	
robot_x = [1,2,3,4]
robot_y = [5,6,7,8]
map_x = [2,4,1,3]
map_y = [6,8,5,7]
rejection_distance = 10
landmark_comparison(robot_x,robot_y,map_x,map_y,rejection_distance)