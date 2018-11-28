from startup import *
from robot_controle import *
from driving_calc import *
from slam import Slam
import numpy as np
import sys
from matplotlib import pyplot as plt

class Robot:
	def __init__(self):
		#robot parameters
		self.omtrek_wiel = 0.08 * np.pi
		self.afstand_wielen = 0.26
		self.correctiefactor = 0.95
		self.scan_hoek = 365/4/60
		self.Robot_parts = ['linkermotor','rechtermotor','Laser_Sensor','Laser_Sensor0','Laser_Sensor1','Laser_Sensor2','Lidar_Motor','Dummy']
		self.motors = ['linkermotor','rechtermotor']
		self.sensors = ['Laser_Sensor','Laser_Sensor0','Laser_Sensor1','Laser_Sensor2']
		self.rejection_distance = 0.2
		
		#initialize slam
		self.ObjSlam = Slam()
		
		#startup parameters
		self.modus = 0 	#0 is ronde scannen
						#1 is stukje rijden
		self.ticks = 0
		self.locatie = [0,0,0]
		self.distance = {'Laser_Sensor':[],'Laser_Sensor0':[],'Laser_Sensor1':[],'Laser_Sensor2':[]}
		self.map_landmark = []
		self.geschiedenis_locatie_x = []
		self.geschiedenis_locatie_y = []
		self.slam_position = [0,0,0]
		self.real_position = [0,0]
		
		self.clientID = make_connection()
		self.handle = get_handle(self.clientID,self.Robot_parts)
		self.old_motor_position = position_motor(self.handle,self.motors,self.clientID)
		vrep.simxSynchronousTrigger(self.clientID)
		plt.ion()
		plt.show()

	def pos_calculate(self):
		self.motor_position = position_motor(self.handle,self.motors,self.clientID)
		self.locatie,self.old_motor_position=drive_calc(self.motor_position,self.old_motor_position,self.motors,self.omtrek_wiel,self.afstand_wielen,self.locatie,self.correctiefactor)
		self.geschiedenis_locatie_x.append(self.locatie[0])
		self.geschiedenis_locatie_y.append(self.locatie[1])
		
		#echte positie ophalen uit vrep, eerst rotatie dan positie
		orrientatie = get_orrientation(self.handle,self.clientID,'Dummy')
		self.real_position = get_position(self.handle,self.clientID,'Dummy')
		
		
	def drive(self):
		self.driving(3,6)
		self.ticks += 1
		self.pos_calculate()
		ObjRobot.plotting()
		if self.ticks > 20:
			self.modus = 0
			self.ticks = 1
		
	def driving(self,left_motor,right_motor):
		move_motor(self.handle,'linkermotor',left_motor,self.clientID)
		move_motor(self.handle,'rechtermotor',right_motor,self.clientID)
	
	def scanning(self):
		self.driving(0,0)
		lidar_positie = position_lidar_motor(self.handle,'Lidar_Motor',self.clientID)
		lidar_positie += (self.scan_hoek/365)*2*np.pi
		self.distance = distance_sensor(self.handle,self.sensors,self.clientID,self.distance)
		if lidar_positie > np.pi/2:
			lidar_positie = 0
			self.measured_points_x,self.measured_points_y,self.map_x,self.map_y,self.robot_map_x,self.robot_map_y,self.new_location = self.ObjSlam.calculations(self.distance,self.locatie)
			real_locatie = get_position(self.handle,self.clientID,'Dummy')
			ObjRobot.plotting()
			self.distance = {'Laser_Sensor':[],'Laser_Sensor0':[],'Laser_Sensor1':[],'Laser_Sensor2':[]}
			self.modus = 1
		move_lidar_motor(self.handle,'Lidar_Motor',lidar_positie,self.clientID)
		
	def plotting(self):
		#rood is berekende locatie
		#groen is werkelijke locatie
		plt.gcf().clear()
		plt.scatter(self.locatie[0],self.locatie[1],color='red')
		if self.ticks > 0:
			plt.scatter(self.real_position[1][1],-self.real_position[1][0],color='green')
		if self.new_location is not False:
			plt.scatter(self.new_location[0],self.new_location[1],color='orange')		
		plt.scatter(self.measured_points_x,self.measured_points_y,color='purple')
		plt.scatter(self.robot_map_x,self.robot_map_y,color='blue')
		plt.scatter(self.map_x,self.map_y,color='yellow')
		plt.axis('equal')
		plt.draw
		plt.pause(0.001)
		
		
ObjRobot = Robot()


try:
	while True:
		if ObjRobot.modus == 0:
			ObjRobot.scanning()
		if ObjRobot.modus == 1:
			ObjRobot.drive()
		vrep.simxSynchronousTrigger(ObjRobot.clientID)
		
except KeyboardInterrupt:
	vrep.simxStopSimulation(ObjRobot.clientID,vrep.simx_opmode_oneshot)
	print('Simulation stopped')
	sys.exit()