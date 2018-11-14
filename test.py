import numpy as np
import vrep

vrep.simxFinish(-1) #alle open connecties sluiten
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) #v-rep verbinden
if clientID == -1: #visueel weergeven of verbinding is gelukt
	print('verbinding gefaald')
	exit()
else:
	print('verbinding gelukt')

errorCode,Linker_motor_handle = vrep.simxGetObjectHandle(clientID,'linkermotor',vrep.simx_opmode_blocking)
if errorCode > 0:
	print('linker motor geen handle')
else:
	print('linker motor heeft handle')

errorCode,Rechter_motor_handle = vrep.simxGetObjectHandle(clientID,'rechtermotor',vrep.simx_opmode_blocking)
if errorCode > 0:
	print('rechter motor geen handle')
else:
	print('rechter motor heeft handle')

errorCode,proximity_sensor_handle = vrep.simxGetObjectHandle(clientID,'proximity_sensor',vrep.simx_opmode_blocking)
if errorCode > 0:
	print('proximity sensor geen handle')
else:
	print('proximity sensor heeft handle')

errorCode,base_handle = vrep.simxGetObjectHandle(clientID,'Base',vrep.simx_opmode_blocking)
if errorCode > 0:
	print('base geen handle')
else:
	print('base heeft handle')

#initializatie voor de loop

#kaar maken 
#0 is niet gescand
#1 is gescand, niets te zien
#2 is gescand, iets in de weg
kaart_positie = [[1]]
kaart_grote_stappen = 0.05
richting = 0
Locatie = [0,0]
afgelegde_afstand = 0

#motoren aanzetten
vrep.simxSetJointTargetVelocity(clientID,Linker_motor_handle,0.25,vrep.simx_opmode_oneshot)	
vrep.simxSetJointTargetVelocity(clientID,Rechter_motor_handle,0.5,vrep.simx_opmode_oneshot)

#parameters initializeren
omtrek_wiel = 0.1*np.pi
Linker_Wiel_Oude_Positie = 0
Rechter_Wiel_Oude_Positie = 0
afstand_wielen = 0.26

try:
	while True:
		errorCode,prox_status,sensor_gemeten_afstand,gedetecteerde_handle,normaal_vector_oppervlakte_gedetecteerde=vrep.simxReadProximitySensor(clientID,proximity_sensor_handle,vrep.simx_opmode_blocking)
		errorCode,Linker_wiel_positie = vrep.simxGetJointPosition(clientID,Linker_motor_handle,vrep.simx_opmode_blocking)
		errorCode,Rechter_wiel_positie = vrep.simxGetJointPosition(clientID,Rechter_motor_handle,vrep.simx_opmode_blocking)
		errorCode,positie = vrep.simxGetObjectPosition(clientID,base_handle,-1,vrep.simx_opmode_blocking)
		errorCode,orrientatie = vrep.simxGetObjectOrientation(clientID,base_handle,-1,vrep.simx_opmode_blocking)
		
		#berekenen verdraaiing van robot
		#eerst berekenen verdraaiing van de wielen
		Linker_Wiel_Verdraaiing = -Linker_Wiel_Oude_Positie + Linker_wiel_positie
		Linker_Wiel_Oude_Positie = Linker_wiel_positie
		if Linker_Wiel_Verdraaiing < -5:
			Linker_Wiel_Verdraaiing = Linker_Wiel_Verdraaiing + 2*np.pi
			
		Rechter_Wiel_Verdraaiing = -Rechter_Wiel_Oude_Positie + Rechter_wiel_positie
		Rechter_Wiel_Oude_Positie = Rechter_wiel_positie
		if Rechter_Wiel_Verdraaiing < -5:
			Rechter_Wiel_Verdraaiing = Rechter_Wiel_Verdraaiing + 2*np.pi
		
		#berekenen verplaatsing van de wielen
		Rechter_Wiel_Afstand = (Rechter_Wiel_Verdraaiing*omtrek_wiel)/8
		Linker_Wiel_Afstand = (Linker_Wiel_Verdraaiing*omtrek_wiel)/8
		
		afgelegde_afstand = afgelegde_afstand + (Rechter_Wiel_Afstand + Linker_Wiel_Afstand)/2
		
		radius = ((Linker_Wiel_Afstand/Rechter_Wiel_Afstand)-1)*afstand_wielen*2*1.5
		
		if -0.001 < radius and radius < 0.001:
			radius = 0
			print('a')
		
		hoek = +(((Rechter_Wiel_Afstand+Linker_Wiel_Afstand)/2)/((radius + (afstand_wielen/2)) * 2 *np.pi))
		richting = richting + hoek	
			
		x_verplaatsing = np.cos(richting)*((Rechter_Wiel_Afstand+Linker_Wiel_Afstand)/2)
		y_verplaatsing = np.sin(richting)*((Rechter_Wiel_Afstand+Linker_Wiel_Afstand)/2)
		
		Locatie = [Locatie[0]+x_verplaatsing,Locatie[1]+y_verplaatsing]
		
		print(radius)
		
except KeyboardInterrupt:
	vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
	pass