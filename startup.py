import vrep

def make_connection():
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
		return clientID
			
def get_handle (clientID,part_name):
	handle = {}
	i=0
	while i < len(part_name):
		errorCode,handle[part_name[i]] = vrep.simxGetObjectHandle(clientID,part_name[i],vrep.simx_opmode_blocking)
		if errorCode > 0:
			print(part_name[i] + ' has no handle')
			exit()
		else:
			print(part_name[i] + ' has handle')
		i+=1
	return handle