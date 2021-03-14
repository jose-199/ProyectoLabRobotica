import sim
import numpy as np

def connect(port):
# Establece la conexión a VREP
# port debe coincidir con el puerto de conexión en VREP
# retorna el número de cliente o -1 si no puede establecer conexión
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',port,True,True,2000,5) # Conectarse
    if clientID == 0: print("conectado a", port)
    else: print("no se pudo conectar")
    return clientID


# Conectarse al servidor de VREP
# *** ejecutar cada vez que se reinicia la simulación ***
clientID = connect(19999)

#  Obtener los manejadores (handlers)
retCode,tip=sim.simxGetObjectHandle(clientID,'suctionPadSensor',sim.simx_opmode_blocking)
retCode,joint1=sim.simxGetObjectHandle(clientID,'joint1',sim.simx_opmode_blocking)
retCode,joint2=sim.simxGetObjectHandle(clientID,'joint2',sim.simx_opmode_blocking)
retCode,joint3=sim.simxGetObjectHandle(clientID,'joint3',sim.simx_opmode_blocking)
retCode,joint4=sim.simxGetObjectHandle(clientID,'joint4',sim.simx_opmode_blocking)
retCode,joint5=sim.simxGetObjectHandle(clientID,'joint5',sim.simx_opmode_blocking)
print(tip, joint1, joint2, joint3, joint4, joint5)

# angulos de las articulaciones

j1 = 45    #ángulo joint 1 
j2 = 60    #ángulo joint 2
j3 = 30    #ángulo joint 3
j4 = -20    #ángulo joint 4
j5 = 90    #ángulo joint 5

q = [j1 * np.pi/180, j2 * np.pi/180, j3 * np.pi/180, j4 * np.pi/180, j5 * np.pi/180]

# enviamos los ángulos a las articulaciones
clientID = connect(19999)
retCode = sim.simxSetJointTargetPosition(clientID, joint1, q[0], sim.simx_opmode_oneshot)
retCode = sim.simxSetJointTargetPosition(clientID, joint2, q[1], sim.simx_opmode_oneshot)
retCode = sim.simxSetJointTargetPosition(clientID, joint3, q[2], sim.simx_opmode_oneshot)
retCode = sim.simxSetJointTargetPosition(clientID, joint4, q[3], sim.simx_opmode_oneshot)
retCode = sim.simxSetJointTargetPosition(clientID, joint5, q[4], sim.simx_opmode_oneshot)