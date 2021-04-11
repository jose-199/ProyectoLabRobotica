import sim
import numpy as np
import time

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
retCode,tip=sim.simxGetObjectHandle(clientID,'Dummy',sim.simx_opmode_blocking)
retCode,joint1=sim.simxGetObjectHandle(clientID,'joint1',sim.simx_opmode_blocking)
retCode,joint2=sim.simxGetObjectHandle(clientID,'joint2',sim.simx_opmode_blocking)
retCode,joint3=sim.simxGetObjectHandle(clientID,'joint3',sim.simx_opmode_blocking)
retCode,joint4=sim.simxGetObjectHandle(clientID,'joint4',sim.simx_opmode_blocking)
retCode,joint5=sim.simxGetObjectHandle(clientID,'joint5',sim.simx_opmode_blocking)
retCode,joint6=sim.simxGetObjectHandle(clientID,'joint6',sim.simx_opmode_blocking)
print(tip, joint1, joint2, joint3, joint4, joint5, joint6)

# angulos de las articulaciones

#j1 = 45    #ángulo joint 1 
#j2 = 60    #ángulo joint 2
#j3 = 30    #ángulo joint 3
#j4 = -20    #ángulo joint 4
#j5 = 90    #ángulo joint 5
#j5 = 90    #ángulo joint 5

#j = [25.6410,60.3874,-124.9053,64.5179,(154.3590),0]
#j = [2.3749e+01, 6.0805e+01, -1.2580e+02, 6.4998e+01, 1.5625e+02, 0]  # (5,5)
#j = [34.0428, 46.0533, -94.9319, 48.8787, 145.9572, 0]  # (13.6, 16.7)
#j = [9.0000e+01, 7.4719e+01, -1.6836e+02, 9.3644e+01, 9.0000e+01, 0]  # (-20, 0)
#j = [-3.4992e+01, 6.3624e+01, -1.3194e+02, -1.1169e+02, 1.4501e+02, 1.8000e+02] # (0, -20)
#j = [ -5.9534e+01, 4.7427e+01, -9.7759e+01, -1.2967e+02, 1.2047e+02, -1.8000e+02] # (-40, 0)
#j = [1.6699e+01, 6.6868e+01, -1.3922e+02, 7.2352e+01, 1.6330e+02, 2.7672e-15] # (0, 0)
#j = [ -2.9249e+01,   5.9457e+01,  -1.2291e+02,  -1.1655e+02,   1.5075e+02,   1.8000e+02] # (5, -20)

#q = [j[0] * np.pi/180, j[1] * np.pi/180, j[2] * np.pi/180, j[3] * np.pi/180, j[4] * np.pi/180, j[5] * np.pi/180]

# enviamos los ángulos a las articulaciones
#clientID = connect(19999)

S = np.array([0.1,0.2,0.3,0.4,0.5,0.6])

def env_ang(P, T0w):
    retCode = sim.simxSetJointTargetPosition(clientID, joint1, P[0], sim.simx_opmode_oneshot)
    retCode = sim.simxSetJointTargetPosition(clientID, joint2, P[1], sim.simx_opmode_oneshot)
    retCode = sim.simxSetJointTargetPosition(clientID, joint3, P[2], sim.simx_opmode_oneshot)
    retCode = sim.simxSetJointTargetPosition(clientID, joint4, P[3], sim.simx_opmode_oneshot)
    retCode = sim.simxSetJointTargetPosition(clientID, joint5, P[4], sim.simx_opmode_oneshot)
    retCode = sim.simxSetJointTargetPosition(clientID, joint6, P[5], sim.simx_opmode_oneshot)

    retCode,pos=sim.simxGetObjectPosition(clientID, tip, -1, sim.simx_opmode_blocking)
    P0 = np.array([[pos[0]*100],[pos[1]*100],[pos[2]*100],[1]])
    Pw = np.dot(np.linalg.inv(T0w),P0)
    print("Sistema global  ->        " + "x: " + str(round(pos[0]*100,2)) + "       " + "y: " + str(round(pos[1]*100,2)) + "       " + "z: " + str(round(pos[2]*100,2)))
    print("Sistema local w ->        " + "x: " + str(round(Pw[0][0],2)) + "       " + "y: " + str(round(Pw[1][0],2)) + "       " + "z: " + str(round(Pw[2][0],2)))