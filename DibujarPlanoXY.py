import numpy as np
import cv2
import sim
import time
import robotic_library as rl
import mover as mo

#Parametros robot
d1=11
a2=30
a3=30
d6=14

#Para un espacio de trabajo en el plano xy
T0w =  np.array([[0, 1, 0, 22],
                 [-1, 0, 0, 45],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

#Eje z de la herramienta perpendicular al plano de trabajo
Tmh = np.array([[-1, 0, 0, 0],
                [0, 0, 1, -10],
                [0, 1, 0, 0],
                [0, 0, 0, 1]])

#Obtener contornos a dibujar
imagen = cv2.imread('xy3.png')
gray = cv2.cvtColor(imagen,cv2.COLOR_BGR2GRAY)
_,th = cv2.threshold(gray,100,255,cv2.THRESH_BINARY)

contornos,hierarchy = cv2.findContours(th, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

#contruyendo las diferentes Twh
for j in range(0,len(contornos)-1):
    for i in range(0,len(contornos[j])):
        #print("x: " + str(round(contornos[j][i][0][0]/3,2)) + "       " + "y: " + str(round(contornos[j][i][0][1]/3,2)))
        Twh = np.array([[1, 0, 0, contornos[j][i][0][0]/3],
                        [0, 1, 0, contornos[j][i][0][1]/3],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

        #Obtener matriz T0m
        T0m = np.dot(T0w,np.dot(Twh,np.linalg.inv(Tmh)))
        P = rl.CineInvRob1(d1,a2,a3,d6,T0m)
        
        #Enviar angulos al robot
        mo.env_ang(P[2],T0w)

        if (i == 0):
            time.sleep(1) 
        else:
            time.sleep(0.1)
    
    Twh = np.array([[1, 0, 0, 45],
                        [0, 1, 0, 15],
                        [0, 0, 1, 10],
                        [0, 0, 0, 1]])

    #Obtener matriz T0m
    T0m = np.dot(T0w,np.dot(Twh,np.linalg.inv(Tmh)))
    P = rl.CineInvRob1(d1,a2,a3,d6,T0m)
       
    #Enviar angulos al robot
    mo.env_ang(P[2],T0w)

    time.sleep(1) 