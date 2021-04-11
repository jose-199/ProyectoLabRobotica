import numpy as np

def Rx(angle):
    R = np.array([[1, 0, 0, 0],
                   [0, np.cos(angle), -np.sin(angle), 0],
                   [0, np.sin(angle), np.cos(angle), 0],
                   [0, 0, 0, 1]])
    return R

def Ry(angle):
    R = np.array([[np.cos(angle), 0, np.sin(angle), 0],
                   [0, 1, 0, 0],
                   [-np.sin(angle), np.cos(angle), 0, 0],
                   [0, 0, 0, 1]])
    return R

def Rz(angle):
    R = np.array([[np.cos(angle), -np.sin(angle), 0, 0],
                   [np.sin(angle), np.cos(angle), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    return R

def Tx(dist):
    R = np.array([[1, 0, 0, dist],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    return R

def Ty(dist):
    R = np.array([[1, 0, 0, 0],
                  [0, 1, 0, dist],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    return R

def Tz(dist):
    R = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, dist],
                  [0, 0, 0, 1]])
    return R

def DH(a,alfa,d,theta):
#    R = np.dot(Tz(d), Rz(theta))
#    R = np.dot(R, Rx(alfa))
#    R = np.dot(R, Tx(a))


    R = np.dot(np.dot(np.dot(Tz(d),Rz(theta)),Rx(alfa)),Tx(a))
    return R

def CineDirecRob1(d1,a2,a3,d6,angles):        #cinematica directa para el robot hecho en clase
    A1 = DH(0,np.pi/2,d1,angles[0])
    A2 = DH(a2,0,0,angles[1])
    A3 = DH(a3,0,0,angles[2])
    A4 = DH(0,-np.pi/2,0,angles[3])
    A5 = DH(0,np.pi/2,0,angles[4])
    A6 = DH(0,0,d6,angles[5])

    T1 = A1
    T2 = np.dot(A1,A2)
    T3 = np.dot(T2,A3)
    T4 = np.dot(T3,A4)
    T5 = np.dot(T4,A5)
    T6 = np.dot(T5,A6)

    T = np.array([T1,T2,T3,T4,T5,T6])

    return T

def CineInvRob1(d1,a2,a3,d6,T):

    theta1 = np.array([])
    theta2 = np.array([])
    theta3 = np.array([])
    theta4 = np.array([])
    theta5 = np.array([])
    theta6 = np.array([])

    Cmu = np.dot(T,np.array([[0],[0],[-d6],[1]]))

    #--------------------------------------------------------------------
    #-------------------------Grupo 1 de ángulos-------------------------
    #     xc, yc, sqrt(1-C3^2), +Euler

    # Encontrando theta 1
    
    theta1 = np.append(theta1, np.arctan2(Cmu[1][0],Cmu[0][0]))

    # Encontrando theta 3
    C3 = (np.power(Cmu[0][0],2)+np.power(Cmu[1][0],2)+np.power((Cmu[2][0]-d1),2)-np.power(a2,2)-np.power(a3,2))/(2*a2*a3)
    theta3 = np.append(theta3,np.arctan2(np.sqrt(1-np.power(C3,2)),C3))

    # Encontrando theta 2
    theta2 = np.append(theta2, np.arctan2(Cmu[2][0]-d1,np.sqrt(np.power(Cmu[0][0],2)+np.power(Cmu[1][0],2)))-np.arctan2(a3*np.sin(theta3[0]),a2+a3*C3))

    # Encontrando theta 4,5 y 6
    A1 = DH(0,np.pi/2,d1,theta1[0])
    A2 = DH(a2,0,0,theta2[0]);
    A3 = DH(a3,0,0,theta3[0]);
    T_Euler = np.dot(np.linalg.inv(np.dot(np.dot(A1,A2),A3)),T)
    sin_th = np.sqrt(1-np.power(T_Euler[2][2],2))

    if (sin_th < 1e-6):
        if (T_Euler[2][2] > 0):
            theta4 = np.append(theta4, 0)
            theta5 = np.append(theta5, 0)
            theta6 = np.append(theta6, np.arctan2(-T_Euler[0][1],T_Euler[0][0]))
        else:
            theta4 = np.append(theta4,0)
            theta5 = np.append(theta5, np.pi)
            theta6 = np.append(theta6, np.arctan2(T_Euler[0][1],-T_Euler[0][0]))
    else:
        theta4 = np.append(theta4, np.arctan2(T_Euler[1,2],T_Euler[0,2]))
        theta5 = np.append(theta5, np.arctan2(np.sqrt(1-np.power(T_Euler[2,2],2)),T_Euler[2,2]))
        theta6 = np.append(theta6, np.arctan2(T_Euler[2,1],-T_Euler[2,0]))

    
    #--------------------------------------------------------------------
    #-------------------------Grupo 2 de ángulos-------------------------
    #     xc, yc, sqrt(1-C3^2), -Euler

    # Encontrando theta 1
    
    theta1 = np.append(theta1, np.arctan2(Cmu[1][0],Cmu[0][0]))

    # Encontrando theta 3
    C3 = (np.power(Cmu[0][0],2)+np.power(Cmu[1][0],2)+np.power((Cmu[2][0]-d1),2)-np.power(a2,2)-np.power(a3,2))/(2*a2*a3)
    theta3 = np.append(theta3,np.arctan2(np.sqrt(1-np.power(C3,2)),C3))

    # Encontrando theta 2
    theta2 = np.append(theta2, np.arctan2(Cmu[2][0]-d1,np.sqrt(np.power(Cmu[0][0],2)+np.power(Cmu[1][0],2)))-np.arctan2(a3*np.sin(theta3[1]),a2+a3*C3))

    # Encontrando theta 4,5 y 6
    A1 = DH(0,np.pi/2,d1,theta1[1])
    A2 = DH(a2,0,0,theta2[1]);
    A3 = DH(a3,0,0,theta3[1]);
    T_Euler = np.dot(np.linalg.inv(np.dot(np.dot(A1,A2),A3)),T)
    sin_th = np.sqrt(1-np.power(T_Euler[2][2],2))

    if (sin_th < 1e-6):
        if (T_Euler[2][2] > 0):
            theta4 = np.append(theta4, 0)
            theta5 = np.append(theta5, 0)
            theta6 = np.append(theta6, np.arctan2(-T_Euler[0][1],T_Euler[0][0]))
        else:
            theta4 = np.append(theta4,0)
            theta5 = np.append(theta5, np.pi)
            theta6 = np.append(theta6, np.arctan2(T_Euler[0][1],-T_Euler[0][0]))
    else:
        theta4 = np.append(theta4, np.arctan2(-T_Euler[1,2],-T_Euler[0,2]))
        theta5 = np.append(theta5, np.arctan2(-np.sqrt(1-np.power(T_Euler[2,2],2)),T_Euler[2,2]))
        theta6 = np.append(theta6, np.arctan2(-T_Euler[2,1],T_Euler[2,0]))
 
    P = np.array([[theta1[0], theta2[0], theta3[0], theta4[0], theta5[0], theta6[0]],
                 [theta1[1], theta2[1], theta3[1], theta4[1], theta5[1], theta6[1]]])



    #--------------------------------------------------------------------
    #-------------------------Grupo 3 de ángulos-------------------------
    #    xc, yc, -sqrt(1-C3^2), +Euler

    # Encontrando theta 1
    
    theta1 = np.append(theta1, np.arctan2(Cmu[1][0],Cmu[0][0]))

    # Encontrando theta 3
    C3 = (np.power(Cmu[0][0],2)+np.power(Cmu[1][0],2)+np.power((Cmu[2][0]-d1),2)-np.power(a2,2)-np.power(a3,2))/(2*a2*a3)
    theta3 = np.append(theta3,np.arctan2(-np.sqrt(1-np.power(C3,2)),C3))

    # Encontrando theta 2
    theta2 = np.append(theta2, np.arctan2(Cmu[2][0]-d1,np.sqrt(np.power(Cmu[0][0],2)+np.power(Cmu[1][0],2)))-np.arctan2(a3*np.sin(theta3[2]),a2+a3*C3))

    # Encontrando theta 4,5 y 6
    A1 = DH(0,np.pi/2,d1,theta1[2])
    A2 = DH(a2,0,0,theta2[2]);
    A3 = DH(a3,0,0,theta3[2]);
    T_Euler = np.dot(np.linalg.inv(np.dot(np.dot(A1,A2),A3)),T)
    sin_th = np.sqrt(1-np.power(T_Euler[2][2],2))

    if (sin_th < 1e-6):
        if (T_Euler[2][2] > 0):
            theta4 = np.append(theta4, 0)
            theta5 = np.append(theta5, 0)
            theta6 = np.append(theta6, np.arctan2(-T_Euler[0][1],T_Euler[0][0]))
        else:
            theta4 = np.append(theta4,0)
            theta5 = np.append(theta5, np.pi)
            theta6 = np.append(theta6, np.arctan2(T_Euler[0][1],-T_Euler[0][0]))
    else:
        theta4 = np.append(theta4, np.arctan2(T_Euler[1,2],T_Euler[0,2]))
        theta5 = np.append(theta5, np.arctan2(np.sqrt(1-np.power(T_Euler[2,2],2)),T_Euler[2,2]))
        theta6 = np.append(theta6, np.arctan2(T_Euler[2,1],-T_Euler[2,0]))
 
    P = np.array([[theta1[0], theta2[0], theta3[0], theta4[0], theta5[0], theta6[0]],
                  [theta1[1], theta2[1], theta3[1], theta4[1], theta5[1], theta6[1]],
                  [theta1[2], theta2[2], theta3[2], theta4[2], theta5[2], theta6[2]]])



    #--------------------------------------------------------------------
    #-------------------------Grupo 4 de ángulos-------------------------
    #    xc, yc, -sqrt(1-C3^2), -Euler

    # Encontrando theta 1
    
    theta1 = np.append(theta1, np.arctan2(Cmu[1][0],Cmu[0][0]))

    # Encontrando theta 3
    C3 = (np.power(Cmu[0][0],2)+np.power(Cmu[1][0],2)+np.power((Cmu[2][0]-d1),2)-np.power(a2,2)-np.power(a3,2))/(2*a2*a3)
    theta3 = np.append(theta3,np.arctan2(-np.sqrt(1-np.power(C3,2)),C3))

    # Encontrando theta 2
    theta2 = np.append(theta2, np.arctan2(Cmu[2][0]-d1,np.sqrt(np.power(Cmu[0][0],2)+np.power(Cmu[1][0],2)))-np.arctan2(a3*np.sin(theta3[3]),a2+a3*C3))

    # Encontrando theta 4,5 y 6
    A1 = DH(0,np.pi/2,d1,theta1[3])
    A2 = DH(a2,0,0,theta2[3]);
    A3 = DH(a3,0,0,theta3[3]);
    T_Euler = np.dot(np.linalg.inv(np.dot(np.dot(A1,A2),A3)),T)
    sin_th = np.sqrt(1-np.power(T_Euler[2][2],2))

    if (sin_th < 1e-6):
        if (T_Euler[2][2] > 0):
            theta4 = np.append(theta4, 0)
            theta5 = np.append(theta5, 0)
            theta6 = np.append(theta6, np.arctan2(-T_Euler[0][1],T_Euler[0][0]))
        else:
            theta4 = np.append(theta4,0)
            theta5 = np.append(theta5, np.pi)
            theta6 = np.append(theta6, np.arctan2(T_Euler[0][1],-T_Euler[0][0]))
    else:
        theta4 = np.append(theta4, np.arctan2(-T_Euler[1,2],-T_Euler[0,2]))
        theta5 = np.append(theta5, np.arctan2(-np.sqrt(1-np.power(T_Euler[2,2],2)),T_Euler[2,2]))
        theta6 = np.append(theta6, np.arctan2(-T_Euler[2,1],T_Euler[2,0]))
 
    P = np.array([[theta1[0], theta2[0], theta3[0], theta4[0], theta5[0], theta6[0]],
                  [theta1[1], theta2[1], theta3[1], theta4[1], theta5[1], theta6[1]],
                  [theta1[2], theta2[2], theta3[2], theta4[2], theta5[2], theta6[2]],
                  [theta1[3], theta2[3], theta3[3], theta4[3], theta5[3], theta6[3]]])



    #--------------------------------------------------------------------
    #-------------------------Grupo 5 de ángulos-------------------------
    #    -xc, -yc, -sqrt(1-C3^2), +Euler

    # Encontrando theta 1
    
    theta1 = np.append(theta1, np.arctan2(-Cmu[1][0],-Cmu[0][0]))

    # Encontrando theta 3
    C3 = (np.power(Cmu[0][0],2)+np.power(Cmu[1][0],2)+np.power((Cmu[2][0]-d1),2)-np.power(a2,2)-np.power(a3,2))/(2*a2*a3)
    theta3 = np.append(theta3,np.arctan2(-np.sqrt(1-np.power(C3,2)),C3))

    # Encontrando theta 2
    theta2 = np.append(theta2, np.pi - np.arctan2(Cmu[2][0]-d1,np.sqrt(np.power(Cmu[0][0],2)+np.power(Cmu[1][0],2)))-np.arctan2(a3*np.sin(theta3[4]),a2+a3*C3))

    # Encontrando theta 4,5 y 6
    A1 = DH(0,np.pi/2,d1,theta1[4])
    A2 = DH(a2,0,0,theta2[4]);
    A3 = DH(a3,0,0,theta3[4]);
    T_Euler = np.dot(np.linalg.inv(np.dot(np.dot(A1,A2),A3)),T)
    sin_th = np.sqrt(1-np.power(T_Euler[2][2],2))

    if (sin_th < 1e-6):
        if (T_Euler[2][2] > 0):
            theta4 = np.append(theta4, 0)
            theta5 = np.append(theta5, 0)
            theta6 = np.append(theta6, np.arctan2(-T_Euler[0][1],T_Euler[0][0]))
        else:
            theta4 = np.append(theta4,0)
            theta5 = np.append(theta5, np.pi)
            theta6 = np.append(theta6, np.arctan2(T_Euler[0][1],-T_Euler[0][0]))
    else:
        theta4 = np.append(theta4, np.arctan2(T_Euler[1,2],T_Euler[0,2]))
        theta5 = np.append(theta5, np.arctan2(np.sqrt(1-np.power(T_Euler[2,2],2)),T_Euler[2,2]))
        theta6 = np.append(theta6, np.arctan2(T_Euler[2,1],-T_Euler[2,0]))
 
    P = np.array([[theta1[0], theta2[0], theta3[0], theta4[0], theta5[0], theta6[0]],
                  [theta1[1], theta2[1], theta3[1], theta4[1], theta5[1], theta6[1]],
                  [theta1[2], theta2[2], theta3[2], theta4[2], theta5[2], theta6[2]],
                  [theta1[3], theta2[3], theta3[3], theta4[3], theta5[3], theta6[3]],
                  [theta1[4], theta2[4], theta3[4], theta4[4], theta5[4], theta6[4]]])



    #--------------------------------------------------------------------
    #-------------------------Grupo 6 de ángulos-------------------------
    #    -xc, -yc, -sqrt(1-C3^2), -Euler

    # Encontrando theta 1
    
    theta1 = np.append(theta1, np.arctan2(-Cmu[1][0],-Cmu[0][0]))

    # Encontrando theta 3
    C3 = (np.power(Cmu[0][0],2)+np.power(Cmu[1][0],2)+np.power((Cmu[2][0]-d1),2)-np.power(a2,2)-np.power(a3,2))/(2*a2*a3)
    theta3 = np.append(theta3,np.arctan2(-np.sqrt(1-np.power(C3,2)),C3))

    # Encontrando theta 2
    theta2 = np.append(theta2, np.pi - np.arctan2(Cmu[2][0]-d1,np.sqrt(np.power(Cmu[0][0],2)+np.power(Cmu[1][0],2)))-np.arctan2(a3*np.sin(theta3[5]),a2+a3*C3))

    # Encontrando theta 4,5 y 6
    A1 = DH(0,np.pi/2,d1,theta1[5])
    A2 = DH(a2,0,0,theta2[5]);
    A3 = DH(a3,0,0,theta3[5]);
    T_Euler = np.dot(np.linalg.inv(np.dot(np.dot(A1,A2),A3)),T)
    sin_th = np.sqrt(1-np.power(T_Euler[2][2],2))

    if (sin_th < 1e-6):
        if (T_Euler[2][2] > 0):
            theta4 = np.append(theta4, 0)
            theta5 = np.append(theta5, 0)
            theta6 = np.append(theta6, np.arctan2(-T_Euler[0][1],T_Euler[0][0]))
        else:
            theta4 = np.append(theta4,0)
            theta5 = np.append(theta5, np.pi)
            theta6 = np.append(theta6, np.arctan2(T_Euler[0][1],-T_Euler[0][0]))
    else:
        theta4 = np.append(theta4, np.arctan2(-T_Euler[1,2],-T_Euler[0,2]))
        theta5 = np.append(theta5, np.arctan2(-np.sqrt(1-np.power(T_Euler[2,2],2)),T_Euler[2,2]))
        theta6 = np.append(theta6, np.arctan2(-T_Euler[2,1],T_Euler[2,0]))
 
    P = np.array([[theta1[0], theta2[0], theta3[0], theta4[0], theta5[0], theta6[0]],
                  [theta1[1], theta2[1], theta3[1], theta4[1], theta5[1], theta6[1]],
                  [theta1[2], theta2[2], theta3[2], theta4[2], theta5[2], theta6[2]],
                  [theta1[3], theta2[3], theta3[3], theta4[3], theta5[3], theta6[3]],
                  [theta1[4], theta2[4], theta3[4], theta4[4], theta5[4], theta6[4]],
                  [theta1[5], theta2[5], theta3[5], theta4[5], theta5[5], theta6[5]]])



    #--------------------------------------------------------------------
    #-------------------------Grupo 7 de ángulos-------------------------
    #    -xc, -yc, sqrt(1-C3^2), +Euler

    # Encontrando theta 1
    
    theta1 = np.append(theta1, np.arctan2(-Cmu[1][0],-Cmu[0][0]))

    # Encontrando theta 3
    C3 = (np.power(Cmu[0][0],2)+np.power(Cmu[1][0],2)+np.power((Cmu[2][0]-d1),2)-np.power(a2,2)-np.power(a3,2))/(2*a2*a3)
    theta3 = np.append(theta3,np.arctan2(np.sqrt(1-np.power(C3,2)),C3))

    # Encontrando theta 2
    theta2 = np.append(theta2, np.pi - np.arctan2(Cmu[2][0]-d1,np.sqrt(np.power(Cmu[0][0],2)+np.power(Cmu[1][0],2)))-np.arctan2(a3*np.sin(theta3[6]),a2+a3*C3))

    # Encontrando theta 4,5 y 6
    A1 = DH(0,np.pi/2,d1,theta1[6])
    A2 = DH(a2,0,0,theta2[6]);
    A3 = DH(a3,0,0,theta3[6]);
    T_Euler = np.dot(np.linalg.inv(np.dot(np.dot(A1,A2),A3)),T)
    sin_th = np.sqrt(1-np.power(T_Euler[2][2],2))

    if (sin_th < 1e-6):
        if (T_Euler[2][2] > 0):
            theta4 = np.append(theta4, 0)
            theta5 = np.append(theta5, 0)
            theta6 = np.append(theta6, np.arctan2(-T_Euler[0][1],T_Euler[0][0]))
        else:
            theta4 = np.append(theta4,0)
            theta5 = np.append(theta5, np.pi)
            theta6 = np.append(theta6, np.arctan2(T_Euler[0][1],-T_Euler[0][0]))
    else:
        theta4 = np.append(theta4, np.arctan2(T_Euler[1,2],T_Euler[0,2]))
        theta5 = np.append(theta5, np.arctan2(np.sqrt(1-np.power(T_Euler[2,2],2)),T_Euler[2,2]))
        theta6 = np.append(theta6, np.arctan2(T_Euler[2,1],-T_Euler[2,0]))
 


    #--------------------------------------------------------------------
    #-------------------------Grupo 8 de ángulos-------------------------
    #    -xc, -yc, sqrt(1-C3^2), -Euler

    # Encontrando theta 1
    
    theta1 = np.append(theta1, np.arctan2(-Cmu[1][0],-Cmu[0][0]))

    # Encontrando theta 3
    C3 = (np.power(Cmu[0][0],2)+np.power(Cmu[1][0],2)+np.power((Cmu[2][0]-d1),2)-np.power(a2,2)-np.power(a3,2))/(2*a2*a3)
    theta3 = np.append(theta3,np.arctan2(np.sqrt(1-np.power(C3,2)),C3))

    # Encontrando theta 2
    theta2 = np.append(theta2, np.pi - np.arctan2(Cmu[2][0]-d1,np.sqrt(np.power(Cmu[0][0],2)+np.power(Cmu[1][0],2)))-np.arctan2(a3*np.sin(theta3[7]),a2+a3*C3))

    # Encontrando theta 4,5 y 6
    A1 = DH(0,np.pi/2,d1,theta1[7])
    A2 = DH(a2,0,0,theta2[7]);
    A3 = DH(a3,0,0,theta3[7]);
    T_Euler = np.dot(np.linalg.inv(np.dot(np.dot(A1,A2),A3)),T)
    sin_th = np.sqrt(1-np.power(T_Euler[2][2],2))

    if (sin_th < 1e-6):
        if (T_Euler[2][2] > 0):
            theta4 = np.append(theta4, 0)
            theta5 = np.append(theta5, 0)
            theta6 = np.append(theta6, np.arctan2(-T_Euler[0][1],T_Euler[0][0]))
        else:
            theta4 = np.append(theta4,0)
            theta5 = np.append(theta5, np.pi)
            theta6 = np.append(theta6, np.arctan2(T_Euler[0][1],-T_Euler[0][0]))
    else:
        theta4 = np.append(theta4, np.arctan2(-T_Euler[1,2],-T_Euler[0,2]))
        theta5 = np.append(theta5, np.arctan2(-np.sqrt(1-np.power(T_Euler[2,2],2)),T_Euler[2,2]))
        theta6 = np.append(theta6, np.arctan2(-T_Euler[2,1],T_Euler[2,0]))



    P = np.array([[theta1[0], theta2[0], theta3[0], theta4[0], theta5[0], theta6[0]],
                  [theta1[1], theta2[1], theta3[1], theta4[1], theta5[1], theta6[1]],
                  [theta1[2], theta2[2], theta3[2], theta4[2], theta5[2], theta6[2]],
                  [theta1[3], theta2[3], theta3[3], theta4[3], theta5[3], theta6[3]],
                  [theta1[4], theta2[4], theta3[4], theta4[4], theta5[4], theta6[4]],
                  [theta1[5], theta2[5], theta3[5], theta4[5], theta5[5], theta6[5]],
                  [theta1[6], theta2[6], theta3[6], theta4[6], theta5[6], theta6[6]],
                  [theta1[7], theta2[7], theta3[7], theta4[7], theta5[7], theta6[7]]])



    return P