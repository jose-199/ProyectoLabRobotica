import cv2
import numpy as np

imagen = cv2.imread('xy3.png')
gray = cv2.cvtColor(imagen,cv2.COLOR_BGR2GRAY)
_,th = cv2.threshold(gray,100,255,cv2.THRESH_BINARY)

contornos,hierarchy = cv2.findContours(th, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

for i in range(0,len(contornos[1])):
    print("x: " + str(contornos[1][i][0][0]) + "       " + "y: " + str(contornos[1][i][0][1]))

cv2.drawContours(imagen, contornos[0], -1, (0,255,0), 3)

cv2.imshow('th',th)
cv2.imshow('imagen',imagen)
#cv2.imshow('img',img)
cv2.waitKey(0)
cv2.destroyAllWindows()