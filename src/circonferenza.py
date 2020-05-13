#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt

def calcolo():
 
    A = [0.499, 0.021]
    B = [0.995, 0.085]
    C = [1.483, 0.192]
   
    
    var = np.array([[A[0],A[1],1], [B[0],B[1],1], [C[0],C[1],1]])

    notA = -(A[0]**2)-(A[1]**2)
    notB = -(B[0]**2)-(B[1]**2)
    notC = -(C[0]**2)-(C[1]**2)
    noti = np.array([notA, notB, notC])
    soluzione = np.linalg.solve(var, noti)
    print(soluzione)
    
    x = np.linspace(-1,1,100)
    y = np.linspace(-1,1,100)

    X,Y = np.meshgrid(x,y)
    
    F = X**2 + Y**2 + soluzione[0]*X + soluzione[1]*Y + soluzione[2]
    plt.contour(X,Y,F,[0])
    plt.show()

calcolo()
