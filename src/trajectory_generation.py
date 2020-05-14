#!/usr/bin/env python
import numpy as np

#Trajectory generation
class Trajectory_generation():
   
    def cyrcular_trajectory(self, t):
        
        A = [0.0, 0.0]
        B = [0.499, 0.021]
        C = [0.995, 0.085]  
        
        var = np.array([[A[0],A[1],1], [B[0],B[1],1], [C[0],C[1],1]])

        notA = -(A[0]**2)-(A[1]**2)
        notB = -(B[0]**2)-(B[1]**2)
        notC = -(C[0]**2)-(C[1]**2)
        noti = np.array([notA, notB, notC])
        soluzione = np.linalg.solve(var, noti)

        R = np.sqrt((soluzione[0]**2/4)+(soluzione[1]**2/4)-soluzione[2])

        v_d_val = 0.5 # m/s
        w_d_val = v_d_val/R

        x_c = -soluzione[0]/2
        y_c = -soluzione[1]/2

        """        
        x_d = [0.0, 0.249, 0.499, 0.747, 0.995, 1.240, 1.483, 1.723, 1.960, 2.193, 2.384, 2.159]
        y_d = [0.0, 0.005, 0.021, 0.048, 0.085, 0.134, 0.192, 0.262, 0.341, 0.430, 0.512, 0.403]
        """

        x_d = R * np.cos(w_d_val * t) + x_c
        y_d = R * np.sin(w_d_val * t) + y_c
        dotx_d = -R*w_d_val*np.sin(w_d_val* t)
        doty_d =  R*w_d_val*np.cos(w_d_val* t)

        i = 0
        k = []
        for element in x_d:
            if x_d[i] < 0 or y_d[i] < 0 or y_d[i] > 2.15: 
                k.append(i)
            i = i+1
        
        x_d = np.delete(x_d,k)
        y_d = np.delete(y_d,k)
        dotx_d = np.delete(dotx_d,k)
        doty_d = np.delete(doty_d,k)

        print(np.shape(x_d))
        print(y_d)

        v_d = np.sqrt(dotx_d**2 + doty_d**2)
        theta_d = np.arctan2(doty_d, dotx_d)
        w_d = w_d_val * np.ones(len(t))
        return [x_d, y_d, v_d, w_d, theta_d, dotx_d, doty_d]

    