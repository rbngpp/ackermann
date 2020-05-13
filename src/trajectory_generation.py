#!/usr/bin/env python
import numpy as np

#Trajectory generation
class Trajectory_generation():

    def cubic_trajectory(self, q_i, q_f, k, t=None):
        x_i = q_i[0]
        y_i = q_i[1]
        x_f = q_f[0]
        y_f = q_f[1]
        if t is not None:
            s = t/t[-1]
            tau = 1/t[-1]
        else:
            s = np.linspace(0,1,200)
            tau = 1
            
        b_x = k*np.cos(q_i[2]) + 3*q_i[0]
        b_y = k*np.sin(q_i[2]) + 3*q_i[1]

        a_x = k*np.cos(q_f[2]) - 3*q_f[0]
        a_y = k*np.sin(q_f[2]) - 3*q_f[1]

        #Cartesian cubic path 
        x = x_f*s**3 - x_i*(s-1)**3 + a_x * s**2 * (s-1) + b_x * s * (s-1)**2
        y = y_f*s**3 - y_i*(s-1)**3 + a_y * s**2 * (s-1) + b_y * s * (s-1)**2

        #Compute first derivative
        xp = 3*x_f*s**2 - 3*x_i*(s-1)**2 + a_x*(3*s**2 -2*s) + b_x*(s-1)*(3*s-1)
        yp = 3*y_f*s**2 - 3*y_i*(s-1)**2  +a_y*(3*s**2 -2*s) + b_y*(s-1)*(3*s-1)
        
        #We can compute the geometric velocity and the posture angle theta
        v = np.sqrt(xp**2 + yp**2)
        theta = np.arctan2(yp,xp)
        
        #Compute second derivative
        xpp = 6*x_f*s - 6*x_i*(s-1) + a_x*(6*s-2) + b_x*(6*s-4)
        ypp = 6*y_f*s - 6*y_i*(s-1) + a_y*(6*s-2) + b_y*(6*s-4)
        
        #Compute the angular velocity
        w = (ypp*xp - xpp*yp)/(v**2)
        
        return [x, y, v, w, theta]

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
            if x_d[i] < 0 and (y_d[i] < 0 and y_d[i] > 2.15): 
                k.append(i)
            i = i+1
        
        x_d = np.delete(x_d,k)
        y_d = np.delete(y_d,k)
        dotx_d = np.delete(dotx_d,k)
        doty_d = np.delete(doty_d,k)

        print(x_d)
        print(y_d)

        v_d = np.sqrt(dotx_d**2 + doty_d**2)
        theta_d = np.arctan2(doty_d, dotx_d)
        w_d = w_d_val * np.ones(len(t))
        return [x_d, y_d, v_d, w_d, theta_d, dotx_d, doty_d]

    def eight_trajectory(self, t):
        x_c = 0
        y_c = 0
        R = 3
        w_d = 1./15  # Desired Angular speed
        x_d = x_c + R*np.sin(2*w_d*t) 
        y_d = x_c + R*np.sin(w_d*t) 
        
        dotx_d = 2*R*w_d*np.cos(2*w_d*t)
        doty_d = R*w_d*np.cos(w_d*t)
        return [x_d,y_d,dotx_d, doty_d]
