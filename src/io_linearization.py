#!/usr/bin/env python
import numpy as np




def io_linearization_control_law(y1, y2, theta, y1d, y2d, doty1d, doty2d, b):
    # Define the two control gains. Notice we can define "how fast" we track on y_1 and y_2 _independently_
    k_1 = 0.5
    k_2 = 0.5
    
    #return virtual input doty1, doty2
    u_1 = doty1d + k_1*(y1d - y1)
    u_2 = doty2d + k_2*(y2d - y2)

    #return control input v, w
    v = np.cos(theta) * u_1 + np.sin(theta) * u_2
    w = u_2/b * np.cos(theta) - u_1/b *np.sin(theta) #dottheta

    return np.array([v, w])



