import numpy as np

'''
This function implements a simple constant gain controller for $k_1(v_d,\omega_d)$
''' 
zeta = 0.7
a = 1

def k1(v_d, w_d):
    global zeta, a
    return 2*zeta*a
    #return 1.2

'''
This function implements a simple constant gain controller for $k_3(v_d,\omega_d)$
''' 
def k3(v_d, w_d):
    global zeta, a
    return 2*zeta*a
    #return 1.2

'''
This function implements the control. k1 and k3 functions
are used to (possibly) implement time varying gains, whereas
the gain k2 is set in the function.
'''
def control(e, v_d, w_d):
    #k2 = 0.5;
    global a, zeta
    k2 = (a**2 -w_d**2)/v_d
    
    u_1 = -k1(v_d, w_d) * e[0]
    
    # Be sure that if e[2] = 0 sin(e[2])/e[2] is computes to 1.0
    if e[2] == 0:
        u_2 = -k2*v_d*e[1] - k3(v_d,w_d)*e[2]
    else:
        u_2 = -k2*v_d*np.sin(e[2])/e[2]*e[1] - k3(v_d,w_d)*e[2]
    
    return np.array([u_1,u_2])
    
'''
Error dynamics. This function is used odeint 
to simulate the closed-loop system. 
'''
def unicycle_error_model(e, t, v_d, w_d, time):
    T = np.searchsorted(time,t)
    if (T>=len(time)):
        T = len(time)-1
        
    u_1, u_2 = control(e, v_d[T], w_d[T])
    edot_1 = u_1 + e[1]*(w_d[T] - u_2)
    edot_2 = v_d[T]*np.sin(e[2]) - e[0]*(w_d[T] - u_2)
    edot_3 = u_2
    ret =  [edot_1,edot_2, edot_3]
    return ret

    