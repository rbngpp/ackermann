#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sys import exit
import readchar

a = 0.11 #interasse delle ruote
b = 0.25 #passo delle ruote
R = 0 #raggio della curva



def publish_callback(msg):
    rospy.loginfo(msg)
    pub.publish(msg)

def inserimento(comando,msg,deltasx,deltadx):
    if comando ==('w'):
        if msg.linear.x < 0: 
            msg.linear.x = 0
        msg.linear.x = msg.linear.x + 0.1
    elif comando == ('a'): 
        if deltasx > 0.7:
            deltasx = 0.6
        deltasx = deltasx +0.1
        R = b/np.tan(deltasx) + a/2
        deltadx = np.arctan(b/(R+a/2))
        if deltadx > 0.7: 
            deltadx = 0.7
    elif comando == ('s'):
        if msg.linear.x > 0: 
            msg.linear.x = 0
        msg.linear.x = msg.linear.x - 0.1
    elif comando == ('d'):
        if deltadx < -0.7:
            deltadx = -0.6
        deltadx = deltadx -0.1
        R = b/np.tan(deltadx) - a/2
        deltasx = np.arctan(b/(R-a/2))
        if deltasx < -0.7:
            deltasx = -0.7

    else:
        exit()
    return [msg,deltasx,deltadx]


if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    pub1 = rospy.Publisher('/posteriori/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('/sinistra/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/destra/command', Float64, queue_size=10)
    print('Input W-A-S-D per il controllo del robot:\n')
    msg = Twist()
    deltasx = Float64()
    deltadx=Float64()

    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0
    deltasx = 0.0
    deltadx = 0.0

    r = rospy.Rate(10) # Setting a rate (hz) at which to publish
    while not rospy.is_shutdown(): # Runnin until killed
        rospy.loginfo("Sending commands")
        comand = readchar.readchar()
        var=inserimento(comand, msg, deltasx,deltadx)

        deltasx = var[1]
        deltadx = var[2]



        pub1.publish(msg) # Sending the message via our publisher
        pub2.publish(deltasx) 
        pub3.publish(deltadx) 
        r.sleep() # Calling sleep to ensure the rate we set above
