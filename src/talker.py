#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sys import exit
import readchar

a = 0.11 #interasse delle ruote
b = 0.25 #passo delle ruote



def publish_callback(msg):
    rospy.loginfo(msg)
    pub.publish(msg)

def inserimento(comando,msg,deltasx,deltadx):
    if comando ==('w'):
        if msg.linear.x < 0: 
            msg.linear.x = 0
        msg.linear.x = msg.linear.x + 0.1
    elif comando == ('a'):
 #       steering = steering + 0.1
        if deltasx > 0.7:
            deltasx = 0.7
        else:
            deltasx = deltasx +0.1
        
        deltadx = np.arctan(np.tan(deltasx)+b/a)
        if deltadx > 0.7: 
            deltadx = 0.7

    #      deltasx = deltasx +0.2
    #     deltadx = deltadx +0.1
    elif comando == ('s'):
        if msg.linear.x > 0: 
            msg.linear.x = 0
        msg.linear.x = msg.linear.x - 0.1
    elif comando == ('d'):
        if deltadx < -0.7:
            deltadx = -0.7
        else:
            deltadx = deltadx -0.1

        deltasx = np.arctan(b/a-np.tan(deltadx))
        if deltasx < -0.7:
            deltasx = -0.7

#        steering = steering -0.1
#        deltadx = deltadx -0.2
#        deltasx = deltasx -0.1
    else:
        exit()
    return msg,deltasx,deltadx


if __name__ == '__main__':
    print('Input W-A-S-D per il controllo del robot:\n')
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0

    deltasx = Float64
    deltasx = 0.0
    deltadx = Float64
    deltadx = 0.0

    while True:
        rospy.init_node('talker', anonymous=True)

        pub = rospy.Publisher('/posteriori/cmd_vel', Twist, queue_size=10)
        publish_callback(msg)
        pub = rospy.Publisher('/sinistra/command', Float64, queue_size=10)
        publish_callback(deltasx)
        pub = rospy.Publisher('/destra/command', Float64, queue_size=10)
        publish_callback(deltadx)


        #timer = rospy.Timer(rospy.Duration(1. / 10), publish_callback(msg))  # 10Hz
     
        #rospy.spin()
        #comando = raw_input('Input W-A-S-D per il controllo del robot:\n')
        comando = readchar.readchar()
        msg,deltasx,deltadx = inserimento(comando,msg,deltasx,deltadx)
    #except rospy.ROSInterruptException:
     #   pass
