#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sys import exit
import readchar



def publish_callback(msg):
    rospy.loginfo(msg)
    pub.publish(msg)

def inserimento(comando,msg):
    if comando ==('w'):
        msg.linear.x = msg.linear.x + 0.1
    elif comando == ('a'):
        msg.angular.z = msg.angular.z - 0.1
    elif comando == ('s'):
        msg.linear.x = msg.linear.x - 0.1
    elif comando == ('d'):
        msg.angular.z = msg.angular.z + 0.1
    else:
        exit()
    return msg


if __name__ == '__main__':
    print('Input W-A-S-D per il controllo del robot:\n')
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0
    while True:
        rospy.init_node('talker', anonymous=True)
        pub = rospy.Publisher('/r2d2_diff_drive_controller/cmd_vel', Twist, queue_size=10)
        #timer = rospy.Timer(rospy.Duration(1. / 10), publish_callback(msg))  # 10Hz
        publish_callback(msg)
        #rospy.spin()
        #comando = raw_input('Input W-A-S-D per il controllo del robot:\n')
        comando = readchar.readchar()
        msg = inserimento(comando,msg)
    #except rospy.ROSInterruptException:
     #   pass
