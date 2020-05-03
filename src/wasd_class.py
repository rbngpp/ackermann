#!/usr/bin/env python

# PROJECT IMPORTS
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sys import exit
import readchar

# DEFINITIONS
a = 0.11 # Wheelbase
b = 0.25 # Step of the wheels
R = 0 # Curve radius
topics = ['/posteriori/cmd_vel','/sinistra/command','/destra/command'] # Topics for publishing
msg = Twist() # Message for the rear traction
deltasx = Float64() # Message for the front left steering wheel
deltadx = Float64() # Message for the front right steering wheel



class ControlWasd():

    def __init__(self,topic,kind):   
        self.pub = rospy.Publisher(topic, kind, queue_size=10)

    def publish(self,msg):
        rospy.loginfo(msg)
        self.pub.publish(msg)

def ctrl_input(command,msg,deltasx,deltadx):
    if command ==('w'):
        if msg.linear.x < 0: 
            msg.linear.x = 0
        msg.linear.x = msg.linear.x + 0.1
    elif command == ('a'): 
        if deltasx > 0.7:
            deltasx = 0.6
        deltasx = deltasx +0.1
        R = b/np.tan(deltasx) + a/2
        deltadx = np.arctan(b/(R+a/2))
        if deltadx > 0.7: 
            deltadx = 0.7
    elif command == ('s'):
        if msg.linear.x > 0: 
            msg.linear.x = 0
        msg.linear.x = msg.linear.x - 0.1
    elif command == ('d'):
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
    r = rospy.Rate(10) # Setting a rate (hz) at which to publish
    # Initialization of the messages
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0
    deltasx = 0.0
    deltadx = 0.0
    # Setting of the channels
    pub1 = ControlWasd(topics[0], Twist)
    pub2 = ControlWasd(topics[1], Float64)
    pub3 = ControlWasd(topics[2], Float64)

    while not rospy.is_shutdown(): # Runnin until killed
        
        rospy.loginfo("Sending commands")
        # Reading of the commands from the user
        command = readchar.readchar()
        move = ctrl_input(command, msg, deltasx,deltadx)
        # Assign commands to topics
        msg = move[0]
        deltasx = move[1]
        deltadx = move[2]  
        # Publish topics in their channels
        pub1.publish(msg)
        pub2.publish(deltasx)
        pub3.publish(deltadx)
        # Wait until new command
        r.sleep()