#!/usr/bin/env python

# PROJECT IMPORTS
import rospy

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sys import exit
import readchar

from scipy.integrate import odeint

# DEFINITIONS
incremento = 0.2
a = 0.21 # Wheelbase
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
        msg.linear.x = msg.linear.x + incremento
    elif command == ('a'): 
        if deltasx > 0.7:
            deltasx = 0.6
        deltasx = deltasx +incremento
        R = b/np.tan(deltasx) + a/2
        deltadx = np.arctan(b/(R+a/2))
        if deltadx > 0.7: 
            deltadx = 0.7
    elif command == ('s'):
        if msg.linear.x > 0: 
            msg.linear.x = 0
        msg.linear.x = msg.linear.x - incremento
    elif command == ('d'):
        if deltadx < -0.7:
            deltadx = -0.6
        deltadx = deltadx -incremento
        R = b/np.tan(deltadx) - a/2
        deltasx = np.arctan(b/(R-a/2))
        if deltasx < -0.7:
            deltasx = -0.7
    else:
        exit()
    return [msg,deltasx,deltadx]

def get_angle_pose(quaternion_pose):
        q = [quaternion_pose.orientation.x,
                quaternion_pose.orientation.y,
                quaternion_pose.orientation.z,
                quaternion_pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        theta = yaw
        return theta

    #current robot pose
def odometryCb(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    theta = get_angle_pose(msg.pose.pose)
    q = np.array([x, y, theta])
    #rospy.loginfo(self.q)
    return q

if __name__ == '__main__':
    
    rospy.loginfo("Starting node Trajectory control")
    rospy.init_node('trajectory_control', anonymous=True) #make node
    #self.twist_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    twist_pub = rospy.Publisher('/posteriori/cmd_vel', Twist, queue_size=10) 

    q = [0,0,0]
    
    
    #rospy.Subscriber('move_base_simple/goal', PoseStamped, self.on_goal)

while True:

    rospy.Subscriber('/ground_truth/state',Odometry, q)
    q = odometryCb

    rospy.loginfo(q)
    


"""
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

"""
