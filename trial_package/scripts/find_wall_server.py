#! /usr/bin/env python

"""
Service server that rotates and points the robot to closest object. Robot then moves towards the object and
performs a 90 degree turn. This works as a precursor to wall following
"""

#Imports
import rospy
from trial_package.srv import FindWall, FindWallResponse
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

#Callback when service is called
def servhandler(request):
    #Find minimum value
    tvar = Twist()

    time.sleep(1)
    updateValues()
    global laserRange

    # minx = min(laserRange)
    inx = laserRange.index(min(laserRange))

    #180 corresponds to laser value in front of bot
    if inx < 177:
        tvar.angular.z = -0.1 #rotate towards right (clockwise)
    elif inx > 183:
        tvar.angular.z = 0.1 #rotate towards left (anti clock)
    else:
        tvar.angular.z = 0

    pub.publish(tvar)
    print("rotating towards wall")

    #Keep rotating till robot faces wall
    while(inx < 177 or inx > 183):
        # minx = min(laserRange)
        updateValues()
        inx = laserRange.index(min(laserRange))

    print("facing the wall now")
    #Move towards the wall
    while(laserRange[180] > 0.3):
        tvar.angular.z = 0
        tvar.linear.x = 0.1
        pub.publish(tvar)
        updateValues()
    
    print("Reached wall")
    tvar.linear.x = 0
    tvar.angular.z = 0
    pub.publish(tvar)

    #Rotate 90degrees or so, until wall is on the robot's right
    while(laserRange[90] > 0.3):
        tvar.angular.z = 0.1
        tvar.linear.x = 0
        pub.publish(tvar)
        updateValues()

    tvar.angular.z = 0
    pub.publish(tvar)
    print("reached init position")

    return True

#Callback function when /scan obtained
def callback_func(msg):
    global lvalues
    lvalues = msg.ranges

#Used to update sensor values at required rate
def updateValues():
    global laserRange, lvalues
    laserRange = lvalues

#Initialization
rospy.init_node('find_wall_server_node')
serv1 = rospy.Service('/wall_finder', FindWall, servhandler)
sub = rospy.Subscriber('/scan', LaserScan, callback_func)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

laserRange = []     #global variable used to access laser values
lvalues = []

rospy.spin()