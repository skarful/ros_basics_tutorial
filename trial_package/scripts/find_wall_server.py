#! /usr/bin/env python

import rospy
from trial_package.srv import FindWall, FindWallResponse
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

def servhandler(request):
    #Find minimum value
    tvar = Twist()

    time.sleep(1)
    updateValues()
    global laserRange

    # minx = min(laserRange)
    inx = laserRange.index(min(laserRange))

    if inx < 177:
        tvar.angular.z = -0.1 #rotate towards right
    elif inx > 183:
        tvar.angular.z = 0.1
    else:
        tvar.angular.z = 0

    pub.publish(tvar)
    print("rotating towards wall")

    while(inx < 177 or inx > 183):
        # minx = min(laserRange)
        updateValues()
        inx = laserRange.index(min(laserRange))

    print("facing the wall now")
    while(laserRange[180] > 0.3):
        tvar.angular.z = 0
        tvar.linear.x = 0.1
        pub.publish(tvar)
        updateValues()
    
    print("Reached wall")
    tvar.linear.x = 0
    tvar.angular.z = 0
    pub.publish(tvar)

    while(laserRange[90] > 0.3):
        tvar.angular.z = 0.1
        tvar.linear.x = 0
        pub.publish(tvar)
        updateValues()

    tvar.angular.z = 0
    pub.publish(tvar)
    print("reached init position")

    return True

def callback_func(msg):
    global lvalues
    lvalues = msg.ranges

def updateValues():
    global laserRange, lvalues
    laserRange = lvalues

rospy.init_node('find_wall_server_node')
serv1 = rospy.Service('/wall_finder', FindWall, servhandler)
sub = rospy.Subscriber('/scan', LaserScan, callback_func)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

laserRange = []     #global variable used to access laser values
lvalues = []

rospy.spin()