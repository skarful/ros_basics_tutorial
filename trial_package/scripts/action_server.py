#! /usr/bin/env python

import rospy
from trial_package.msg import OdomRecordAction, OdomRecordFeedback, OdomRecordResult
import actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import math

def callbackFunc(req):
    #start calculating odometry
    feed = OdomRecordFeedback()
    rest = OdomRecordResult()
    rest.list_of_odoms = []
    tempPoint = Point()

    #Get initial position
    prev_rx, prev_ry, prev_rz = updateValues()

    r = rospy.Rate(1)
    dist = 0

    while (not acty.is_preempt_requested()) and dist <6:
        RX, RY, RZ = updateValues()
        tempPoint.x = RX
        tempPoint.y = RY
        tempPoint.z = RZ
        rest.list_of_odoms.append(tempPoint)

        dist = dist + math.sqrt(pow(RX - prev_rx, 2) + pow(RY - prev_ry, 2))
        prev_ry = RY
        prev_rx = RX
        feed.current_total = dist
        acty.publish_feedback(feed)
        r.sleep()

    #If pre-empted or complete:
    if acty.is_preempt_requested():
        acty.set_preempted(rest, "Preempted solution")
    else:
        acty.set_succeeded(rest, "Completed required distance")

def odomCallback(req):
    global robot_x, robot_y, robot_theta
    robot_x = req.pose.pose.position.x
    robot_y = req.pose.pose.position.y
    robot_theta = req.pose.pose.orientation.z

def updateValues():
    rx = robot_x
    ry = robot_y
    rz = robot_theta
    return rx, ry, rz

rospy.init_node('odometry_action_node')
acty = actionlib.SimpleActionServer('/record_odom', OdomRecordAction, callbackFunc, False)
sub = rospy.Subscriber('/odom', Odometry, odomCallback)
rospy.sleep(1)

#variables to store values
robot_x = 0
robot_y = 0
robot_theta = 0

#start action server
acty.start()

rospy.spin()