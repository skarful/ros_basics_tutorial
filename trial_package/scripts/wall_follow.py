#! /usr/bin/env python

"""
Control node - acts as client to both action_server and find_wall_server. Responsible for wall-following
behaviour of the robot
"""

#Imports
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from trial_package.srv import FindWall, FindWallRequest
from trial_package.msg import OdomRecordGoal, OdomRecordAction
import actionlib

#Note: This can be handled better -> using updateValues() as in action/service servers
def callback_func(msg):
    global laserRange
    laserRange = msg.ranges

#Whenever feedback received, print distance covered
def feedCallback(req):
    print("Distance covered = " + str(req.current_total))

#Initialization
rospy.init_node('Wall_follower')
rospy.wait_for_service('/wall_finder')
serv1 = rospy.ServiceProxy('wall_finder', FindWall)
obj1 = FindWallRequest()

#Setup action client
act_client = actionlib.SimpleActionClient('/record_odom', OdomRecordAction)
act_client.wait_for_server()

sub = rospy.Subscriber('/scan', LaserScan, callback_func)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

laserRange = []

#Call action before starting anything (to get odometry readings)
golly = OdomRecordGoal()
act_client.send_goal(golly, feedback_cb=feedCallback)

#Call service before starting wall following
result = serv1(obj1)
print(result)
print('starting wall follow')

#Loop rate
rr = rospy.Rate(0.5)

#Control loop - velocities changed based on distance/position from wall
while not rospy.is_shutdown():

    tvar = Twist()
    right = laserRange[90]
    centre = laserRange[180]
    # Uncomment below line to print laser readings at back, right, front and left of the bot respectively
    # print(msg.ranges[0], msg.ranges[90], msg.ranges[180], msg.ranges[270])
    if centre < 0.5:
        tvar.angular.z = 0.3
        print('steep turn')
    elif right > 0.3:
        tvar.angular.z = -0.1
        print('towards wall')
    elif right < 0.2:
        tvar.angular.z = 0.1
        print('away from wall')
    else:
        tvar.angular.z = 0
        print('straight')

    tvar.linear.x = 0.03
    pub.publish(tvar)

    cc = act_client.get_state()
    if cc > 2:
        print('lap complete, exiting')
        break

    rr.sleep()

# rospy.spin()