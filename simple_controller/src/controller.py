#! /usr/bin/env python

from turtle import speed
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from std_srvs.srv import Empty, EmptyResponse

x = 0.0
y = 0.0
theta = 0.0

def newOdom(msg):
    global x, y, theta
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def handle_goal(request):
    goal_x = request.goal.x
    goal_y = request.goal.y

    inc_x = goal_x - x
    inc_y = goal_y - y
    angle_to_goal = atan2(inc_y, inc_x)

    while abs(angle_to_goal - theta) > 0.1:
        speed.linear.x = 0.0
        speed.angular.z = 0.1
        pub.publish(speed)
        rospy.sleep(0.1)

    speed.linear.x = 0.5
    speed.angular.z = 0.0
    pub.publish(speed)
    rospy.sleep(2.0)  # Adjust this sleep duration based on the time needed to reach the goal

    speed.linear.x = 0.0
    speed.angular.z = 0.0
    pub.publish(speed)

    return EmptyResponse()

rospy.init_node("goal_controller")
sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)

rospy.Service('move_to_goal', Empty, handle_goal)

rospy.spin()
