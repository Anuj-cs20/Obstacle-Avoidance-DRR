#!/usr/bin/env python
from pydoc import describe
from numpy import angle
import rospy
import time
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def pose_value(pose_msg):
    global x,y,yaw
    x = pose_msg.x
    y = pose_msg.y
    yaw = pose_msg.theta

def motion_inLine(vel_pub,speed,distance,isFor):
    vel_cmd = Twist()
    global x,y
    x0 = x
    y0 = y

    if isFor:
        vel_cmd.linear.x = abs(speed)
    else:
        vel_cmd.linear.x = -abs(speed)

    distance_moved = 0.0
    rate = rospy.Rate(2)
    while True:
        vel_pub.publish(vel_cmd)
        rate.sleep()
        distance_moved = abs(math.sqrt(((x-x0)**2) + ((y-y0)**2)))

        if distance_moved > distance:
            rospy.loginfo("reached")
            break

    vel_cmd.linear.x = 0
    vel_pub.publish(vel_cmd)

def motion_inAngle(vel_pub,angular_speed_degree,relative_angle_degree,clock):
    vel_cmd = Twist()
    angular_speed = math.radians(abs(angular_speed_degree))
    relative_angle = math.radians(abs(relative_angle_degree))

    if clock:
        vel_cmd.angular.z = -abs(angular_speed)
    else:
        vel_cmd.angular.z = abs(angular_speed)

    curr_angle = 0.0
    t0 = rospy.Time.now().to_sec()
    rate = rospy.Rate(10)
    while True:
        vel_pub.publish(vel_cmd)
        rate.sleep()
        t = rospy.Time.now().to_sec()
        curr_angle = angular_speed*(t-t0)

        if curr_angle > relative_angle:
            rospy.loginfo("reached")
            break

    vel_cmd.angular.z = 0
    vel_pub.publish(vel_cmd)

def motion_goal(vel_pub,goal_x,goal_y):
    vel_cmd = Twist()
    global x
    global y,yaw
    
    rate = rospy.Rate(10)
    while True:
        kd = 0.5
        distance = abs(math.sqrt(math.sqrt(((goal_x-x)**2) + ((goal_y-y)**2))))
        vel_cmd.linear.x = kd*distance

        ka = 4.0
        angle = math.atan2(goal_y-y,goal_x-x)
        vel_cmd.angular.z = (angle -yaw)*ka

        vel_pub.publish(vel_cmd)
        #rate.sleep()
        print(f"x = {x}  y = {y}  distance to goal = {distance}")
        

        if distance < 0.01:
            rospy.loginfo("reached")
            break

    vel_cmd.linear.x = 0
    vel_cmd.angular.z = 0
    vel_pub.publish(vel_cmd)    
        
def motion_Rorin(vel_pub,speed_in_degree,desired_angle_degree):
    relative_angle_radians = math.radians(desired_angle_degree) - yaw
    clockwise = 0
    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0

    print(f"relative angle in degrees = {math.degrees(relative_angle_radians)}")
    print(f"desired angle in degrees = {desired_angle_degree}")
    motion_inAngle(vel_pub,speed_in_degree,math.degrees(abs(relative_angle_radians)),clockwise)        

def motion_spiral(vel_pub,wk,rk):
    vel_msg = Twist()
    loop_rate = rospy.Rate(1)

    while ((x<10.0) and (y<10.0)):
        rk = rk+1
        vel_msg.linear.x = rk
        vel_msg.angular.z = wk
        vel_pub.publish(vel_msg)
        loop_rate.sleep()

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    vel_pub.publish(vel_msg)    
       
def motion_grid(vel_pub):
    desired_pose = Pose()
    desired_pose.x = 0.5
    desired_pose.y = 0.5
    desired_pose.theta = 0

    motion_goal(vel_pub,0.5,0.5)
    motion_Rorin(vel_pub,30,math.degrees(desired_pose.theta))

    for i in range(5):
        motion_inLine(vel_pub,2.0,0.5,True)
        motion_inAngle(vel_pub,30,90,False)
        motion_inLine(vel_pub,3.0,9.0,True)
        motion_inAngle(vel_pub,30,90,True)
        motion_inLine(vel_pub,2.0,0.5,True)
        motion_inAngle(vel_pub,30,90,True)
        motion_inLine(vel_pub,3.0,9.0,True)
        motion_inAngle(vel_pub,30,90,False)
    pass    



if __name__ == '__main__':
    try:
        rospy.init_node("turtlesim_cleaner",anonymous=True)
        vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=100)
        pose_msg = rospy.Subscriber('/turtle1/pose',Pose,pose_value)
        time.sleep(2)

        #motion_inLine(vel_pub,1.0,3.0,True)
        #motion_inAngle(vel_pub,30,90,True)
        #motion_goal(vel_pub,1.0,1.0)
        #motion_Rorin(vel_pub,30,90)
        motion_spiral(vel_pub,2,0)
        motion_grid(vel_pub)

    except rospy.ROSInterruptException :
        print("node terminated") 
