#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

vel_pub = None

def callBackFn(laser_msg):
    # 640  -  260
    # 100  -  40 
    # 460/5 = 92
    regions = {
       "backleft" : min(min(laser_msg.ranges[0:99]),10),
       "left" : min(min(laser_msg.ranges[100:191]),10),
       "frontleft" : min(min(laser_msg.ranges[192:283]),10),
       "front" : min(min(laser_msg.ranges[284:375]),10),
       "frontright" : min(min(laser_msg.ranges[376:467]),10),
       "right" : min(min(laser_msg.ranges[468:559]),10),
       "backright" : min(min(laser_msg.ranges[560:659]),10)
    }

    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    if regions['front'] > 1 and regions['frontleft'] > 1 and regions['frontright'] > 1:
        state_description = 'Object not detected'
        linear_x = 0.6
        angular_z = 0
    elif regions['front'] < 1 and regions['frontleft'] > 1 and regions['frontright'] > 1:
        state_description = 'Object detected at - front'
        linear_x = 0
        angular_z = 0.2
    elif regions['front'] > 1 and regions['frontleft'] > 1 and regions['frontright'] < 1:
        state_description = 'Object detected at - frontright'
        linear_x = 0
        angular_z = 0.2
    elif regions['front'] > 1 and regions['frontleft'] < 1 and regions['frontright'] > 1:
        state_description = 'Object detected at - frontleft'
        linear_x = 0
        angular_z = -0.2
    elif regions['front'] < 1 and regions['frontleft'] > 1 and regions['frontright'] < 1:
        state_description = 'Object detected at - front and frontright'
        linear_x = 0
        angular_z = 0.2
    elif regions['front'] < 1 and regions['frontleft'] < 1 and regions['frontright'] > 1:
        state_description = 'Object detected at - front and frontleft'
        linear_x = 0
        angular_z = -0.2
    elif regions['front'] < 1 and regions['frontleft'] < 1 and regions['frontright'] < 1:
        state_description = 'Object detected at - front and frontleft and frontright'
        linear_x = 0
        angular_z = 0.2
    elif regions['front'] > 1 and regions['frontleft'] < 1 and regions['frontright'] < 1:
        state_description = 'Object detected at  - frontleft and frontright'
        linear_x = 0.6
        angular_z = 0
    else:
        state_description = 'Object not detected'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = -angular_z
    vel_pub.publish(msg)

def main():
    global vel_pub
    rospy.init_node("Obstacle_Avoidance",anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=100)
    scan_msg = rospy.Subscriber('/scan',LaserScan,callBackFn)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException :
        print("node terminated") 
