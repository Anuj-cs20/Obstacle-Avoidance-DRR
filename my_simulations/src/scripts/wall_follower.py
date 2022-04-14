#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

vel_pub = None
State = 0
rk = 0
wk = 2

def update_status(pstate):
    global State
    if State is not pstate:
        State = pstate

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
    
    if regions['front'] > 1.5 and regions['frontleft'] > 1.5 and regions['frontright'] > 1.5:
        state_description = 'Object not detected'
        update_status(0)
    elif regions['front'] < 1.5 and regions['frontleft'] > 1.5 and regions['frontright'] > 1.5:
        state_description = 'Object detected at - front'
        update_status(1)
    elif regions['front'] > 1.5 and regions['frontleft'] > 1.5 and regions['frontright'] < 1.5:
        state_description = 'Object detected at - frontright'
        update_status(2)
    elif regions['front'] > 1.5 and regions['frontleft'] < 1.5 and regions['frontright'] > 1.5:
        state_description = 'Object detected at - frontleft'
        update_status(0)
    elif regions['front'] < 1.5 and regions['frontleft'] > 1.5 and regions['frontright'] < 1.5:
        state_description = 'Object detected at - front and frontright'
        update_status(1)
    elif regions['front'] < 1.5 and regions['frontleft'] < 1.5 and regions['frontright'] > 1.5:
        state_description = 'Object detected at - front and frontleft'
        update_status(1)
    elif regions['front'] < 1.5 and regions['frontleft'] < 1.5 and regions['frontright'] < 1.5:
        state_description = 'Object detected at - front and frontleft and frontright'
        update_status(1)
    elif regions['front'] > 1.5 and regions['frontleft'] < 1.5 and regions['frontright'] < 1.5:
        state_description = 'Object detected at  - frontleft and frontright'
        update_status(2)
    else:
        state_description = 'Object not detected'
        rospy.loginfo(regions)

def find_wall():
    msg = Twist()
    msg.linear.x = -0.2
    msg.angular.z = -0.3
    return msg   

def turn_left():
    msg = Twist()
    msg.angular.z = 0.3
    return msg

def follow_wall():
    msg = Twist()
    msg.linear.x = -0.3
    return msg

def main():
    global vel_pub,State
    rospy.init_node("Obstacle_Avoidance",anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=100)
    scan_msg = rospy.Subscriber('/scan',LaserScan,callBackFn)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()

        if State == 0:
            msg = find_wall()
        elif State == 1:
            msg = turn_left()
        elif State == 2:
            msg = follow_wall()        
        
        vel_pub.publish(msg)
        #rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException :
        print("node terminated") 
