#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Int32, Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Pose
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from dronecontroller import *


class ControlDrone():
    def __init__(self):
        self.uav = DroneController()
        self.error_thr = 0.1
    def goto(self,x,y,z):
        self.uav.arm()
        self.uav.fill_buffer()
        self.uav.mode_service(custom_mode="OFFBOARD")
        
        dest = PoseStamped()
        dest.pose.position.x = x
        dest.pose.position.y = y
        dest.pose.position.z = z
        count = 0
        while self.uav.pose.pose.position != dest.pose.position:
            if self.uav.current_state.mode != "OFFBOARD":
                self.uav.mode_service(custom_mode="OFFBOARD")
            
            self.uav.set_pos(x,y,z)
            # print count,"in loop"
            count = count + 1

            if self.uav.pose.pose.position.x < dest.pose.position.x + self.error_thr and self.uav.pose.pose.position.x > dest.pose.position.x - self.error_thr and \
                self.uav.pose.pose.position.y < dest.pose.position.y + self.error_thr and self.uav.pose.pose.position.y > dest.pose.position.y - self.error_thr and \
                self.uav.pose.pose.position.z < dest.pose.position.z + self.error_thr and self.uav.pose.pose.position.z > dest.pose.position.z - self.error_thr:
                
                self.uav.land()
                break

            self.uav.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("Pose_control_node",disable_signals=True)

    my_control = ControlDrone()
    
    try:
        # my_control.goto(0,0,2)
        # my_control.goto(2,0,2)
        # my_control.goto(2,2,2)
        # my_control.goto(0,2,2)
        # my_control.goto(0,0,2)
        my_control.uav.goto(0,0,2)
        my_control.uav.goto(2,0,2)
        my_control.uav.goto(2,2,2)
        my_control.uav.goto(0,2,2)
        my_control.uav.goto(0,0,2)

        while not rospy.is_shutdown():
            if my_control.uav.current_state.mode == "AUTO.LAND":
                rospy.signal_shutdown("Drone has completed it's task")
                print "Drone has completed it's task"
            my_control.uav.rate.sleep()
        
        rospy.spin()
    
    except KeyboardInterrupt:
        # print("In keyboard interrupt...")
        try:
            my_control.uav.land()
        except:
            print "DONE!DONE!DONE!"