#!/usr/bin/env python
"""
Simple MAVROS pose controller.

AUTHOR: Harshal Deshpande
"""

import rospy
from dronecontroller import *
from mavros_msgs.srv import CommandBool


def main():
    global my_drone
    my_drone.arm()
    my_drone.fill_buffer()
    new_rate = rospy.Rate(100)

    result = my_drone.mode_service(custom_mode="OFFBOARD")
    print("setting mode: ", result)
    
    while not rospy.is_shutdown():
        if my_drone.current_state.mode != "OFFBOARD":
            # print ('offboard acheived')
            my_drone.mode_service(base_mode=0, custom_mode="OFFBOARD")
            # my_drone.rate.sleep()

        my_drone.set_pos(0,0,5)
        # my_drone.rate.sleep()
        new_rate.sleep()


if __name__ == "__main__":
    rospy.init_node("drone_control",disable_signals=True)
    my_drone = DroneController()
    try:
        main()
    except KeyboardInterrupt:
        # print("In keyboard interrupt...")
        my_drone.land()