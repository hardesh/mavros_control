#!/usr/bin/env python

import rospy
from dronecontroller import *
from mavros_msgs.srv import CommandBool



def main():
    global my_drone

    arm_cmd = CommandBool()
    arm_cmd.value = True
    # result = my_drone.arm_service(arm_cmd.value)
    my_drone.arm()
    # print("arming: ", result)

    my_drone.fill_buffer()
        
    result = my_drone.mode_service(custom_mode="OFFBOARD")
    print("setting mode: ", result)
    
    while not rospy.is_shutdown():
        if my_drone.current_state.mode != "OFFBOARD":
            print ('offboard acheived')
            my_drone.mode_service(base_mode=0, custom_mode="OFFBOARD")
            my_drone.rate.sleep()

        my_drone.set_pos(0,0,5)
        # rospy.spin()
        
        my_drone.rate.sleep()

    my_drone.land()

if __name__ == "__main__":
    rospy.init_node("drone_control")
    my_drone = DroneController()
    try:
        main()
    except KeyboardInterrupt:
        my_drone.land()
        # my_drone.rate.sleep()