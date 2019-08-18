#!/usr/bin/env/python

"""
General Drone Class for MAVROS.

Author: Harshal Deshpande
"""


import rospy
import time
from std_msgs.msg import Int32, Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Pose
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
# from mavros_msgs.srv import CommandTOL


class DroneController:
	def __init__(self):
		"""
		Different Modes:
		Stabilize = 0
		Guided = 4
		Land = 9
		"""
		# self.pos_flag = 0
		self.pose = PoseStamped()
		self.current_state = State()

		self.pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
		self.state_sub = rospy.Subscriber('mavros/state', State, self.state_cb)

		self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=5)
		self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=5)

		self.mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
		# rospy.wait_for_service("/mavros/cmd/arming")
		self.arm_service = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
		self.rate = rospy.Rate(10)
		# self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

	def state_cb(self,state):
		self.current_state = state

	def pose_callback(self,data):
		self.pose = data
		self.rate.sleep()		
	
	def set_vel_cmd(self, vx, vy, vz, avx=0, avy=0, avz=0):
		"""
		To send the velocity commands the vehicle must be in GUIDED mode (mode = 4)
		"""
		cmd_vel = Twist()

		cmd_vel.linear.x = vx
		cmd_vel.linear.y = vy
		cmd_vel.linear.z = vz

		cmd_vel.angular.x = avx
		cmd_vel.angular.y = avy
		cmd_vel.angular.z = avz

		self.cmd_vel_pub.publish(cmd_vel)

	def set_pos(self,x,y,z):
		"""
		This function is used to send the position commands to mavros.
		"""
		set_pose = PoseStamped()
		set_pose.header = Header()
		# set_pose.header = self.pose.header
		set_pose.header.stamp = rospy.Time.now()

		set_pose.pose.position.x = x
		set_pose.pose.position.y = y
		set_pose.pose.position.z = z

		set_pose.pose.orientation.x = 0
		set_pose.pose.orientation.y = 0
		set_pose.pose.orientation.z = 0
		set_pose.pose.orientation.w = 0 

		self.cmd_pos_pub.publish(set_pose)

	def arm(self):
		"""
		This function is used to arm the drone.
		"""
		result = self.arm_service(True)
		print("Arming: ",result)

	def disarm(self):
		"""
		This function is used to disarm the drone.
		"""
		result = self.arm_service(False)
		print("Disarming: ", result)

	def land(self):
		"""
		This function is used to land the drone.
		"""
		result = self.mode_service(custom_mode="9")
		self.disarm()

	def fill_buffer(self):
		"""
		This function fills the starting buffer which is necessary for the 
		OFFBOARD mode.
		"""
		for i in range(100):
			self.set_pos(0,0,0)
		print("Buffer Filled")
