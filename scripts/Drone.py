#!/usr/bin/env/python

import rospy
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import *
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State



class Drone:


	def arm(self,value):
		self.arming_cmd = CommandBool()
		self.arming_cmd.value = value
		self.arming_client = rospy.ServiceProxy(str(self.id)+"/mavros/cmd/arming", CommandBool)
		while not self.arming_client(self.arming_cmd.value).success == True:
			self.rate.sleep()
		print("Arming/Disarming Completed")

	def get_id(self,msg):
		if self.id_state==0:
			self.id = "uav"+str(msg.data)
			self.id_state = 1
			print("ID is ", str(self.id))

	def setmode(self,MODE):
		for i in range(100):
			self.local_pos_pub.publish(self.pose)
			self.rate.sleep()
		print("ID of the drone is ", self.id)
		print("Mode is ",str(self.id)+"/mavros/set_mode")
		self.set_mode = SetMode()
		self.set_mode.custom_mode = MODE
		
		while(self.state.mode != MODE):
			self.set_mode_client(0,self.set_mode.custom_mode)
			print("setting mode", self.set_mode.custom_mode)
			print(self.state.mode, MODE)
			time.sleep(1)

	def update_pose(self,msg):
		self.pose = msg
		#print("pose updated")

	def get_dist_from_target(self):
		x = self.pose.pose.position.x
		y = self.pose.pose.position.y
		z = self.pose.pose.position.z
		xt = self.target.pose.position.x
		yt = self.target.pose.position.y
		zt = self.target.pose.position.z
		self.dist_from_target = ((x-xt)**2+(y-yt)**2+(z-zt)**2)**0.5
		return self.dist_from_target
	
	def check_reached(self):
		if self.get_dist_from_target() < 0.5:
			return 1
		else:
			return 0

	def state_cb(self,msg):
		self.state = msg
		self.mode = self.state.mode
		#print("state updated")

	def set_target(self,target):
		self.target = target
		#print("Within target functions" )
		self.local_pos_pub.publish(self.target)


	def __init__(self,drone_id):
		# self.id = 0
		self.id = drone_id
		self.id_state = 0
		self.id_pub = rospy.Publisher("ID", Int32)
		self.arming_cmd = CommandBool()
		self.arming_cmd.value=True
		rospy.init_node("controller", anonymous=True)
		rospy.Subscriber("request_ID", Int32, self.get_id, queue_size=1)
		time.sleep(1)
		while self.id_state == 0:
			self.id_pub.publish(self.id_state)
			print("Waiting for ID")
			time.sleep(0.5)

		
		self.rate = rospy.Rate(20)
		print("ID received", self.id)
		rospy.Subscriber(str(self.id)+"/mavros/state", State, self.state_cb)
		rospy.Subscriber(str(self.id)+"/mavros/local_position/pose", PoseStamped, self.update_pose)
		self.set_mode_client = rospy.ServiceProxy(str(self.id)+"/mavros/set_mode", SetMode)
		self.local_pos_pub = rospy.Publisher(str(self.id)+"/mavros/setpoint_position/local", PoseStamped, queue_size=10)
		self.state = State()
		self.pose = PoseStamped()
		self.mode = self.state.mode
		self.target = PoseStamped()
		time.sleep(2)
