#!/usr/bin/env python
########
# Things to change when transfering code to new drone:
# self.vehical_id
# set_offset
# desired_pose (inside __init__)
# node name
# instance of Drone object
########

# Branch: 2D_algorithm

from Drone import *
import math
import numpy
import argparse

from std_msgs.msg import Int16,Float32MultiArray

# parser = argparse.ArgumentParser(description = 'Get Required data from the user')
# parser.add_argument('--id', help="Drone ID")
# args = parser.parse_args()

ID = 1

class DroneMsg(object):
	def __init__(self, id, x, y, z, reached, dir_ip):
		self.id = id
		self.x = x
		self.y = y
		self.z = z
		self.reached = reached
		self.dir = dir_ip


class DroneControl1(object):
	def __init__(self):
		self.initialize_variables()
		self.initialize_pub_sub()
		# self.uav = Drone(self.vehical_id,2*ID,0) # id, x, y
		self.uav = Drone(self.vehical_id)
		self.uav.desired_pose.pose.position.x = 1*ID
		self.uav.desired_pose.pose.position.y = 0
		self.uav.desired_pose.pose.position.z = 3
		self.uav.command_format = 0;  # Postion
		self.uav.set_offset(1*ID,0)  
		self.uav.start_controller = True  # Starts giving the commands. 


		# rospy.Timer(rospy.Duration(0.025), self.controller)
		# rospy.Timer(rospy.Duration(0.5), self.update_parameters)
		rospy.Timer(rospy.Duration(0.001), self.get_sensor_data)
		# rospy.Timer(rospy.Duration(0.005), self.trajectory_planner)


	def initialize_pub_sub(self):
		self.reach_status_pub = rospy.Publisher('/uav'+str(self.vehical_id)+'/reached', Int16, queue_size=10)
		self.status_pub =  rospy.Publisher('/uav'+str(self.vehical_id)+'/status', Int16, queue_size=10)
		self.internal_pub = rospy.Publisher('/uav'+str(self.vehical_id)+'/dcom', Int16, queue_size=10)

		rospy.Subscriber('/uav'+str(self.vehical_id)+'/com', Float32MultiArray, self.internal_data_cb)

	def initialize_variables(self):
		self.vehical_id = ID
		self.gas_center = [6,6,3] # x,y,z
		self.gas_radius = 2.0
		# self.others_data = [-1.0, 0., 0., 0., 0.]
		self.others_data = []
		# self.other_x = 0.
		# self.other_y = 0.
		# self.other_z = 0.
		# self.other_reached = 0
		# self.other_id = -1.0
		self.reached = 0

		self.status = 0  # 0 = too close or too far | 1 = on trajectory | 2 = on boundary

		# self.uav_inner_lim = 9.0  # How close 2 uavs can come
		# self.uav_inner_rad = self.uav_inner_lim + 1.0  # How far the uavs go if they come too close
		# self.push_out_dist = 1.0  # How much to move out in one step if gone inside the bloom

		# self.inner_threshold = 10.5  # Inner limit of the bloom
		# self.outer_threshold = 11.5  # Outer limit of the bloom

		# self.just_reached = True

		self.pos_hold_x = 0.0
		self.pos_hold_y = 0.0

		self.centers_list = {}  # [x,y of each of the center]

		self.start_mission = 1  # Only when this is true drone will start executing the mission. Till then it will hover at takeoff location

		self.sensing_radius = 2*math.sqrt(2)  # Distance between 2 drones on the boundary
		self.inner_radius = self.sensing_radius - 0.5
		self.outer_radius = self.sensing_radius + 0.5
		

		self.des_x = 0.0
		self.des_y = 0.0
		self.des_vel_x = 0.0
		self.des_vel_y = 0.0

		self.someone_reached = False  # True when one of the drone reaches the boundary
									  # Required for the drones to deside the setpoint when not reached.

		self.lead_drone_center = [self.gas_center[0], self.gas_center[1]]

		self.first_reached = False
		self.direction = 0
		self.counter = 1

		self.once_reached = False
		self.v0 = 1


		# self.uav_switch = [-1,0,0,0,0]
		for i in range(0,5):
			self.others_data.append(DroneMsg(i,0.,0.,0.,0,0))

	def internal_data_cb(self, data):
		id_recieved = data.data[0]
		if id_recieved != self.vehical_id:
			self.others_data[int(id_recieved)].id = id_recieved
			self.others_data[int(id_recieved)].x = data.data[1]
			self.others_data[int(id_recieved)].y = data.data[2]
			self.others_data[int(id_recieved)].z = data.data[3]
			self.others_data[int(id_recieved)].reached = data.data[4]
			self.others_data[int(id_recieved)].dir = data.data[5]
			# if self.others_data[int(id_recieved)].reached == 1:
			self.centers_list[int(id_recieved)] = [self.others_data[int(id_recieved)].x, self.others_data[int(id_recieved)].y]
		

	def update_parameters(self, event):
		if rospy.has_param('/uav'+str(self.vehical_id)+'/start_mission'):
			self.start_mission = int(rospy.get_param('/uav'+str(self.vehical_id)+'/start_mission'))
		else:
			rospy.set_param('/uav'+str(self.vehical_id)+'/start_mission', '0')

	def get_sensor_data(self,event):  # This is going to be the callback of the sensor node in the real case
		gas_center_dist = math.sqrt((self.gas_center[0]-self.uav.lpose_x)**2 + (self.gas_center[1]-self.uav.lpose_y)**2)
		if gas_center_dist < (self.gas_radius+1) and gas_center_dist > (self.gas_radius-1):
			if self.once_reached == False:
				self.once_reached = True
			if self.just_reached:
				self.pos_hold_x = self.uav.lpose_x
				self.pos_hold_y = self.uav.lpose_y
				self.just_reached = False
			self.uav.reached = 1
		else:
			if not self.once_reached:
				self.just_reached = True
				self.uav.reached = 0

		self.reach_status_pub.publish(self.uav.reached)

		if self.once_reached:
			self.uav.reached = 1
			self.on_boundary_controller()
		else:
			self.on_the_way_controller()

	def pos_hold(self):
		self.uav.desired_pose.pose.position.x = self.pos_hold_x
		self.uav.desired_pose.pose.position.y = self.pos_hold_y
		vel_to_publish = TwistStamped()
		self.uav.vel_publisher.publish(vel_to_publish)

	def on_boundary_controller(self):
		# print "I am reached"
		self.pos_hold()
		if self.first_reached:
			print "Fisrt"
			dist_list = []
			id_list = []
			for c in self.centers_list:
				dist_list.append(math.sqrt((self.centers_list[c][0]-self.uav.lpose_x)**2 + (self.centers_list[c][1]-self.uav.lpose_y)**2))
				id_list.append(c)
			center_dist = min(dist_list)
			i=0
			while 1:
				print self.others_data[(id_list[(dist_list.index(center_dist))])+i].id
				if (not self.others_data[(id_list[(dist_list.index(center_dist))])+i].reached):
					print "Giving direction"
					self.counter = -self.counter
					while not self.others_data[(id_list[(dist_list.index(center_dist))])+i].reached:
						dir_obj = Int16()
						dir_obj.data = self.counter
						self.internal_pub.publish(dir_obj)
					break
				i = i+1



	def on_the_way_controller(self):
		# self.uav.
		# self.uav.desired_vel.twist.linear.x = 1
			##########################################################################
		# self.get_sensor_data(1)

		# Finding if someone has reahced
		for i in range(1,4):
			if self.others_data[i].reached:
				self.someone_reached = True
				break
			else:
				self.someone_reached = False

		print "someone_reached:", self.someone_reached

		# Controller will start giving commands only after the start_mission parameter is set
		if self.start_mission:
			if not self.someone_reached:  # If no one has reached
				self.uav.command_format = 0
				self.first_reached = True
				if not self.uav.reached:  # Till self not reached
					phi = numpy.arctan2((self.gas_center[1]-self.uav.lpose_y), (self.gas_center[0]-self.uav.lpose_x))
					self.uav.desired_pose.pose.position.x = self.gas_center[0] - (self.gas_radius-1)*math.cos(phi)
					self.uav.desired_pose.pose.position.y = self.gas_center[1] - (self.gas_radius-1)*math.sin(phi)
				else:  # After reaching
					print "1"
					self.first_reached = True
					self.pos_hold()
					

			else: # Someone has already reached
				self.first_reached = False
				dist_list = []
				id_list = []
				for c in self.centers_list:
					dist_list.append(math.sqrt((self.centers_list[c][0]-self.uav.lpose_x)**2 + (self.centers_list[c][1]-self.uav.lpose_y)**2))
					id_list.append(c)
				center_dist = min(dist_list)

				self.lead_drone_center = self.centers_list[id_list[(dist_list.index(center_dist))]]
				if self.others_data[id_list[(dist_list.index(center_dist))]].dir == 1:
					self.direction = 1
				elif self.others_data[id_list[(dist_list.index(center_dist))]].dir == -1:
					self.direction = -1
				dir_obj = Int16()
				dir_obj.data = self.direction
				self.internal_pub.publish(dir_obj)

				if not self.uav.reached:
					if center_dist < self.inner_radius:
						self.status = 0
					elif center_dist < self.outer_radius:
						self.status = 1
					else:
						self.status = 0
				else:
					self.status = 2

				# Giving commands:
				
				if self.status == 0:
					theta = numpy.arctan2((self.lead_drone_center[1]-self.uav.lpose_y), (self.lead_drone_center[0]-self.uav.lpose_x))
					self.des_x = self.lead_drone_center[0] - (self.sensing_radius)*math.cos(theta)
					self.des_y = self.lead_drone_center[1] - (self.sensing_radius)*math.sin(theta)
					self.uav.command_format = 0
					self.uav.desired_pose.pose.position.x = self.des_x
					self.uav.desired_pose.pose.position.y = self.des_y
				elif self.status == 1:
					theta2 = numpy.arctan2((self.lead_drone_center[1]-self.uav.lpose_y), (self.lead_drone_center[0]-self.uav.lpose_x))
					self.des_vel_x = self.direction*(self.v0*math.sin(theta2))
					self.des_vel_y = self.direction*(-self.v0*math.cos(theta2))
					self.uav.command_format = 1
					self.uav.desired_vel.twist.linear.x = self.des_vel_x
					self.uav.desired_vel.twist.linear.y = self.des_vel_y
				elif self.status == 2:
					self.uav.command_format = 0
					self.uav.desired_pose.pose.position.x = self.pos_hold_x
					self.uav.desired_pose.pose.position.y = self.pos_hold_y

			print "status:", self.status
			print "dir: ", self.direction
			# print "des vel:", self.des_vel_x, self.des_vel_y
			print "des_pos:", self.uav.desired_pose.pose.position.x, self.uav.desired_pose.pose.position.y
###############################################################################

		# 		self.lead_drone_center = self.centers_list[(dist_list.index(center_dist))]
		# 		if not self.uav.reached:
		# 			if center_dist < self.inner_radius:
		# 				self.status = 0
		# 			elif center_dist < self.outer_radius:
		# 				self.status = 1
		# 			else:
		# 				self.status = 0
		# 		else:
		# 			self.status = 2




		# if self.start_mission:
		# 	if self.status == 0:
		# 		phi = numpy.arctan2((self.lead_drone_center[1]-self.uav.lpose_y), (self.lead_drone_center[0]-self.uav.lpose_x))
		# 		self.des_x = self.lead_drone_center[0] - (self.sensing_radius-1)*math.cos(phi)
		# 		self.des_y = self.lead_drone_center[1] - (self.sensing_radius-1)*math.sin(phi)
		# 		self.uav.command_format = 0
		# 		self.uav.desired_pose.pose.position.x = self.des_x
		# 		self.uav.desired_pose.pose.position.y = self.des_y
		# 	elif self.status == 1:
		# 		phi = numpy.arctan2((self.lead_drone_center[1]-self.uav.lpose_y), (self.lead_drone_center[0]-self.uav.lpose_x))
		# 		self.des_vel_x = self.v0*math.sin(phi)
		# 		self.des_vel_y = -self.v0*math.cos(phi)
		# 		self.command_format = 1
		# 		self.uav.desired_vel.twist.linear.x = self.des_vel_x
		# 		self.uav.desired_vel.twist.linear.y = self.des_vel_y
		# 	elif self.status == 2:
		# 		self.uav.command_format = 0
		# 		self.uav.desired_pose.pose.position.x = self.pos_hold_x
		# 		self.uav.desired_pose.pose.position.y = self.pos_hold_y
			# theta = numpy.arctan2((center[1]-lpose.y), (center[0]-lpose.x))
	  #       vel_x = v0*math.sin(theta)
	  #       vel_y = -v0*math.cos(theta)

			# phi = numpy.arctan2((center[1]-lpose.y),  (center[0]-lpose.x))
	  #       des_x = center[0] - (radius-1)*math.cos(phi)
	  #       des_y = center[1] - (radius-1)*math.sin(phi)


		

def main():
	rospy.init_node('uav'+str(ID))

	d1 = DroneControl1()
	print("Drone1 started")

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("[Controller]: Shutting down controller node")

if __name__ == '__main__':
	main()

