#!/usr/bin/env python

import numpy
import rospy
import sys

import pygame
from mocap_source_2 import Mocap, Body

#from el2425.msg import dronestate
from mocap_node.srv import *

from geometry_msgs.msg import PoseArray, Pose

class Drone:
	def __init__(self,tf_prefix):

		#initialize mocap connection
		self.mocap = Mocap(host = '192.168.1.199', info = 1)
		# Create a node
		rospy.init_node('mocap_sender', anonymous = True)
		# Create a topic to publish - Removing and defining it as a service
		#self.pub1 = rospy.Publisher('Mocapstate1', dronestate, queue_size = 1)
		self.mocap_body = {}	
		for i in range(0,len(tf_prefix)):
			rospy.Service('drone_states_' + tf_prefix[i],dronestaterequest,self.request_service)
			self.mocap_body[tf_prefix[i]] = self.mocap.get_id_from_name(tf_prefix[i])		# Name defined in MoCap system
		
		rospy.spin()

	def request_service(self,data):
		drone_1_state = self.mocap.get_body(self.mocap_body[data.drone_name])
		if drone_1_state == 'off':
			rospy.logwarn("drone" + "is not found")
		else:

			return dronestaterequestResponse(drone_1_state['x'],drone_1_state['y'],drone_1_state['z'],drone_1_state['yaw'],drone_1_state['pitch'],drone_1_state['roll'])    	




def funk():
	leninputarg = len(sys.argv)
	if (leninputarg > 2):
		allarg = [sys.argv[1]]
		for i in range(2,leninputarg):
			allarg += [sys.argv[i]]
	else:
		allarg = [sys.argv[1]]

	car = Drone(allarg)

if __name__ == '__main__':
	funk()
