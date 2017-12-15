#! /usr/bin/env python


# Will output a dictionary - With Drone name - Mode, Target
import numpy as np
import rospy
import sys
from rosplan_dispatch_msgs.msg import *
from planner.msg import action_msg, action_dict
from geometry_msgs.msg import Vector3
import pickle

class ActionProcess:

	def __init__(self):
		rospy.init_node('action_server')
		self.action_dispatch = rospy.Subscriber('/kcl_rosplan/plan', CompletePlan, self.get_complete_plan)
		self.pub_action_feedback = rospy.Publisher('/kcl_rosplan/action_feedback', ActionFeedback, queue_size=1)
		self.pub_action_dictionary = rospy.Publisher('action_dictionary', action_dict, queue_size=1)
		self.rate = rospy.Rate(1)
		pickle_in = open("/home/el2425/catkin_ws/src/Drone_Project_2017-/planner/common/waypoints.pickle","rb")
		self.name_wp = pickle.load(pickle_in)
		self.start = 0
		rospy.loginfo("Action Server Initilized")
		self.action_feedback = {}


	def get_complete_plan(self,data_plan):
		self.complete_plan = data_plan
		self.NumberofActions=len(self.complete_plan.plan)
		self.action_dispatch_times = np.zeros((self.NumberofActions,1))
		Test_dict = {}
		for i in range(0,len(self.complete_plan.plan)):
			self.action_dispatch_times[i] = round(self.complete_plan.plan[i].dispatch_time)
			key = 'action_id_' + str(i)
			Test_dict[key] = 0
		rospy.set_param('completed_actions',Test_dict)
		self.action_feedback = Test_dict
		self.start = 1
		rospy.loginfo("Fetched CompletePlan")

	def dispatch_plan(self):
		self.Dispatch_Publish = set()
		Sucess = 0
		Get_all_actions = rospy.get_param('completed_actions')
		current_action_num = sum(Get_all_actions.values())
		hold_dispatch = ActionDispatch
		# Send action feedback
		for i in range(0,self.NumberofActions):
			if(Get_all_actions['action_id_' + str(i)] == 1):
				if(self.action_feedback['action_id_' + str(i)] == 0):
					self.pub_action_feedback.publish(ActionFeedback(i,'action acheieved',[]))
					self.action_feedback['action_id_' + str(i)] = 1
					rospy.sleep(0.01)

		if(current_action_num != self.NumberofActions):
			store_dispatchactions_numbers = self.action_dispatch_times == self.action_dispatch_times[current_action_num]
			for i in range(0,len(store_dispatchactions_numbers)):
				if(store_dispatchactions_numbers[i] and Get_all_actions['action_id_' + str(i)] == 0):
					drone = {}
					hold_dispatch = [self.complete_plan.plan[i]]
					# Extract Mode and Covert to WayPoints
					drone['packet_valid'] = 1
					drone['mode'] = hold_dispatch[0].name
					drone['name'] = hold_dispatch[0].parameters[0].value
					drone['action_id'] = hold_dispatch[0].action_id
					if(hold_dispatch[0].name == 'load'):
						drone['waypoint'] = self.name_wp[hold_dispatch[0].parameters[2].value]
						drone['waypoint_valid'] = 0
					elif(hold_dispatch[0].name == 'takeoff'):
						drone['waypoint'] = self.name_wp[drone['name'] + '_takeoff']
						drone['waypoint_valid'] = 1
					elif(hold_dispatch[0].name == 'move'):
						drone['waypoint'] = self.name_wp[hold_dispatch[0].parameters[2].value]
						drone['waypoint_valid'] = 1
					elif(hold_dispatch[0].name == 'land'):
						drone['waypoint'] = {'x': 0,'y': 0,'z': 0}
						drone['waypoint_valid'] = 0
					elif(hold_dispatch[0].name == 'unload'):
						drone['waypoint'] = {'x': 0,'y': 0,'z': 0}
						drone['waypoint_valid'] = 0
					else:
						drone['packet_valid'] = 0
						rospy.loginfo('Unknown Mode Encountered')
						drone['waypoint'] = {'x': 0,'y': 0,'z': 0}
						drone['waypoint_valid'] = 0

					load_action_msg = action_msg(drone['packet_valid'],drone['name'],drone['mode'],drone['waypoint_valid'],drone['waypoint']['x'],drone['waypoint']['y'],drone['waypoint']['z'],drone['action_id'])
					self.Dispatch_Publish.update([load_action_msg])
		else:
			rospy.loginfo("Actions Completed")
			Sucess = 1

		if(not self.Dispatch_Publish):
			load_action_msg = action_msg(0,None,None,0,None,None,None,None)
			self.Dispatch_Publish.update([load_action_msg])
			if(Sucess == 0):
				rospy.loginfo("Error Occured")
		self.pub_action_dictionary.publish(action_dict(self.Dispatch_Publish))

		#drone['waypoint'] = {'x': 0,'y': 0,'z': 0}
		#print(drone['waypoint']['x'])

	def run(self):
		while not rospy.is_shutdown():
			if(self.start == 1):
				self.dispatch_plan()
				#rospy.loginfo("Action Server Running")
			self.rate.sleep()


if __name__ == '__main__':
	actionclass = ActionProcess()
	actionclass.run()
