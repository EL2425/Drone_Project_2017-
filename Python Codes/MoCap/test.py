#!/usr/bin/env python

import numpy
import rospy
import sys

import pygame
from mocap_source_2 import Mocap, Body

from simulation.srv import SetSpeed
from simulation.srv import SetLookahead
from simulation.srv import SetDist
from simulation.srv import SetStop


from simulation.msg import VehicleStateInternal, VehicleState, VehicleReference
from simulation.msg import TimeTopic
from simulation.msg import drive_param


from geometry_msgs.msg import PoseArray, Pose

class Truck:
    def __init__(self):

        #initialize mocap connection
        self.mocap = Mocap(host = 'Inet-PC', info = 1)

        rospy.init_node('mocap_sender', anonymous = True)
        self.pub1 = rospy.Publisher('Mocapstate1', VehicleState, queue_size = 1)
        self.pub2 = rospy.Publisher('Mocapstate2', VehicleState, queue_size = 1)
        self.pub3 = rospy.Publisher('Mocapstate3', VehicleState, queue_size = 1)

        #self.mocap_body1 =self.mocap.get_id_from_name('TruckVehicle1')
        #self.mocap_body2 =self.mocap.get_id_from_name('TruckVehicle2')
        self.mocap_body1 = self.mocap.get_id_from_name('MiniTruck1')
        self.mocap_body2 = self.mocap.get_id_from_name('MiniTruck2')
        self.mocap_body3 = self.mocap.get_id_from_name('MiniTruck3')

        self.sender()
        rospy.spin()


    def sender(self):
        rate = rospy.Rate(40)

        while not rospy.is_shutdown():
            truck_state1 = self.mocap.get_body(self.mocap_body1)
            truck_state2 = self.mocap.get_body(self.mocap_body2)
            truck_state3 = self.mocap.get_body(self.mocap_body3)
            if truck_state1 == 'off':
                rospy.logwarn("truck 1 is not found")
            else:
                msg1 = VehicleState()
                msg1.x = truck_state1['x']
                msg1.y = truck_state1['y']
                msg1.yaw = truck_state1['yaw']
                self.pub1.publish(msg1)

            if truck_state2 == 'off':
                rospy.logwarn("truck 2 is not found")
            else:
                msg2 = VehicleState()
                msg2.x = truck_state2['x']
                msg2.y = truck_state2['y']
                msg2.yaw = truck_state2['yaw']
                self.pub2.publish(msg2)

            if truck_state3 == 'off':
                rospy.logwarn("truck 3 is not found")
            else:
                msg3 = VehicleState()
                msg3.x = truck_state3['x']
                msg3.y = truck_state3['y']
                msg3.yaw = truck_state3['yaw']
                self.pub3.publish(msg3)

            rate.sleep()




def funk():
    car = Truck()

if __name__ == '__main__':
    funk()