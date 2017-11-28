#!/usr/bin/env python

import numpy
import rospy
import sys

import pygame
from mocap_source_2 import Mocap, Body
from mocap_node.srv import *
from geometry_msgs.msg import Twist, Vector3

class Drone:
    def __init__(self, drone_name):

        self.drone_name = drone_name
        self.mocap = Mocap(host='192.168.1.10', info=1)
        rospy.init_node('mocap_sender')
        rospy.Service('mocap_state', dronestaterequest, self.send_state)
        self.mocap_body = self.mocap.get_id_from_name(drone_name)        # Name defined in MoCap system


    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


    def send_state(self, data):
        state = self.mocap.get_body(self.mocap_body)
        if state == 'off':
            rospy.logwarn('Warning, drone ' + self.drone_name  + ' was not found')
            return dronestaterequestResponse(
                state=Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                valid=False
            )
        else:
            return dronestaterequestResponse(
                state=Twist(state)
                valid=True
            )        

if __name__ == '__main__':
    mocap_node = Drone(sys.argv[1])
    mocap_node.run()
