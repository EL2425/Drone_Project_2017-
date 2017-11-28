#!/usr/bin/env python

import numpy
import rospy
import sys

import pygame
from mocap_source_2 import Mocap, Body
from mocap_node.srv import *
from geometry_msgs.msg import PoseArray, Pose

class Drone:
    def __init__(self, drone_name):

        self.mocap = Mocap(host='192.168.1.10', info=1)
        rospy.init_node('mocap_sender')
        self.mocap_body = {}
        rospy.Service('mocap_state', dronestaterequest, self.send_state)
        self.mocap_body = self.mocap.get_id_from_name(drone_name)        # Name defined in MoCap system


    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


    def send_state(self, data):
        state = self.mocap.get_body(self.mocap_body)
        if state == 'off':
            rospy.logwarn("drone is not found")
            #TODO: remove this from request and just fetch from controller node instead
            return dronestaterequestResponse(
                data.prev_x,
                data.prev_y,
                data.prev_z,
                data.prev_yaw,
                data.prev_pitch,
                data.prev_roll,
                True
            )
        else:
            return dronestaterequestResponse(
                state['x'],
                state['y'],
                state['z'],
                state['yaw'],
                -state['pitch'],  #TODO: why negative?
                state['roll'],
                False
            )        

if __name__ == '__main__':
    mocap_node = Drone(sys.argv[1])
    mocap_node.run()
