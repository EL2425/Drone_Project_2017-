#!/usr/bin/env python

import numpy
import rospy
import sys

import pygame
from mocap_source_2 import Mocap, Body
from mocap_node.msg import State, MocapResp
from mocap_node.srv import *

class Drone:

    def __init__(self, drone_name):
        self.mocap = Mocap(host='192.168.1.10', info=1)
        rospy.init_node('mocap_sender')
        self.pub = rospy.Publisher('mocap_state', MocapResp, queue_size=10)
        self.rate = rospy.Rate(30)
        self.mocap_body = self.mocap.get_id_from_name(drone_name)        # Name defined in MoCap system


    def run(self):
        while not rospy.is_shutdown():
            resp = self.mocap.get_body(self.mocap_body)
            if resp == 'off':
                self.pub.publish(MocapResp(
                    State(0, 0, 0, 0, 0, 0),
                    False
                ))
            else:
                self.pub.publish(MocapResp(
                    State(
                        resp['x'], resp['y'], resp['z'],
                        -resp['pitch'], resp['roll'], resp['yaw']
                    ),
                    True
                ))
            self.rate.sleep()

if __name__ == '__main__':
    mocap_node = Drone(sys.argv[1])
    mocap_node.run()
