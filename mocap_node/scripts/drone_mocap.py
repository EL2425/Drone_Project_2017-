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
        self.srv = rospy.Service('mocap_srv', dronestaterequest, self.send_state)
        self.rate = rospy.Rate(30)
        self.mocap_body = self.mocap.get_id_from_name(drone_name)
        self.resp = 'Off'      # Name defined in MoCap system


    def run(self):
        while not rospy.is_shutdown():
            self.resp = self.mocap.get_body(self.mocap_body)
            if self.resp == 'off':
                self.pub.publish(MocapResp(
                    state=State(0, 0, 0, 0, 0, 0),
                    valid=False
                ))
            else:
                self.pub.publish(MocapResp(
                    state=State(
                        self.resp['x'], self.resp['y'], self.resp['z'],
                        -self.resp['pitch'], self.resp['roll'], self.resp['yaw']
                    ),
                    valid=True
                ))
            self.rate.sleep()


    def send_state(self, data):
        if self.resp == 'off':
            return {
                'state': State(0,0,0,0,0,0),
                'valid': False
            }
        return {
            'state': State( self.resp['x'], self.resp['y'], self.resp['z'],
                            0, 0, 0),
            'valid': True
        }

if __name__ == '__main__':
    mocap_node = Drone(sys.argv[1])
    mocap_node.run()
