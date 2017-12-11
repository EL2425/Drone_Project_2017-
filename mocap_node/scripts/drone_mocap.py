#!/usr/bin/env python

import numpy
import rospy
import sys

import pygame
from mocap_source_2 import Mocap, Body
from mocap_node.msg import State, MocapResp
from mocap_node.srv import *

class MocapNode:
    def __init__(self):
        rospy.init_node('mocap_node')
        self.host = Mocap(host='192.168.1.10', info=1)
        self.srv_add = rospy.Service('add_drone', AddDrone, self.add_drone)
        self.srv_find = rospy.Service('find_drones', FindDrones, self.find_drones)
        self.srv_add_all = rospy.Service('add_all_drones', AddAll, self.add_all_drones)
        self.drones = []
        self.rate = rospy.Rate(30)

    def add_drone(self, data):
        mocap_id = self.host.get_id_from_name(data.name)
        drone = Drone(data.name, mocap_id)
        self.drones.append(drone)
        return True

    def find_drones(self, data):
        all_bodies = self.host.get_list_bodies()
        drones = []
        for b in all_bodies:
            name_match = b.startswith('crazyflie')
            b_id = self.host.get_id_from_name(b)
            available = self.host.get_body(b_id)
            if available and name_match:
                drones.append(b)
        return FindDronesResponse(drones)

    def add_all_drones(self, data):
        response = self.find_drones(None)
        for d in response.drones:
            mocap_id = self.host.get_id_from_name(d)
            drone = Drone(d, mocap_id)
            self.drones.append(drone)
        return True

    def run(self):
        while not rospy.is_shutdown():
            [d.update(self.host.get_body(d.id)) for d in self.drones]
            self.rate.sleep()


class Drone:
    def __init__(self, name, mocap_id):
        self.name = name
        self.id = mocap_id
        self.resp = 'off'
        self.pub = rospy.Publisher('/' + name + '/mocap_state', MocapResp, queue_size=10)
        self.srv = rospy.Service('/' + name + '/mocap_srv', dronestaterequest, self.send_state)

    def update(self, resp):
        self.resp = resp
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
    mocap = MocapNode()
    mocap.run()
