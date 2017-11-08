#!/usr/bin/env python

import sys
import numpy as np
import rospy

from geometry_msgs.msg import Twist, Vector3
from mocap_node.srv import dronestaterequest

class DroneController:

    def __init__(self, tf_prefix):
        self.drone_name = 'Crazyflie2'
        self.Kp = 5000
        self.Kd = 2
        self.Ki = 2

        self.x_tgt = 0
        self.y_tgt = 0
        self.z_tgt = 1

        rospy.init_node('DroneController')
        self.pub = rospy.Publisher(
            tf_prefix + '/cmd_vel',
            Twist,
            queue_size=10
        )
        self.state_srv = rospy.ServiceProxy(
            'drone_states_' + tf_prefix,
            dronestaterequest
        )
        self.rate = rospy.Rate(50)

    def get_drone_state(self):
        state = self.state_srv(drone_name=self.drone_name)
        return state.x, state.y, state.z

    def run(self):
        while not rospy.is_shutdown():
            x, y, z = self.get_drone_state()
            e_x = self.x_tgt - x
            e_y = self.y_tgt - y
            e_z = self.z_tgt - z

            lin = Vector3(0, 0, 30000 + self.Kp*e_z)
            ang = Vector3(0, 0, 0)
            self.pub.publish(Twist(lin, ang))

            self.rate.sleep()

if __name__ == '__main__':
    dc = DroneController(sys.argv[1])
    dc.run()

