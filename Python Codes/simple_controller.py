#!/usr/bin/env python

import time
import sys
import tty
import termios
import select
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from el2425_drone_snr.msg import Ctrl

class cf2controller:
    
    def __init__(self, tf_prefix):
        self.fly_mode = False
        self.thrust = 49100.0
    
        rospy.init_node('el2425_drone_snr_' + tf_prefix)
        self.pub = rospy.Publisher(tf_prefix + '/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber(tf_prefix + '/cf_key', Ctrl, self.toggle_fly)

        linear = Vector3(0.0, 0.0, 49250.0)
        angular = Vector3(0.0, 0.0, 0.0)
        self.twist_fly = Twist(linear, angular)
        self.twist_stop = Twist(angular, angular)

        self.rate = rospy.Rate(100)

    def toggle_fly(self, data):
        self.fly_mode = data.fly_mode
        self.thrust = data.thrust
    
    def run(self):
        while not rospy.is_shutdown():
            if self.fly_mode:
                self.twist_fly.linear.z = self.thrust
                self.pub.publish(self.twist_fly)
            else:
                self.pub.publish(self.twist_stop)
            self.rate.sleep()


if __name__ == '__main__':
    ctrl = cf2controller(sys.argv[1])
    ctrl.run()