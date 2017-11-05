#!/usr/bin/env python

import rospy
import sys
import tty
import select
import termios
from el2425_drone_snr.msg import Ctrl

def keylistener(tf_prefix):
    rospy.init_node('cf_keyboard_' + tf_prefix)
    pub = rospy.Publisher(tf_prefix + '/cf_key', Ctrl, queue_size=10)

    settings = termios.tcgetattr(sys.stdin)
   
    fly_mode = False
    thrust = 49000.0

    TOGGLE_FLY_KEY = ' '
    INCREASE_THRUST_KEY = 'w'
    DECREASE_THRUST_KEY = 's'

    thrust_inc = 50.0
 
    msg = Ctrl()

    while True:

        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        if key == '\x03':
            break
        elif key == TOGGLE_FLY_KEY:
            fly_mode = not fly_mode
        elif key == INCREASE_THRUST_KEY:
            thrust = thrust + thrust_inc
        elif key == DECREASE_THRUST_KEY:
            thrust = thrust - thrust_inc
        else:
            print('err, unknown key')
        print('fly mode:', fly_mode, 'thrust:', thrust)
        
        msg.fly_mode = fly_mode
        msg.thrust = thrust

        pub.publish(msg)

if __name__ == '__main__':
   keylistener(sys.argv[1])
