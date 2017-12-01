#!/usr/bin/env python

import numpy as np
import rospy
import sys
from trajectory_generator.srv import *
from geometry_msgs.msg import Twist, Vector3
from mocap_node.srv import dronestaterequest
from mocap_node.msg import State, MocapResp
from drone_controller.srv import SetMode
from crazyflie_driver.srv import UpdateParams

class Controller:

    def __init__(self, tf_prefix):

        self.tf_prefix = tf_prefix

        rospy.init_node('drone_controller')

        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub_mocap = rospy.Subscriber('mocap_state', MocapResp, self.get_mocap)
        self.pub_trajgen_states = rospy.Publisher('targetstates', State, queue_size=10)
        self.sub_waypoint = rospy.Subscriber('waypoints', Twist, self.set_waypoint)
        self.rate = rospy.Rate(30)

        self.mode_srv = rospy.Service('SetMode', SetMode, self.set_flight_mode)

        # Initialize the zero-message
        zerosstop = Vector3(0.0, 0.0, 0.0)
        self.twist_stop = Twist(zerosstop, zerosstop)

        # Intialize the PID controllers
        self.pitchcontrol = pid_controller(tf_prefix, 'X')
        self.rollcontrol = pid_controller(tf_prefix, 'Y')
        self.yawcontrol = pid_controller(tf_prefix, 'Yaw')
        self.thrustcontrol = pid_controller(tf_prefix, 'Z')

        # Intialize Trajectory Generator States - Will only be used if the trajectory gen. function is invoked.
        self.previous_trajgen_state = {
            'X': 0.0,
            'Y': 0.0,
            'Z': 0.0
        }

        # Intialize previous mocap states - Incase if the drone is not found the previous state are returned
        self.mocap_state = State(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        # Safety Variable for Mocap - counter for lost mocap frames
        self.invalid_mocap_counter = 0
        self.INVALID_MOCAP_THRESH = 30

        # If the controller enters Take Off Mode - It will go to these states x,y,z - Yaw by default set to zero.
        # TODO: clean up how the states are expressed in the parameters
        self.takeoff_states = np.array(rospy.get_param('takeoff_states'))
        self.takeoff_states = State(
            x=self.takeoff_states[0],
            y=self.takeoff_states[1],
            z=self.takeoff_states[2],
            pitch=0.0,
            roll=0.0,
            yaw=0.0
        )
        self.landing_states = np.array(rospy.get_param('landing_states'))
        self.landing_states = State(
            x=self.landing_states[0],
            y=self.landing_states[1],
            z=self.landing_states[2],
            pitch=0.0,
            roll=0.0,
            yaw=0.0
        )
        self.fixed_state = np.array(rospy.get_param('FixedWayPointYaw'))
        self.fixed_state = State(
            x=self.fixed_state[0],
            y=self.fixed_state[1],
            z=self.fixed_state[2],
            pitch=0.0,
            roll=0.0,
            yaw=self.fixed_state[3]
        )

        self.waypoint = State(0, 0, 0, 0, 0, 0)

        # Initialize flightmode - start controller in 'Stop'-mode
        self.flightmode = 'Stop'
        # Possible flightmodes available
        self.modes = [
            'TakeOff',
            'FollowWayPoint',
            'FixedWayPoint',
            'Waypoint',
            'Landing',
            'Stop'
        ]

        self._update_params = rospy.ServiceProxy('update_params', UpdateParams)


    def get_mocap(self, resp):
        if not resp.valid:
            self.invalid_mocap_counter += 1
            rospy.loginfo(self.tf_prefix + ' not found by mocap')
            if self.invalid_mocap_counter >= self.INVALID_MOCAP_THRESH:
                rospy.signal_shutdown(self.tf_prefix + ' not found for long time')
        else:
            self.invalid_mocap_counter = 0
            resp.state.pitch = resp.state.pitch*np.pi/180
            resp.state.roll = resp.state.roll*np.pi/180
            resp.state.yaw = resp.state.yaw*np.pi/180
            self.mocap_state = resp.state


    def get_target_states(self,CurrentROSTime, PrevPosX, PrevPosY, PrevPosZ): # CurrentROSTime - Started from zero when the controller is started
        rospy.wait_for_service('generate_state_' + self.tf_prefix)
        try:
            generatestate = rospy.ServiceProxy('generate_state_' + self.tf_prefix, GetStates)
            states = generatestate(CurrentROSTime, PrevPosX, PrevPosY, PrevPosZ)
            return states
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def set_waypoint(self, data):
        self.waypoint = State(data.linear.x, data.linear.y, data.linear.z,
                              0.0, 0.0, 0.0)

    def get_target_state(self):

        if self.flightmode == 'TakeOff':
            return self.takeoff_states
        elif self.flightmode == 'Landing':
            return self.landing_states
        elif self.flightmode == 'FixedWayPoint':
            return self.fixed_state
        elif self.flightmode == 'FollowWayPoint':
            traj_resp = self.get_target_states(
                rospy.get_time() - self.trajectory_tstart,
                self.previous_trajgen_state['X'],
                self.previous_trajgen_state['Y'],
                self.previous_trajgen_state['Z']
            )
            return State(
                x=traj_resp.PosX,
                y=traj_resp.PosY,
                z=traj_resp.PosZ,
                pitch=0,
                roll=0,
                yaw=0
            )
        elif self.flightmode == 'Waypoint':
            return self.waypoint
        return self.mocap_state

    def set_flight_mode(self, args):
        if args.mode in self.modes:
            if self.flightmode == 'Stop' and args.mode != 'Stop':
                self.pid_rest_controller()
            if args.mode == 'FollowWayPoint' and not self.flightmode == 'FollowWayPoint':
                self.trajectory_tstart = rospy.get_time()
            self.flightmode = args.mode
            rospy.loginfo('Flightmode ' + str(self.flightmode) + ' entered')
        else:
            self.flightmode = 'Unknown'
            rospy.loginfo('Unknown flightmode entered')
        return True

    def run(self):

        while not rospy.is_shutdown():
            current_state = self.mocap_state
            target_state = self.get_target_state()

            if not self.flightmode == 'Stop':
                out_pitch = self.pitchcontrol.pid_calculate(target_state.x, current_state.x)
                out_roll = self.rollcontrol.pid_calculate(target_state.y, current_state.y)
                out_yaw = self.yawcontrol.pid_calculate(target_state.yaw, current_state.yaw*180/np.pi)
                out_thrust = self.thrustcontrol.pid_calculate(target_state.z, current_state.z)

                # TODO: check if these are RADs or DEGs
                ctheta = np.cos(current_state.pitch)
                stheta = np.sin(current_state.pitch)
                cpsi = np.cos(current_state.yaw)
                spsi = np.sin(current_state.yaw)
                cphi = np.cos(current_state.roll)
                sphi = np.sin(current_state.roll)

                rot_matrix = np.array([
                    [cpsi, spsi, 0.0],
                    [-spsi, cpsi, 0.0],
                    [0.0, 0.0, 1.0]
                ])

                # rot_matrx takes from original frame to transformed frame.
                rot_vals = np.dot(
                    rot_matrix,
                    np.array([
                        out_pitch,
                        out_roll,
                        out_yaw
                    ])
                )
                #rotated_values[2,0] = 0
                linear = Vector3(rot_vals[0], -rot_vals[1], out_thrust+43000.0)
                angular = Vector3(0.0, 0.0, -rot_vals[2])
                twist_fly = Twist(linear, angular)

            else:
                twist_fly = self.twist_stop

            self.pub_cmd_vel.publish(twist_fly) # Publisher For CrazyFlie
            self.rate.sleep()

            self.pub_trajgen_states.publish(target_state)

    def pid_rest_controller(self):
        self.pitchcontrol.reset()
        self.rollcontrol.reset()
        self.yawcontrol.reset()
        self.thrustcontrol.reset()



class pid_controller:

    def __init__(self, tf_prefix, direction):

        # Load parameters from ROS
        params = rospy.get_param('PIDs/' + direction)
        self.Kp = params['kp']
        self.Kd = params['kd']
        self.Ki = params['ki']
        self.integratorMin = params['integratorMin']
        self.integratorMax = params['integratorMax']
        self.minOutput = params['minOutput']
        self.maxOutput = params['maxOutput']

        # Initialize errors and integrals
        self.integral = 0.0
        self.previousTime = rospy.get_time()
        self.previousError = 0.0

    def pid_calculate(self, target, current):

        time = rospy.get_time()
        dt = time - self.previousTime
        error = target - current
        integral = self.integral + error*dt
        integral = max(min(integral, self.integratorMax), self.integratorMin)
        p = self.Kp * error
        d = 0.0
        if (dt>0):
            d = self.Kd*(error - self.previousError) / dt

        i = self.Ki*integral
        self.previousError = error
        self.previousTime = time
        self.integral = integral
        output = p+i+d
        output = max(min(output, self.maxOutput), self.minOutput)
        return output

    def reset(self):
        self.integral = 0.0
        self.previousError = 0.0
        self.previousTime = rospy.get_time()

"""
        error_vel = self.target.vel - self.current.vel
        requ_acc_phi = self.target.acc[0] + (KpPosphi*(error_pos[0]) + KvPosphi*(error_vel[0]))
        requ_acc_theta = self.target.acc[1] + (KpPostheta*(error_pos[1]) + KvPostheta*(error_vel[1]))

        des_angle_phi = (1.0/self.gravity)*(requ_acc_phi*np.sin(des_angle_yaw) - requ_acc_theta*np.cos(des_angle_yaw))
        des_angle_theta = (1.0/self.gravity)*(requ_acc_phi*np.cos(des_angle_yaw) + requ_acc_theta*np.sin(des_angle_yaw))

        roll = Kpphi*(des_angle_phi - C.A.phi) + Kdphi*(D.O.phi - C.O.phi)
        pitch = Kptheta * (des_angle_theta - C.A.theta) + Kdtheta*(D.O.theta - C.O.theta)
        yawrate = Kdpsi*(D.O.psi - C.O.psi) # Kppsi * (des_angle_yaw - C.A.psi) +

        thrust = self.mass*self.gravity - self.mass*(kd_Thrust*(error_vel[2]) + kp_Thrust*(error_pos[2]))

        linear = Vector3(pitch, roll, thrust)
        angular = Vector3(0.0, 0.0, yawrate)
        self.
        return twist_fly = Twist(linear, angular)

"""

if __name__ == '__main__':
    controller = Controller(sys.argv[1])
    controller.run()
