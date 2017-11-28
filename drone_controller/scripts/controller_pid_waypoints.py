#!/usr/bin/env python

import numpy as np
import rospy
import sys
from trajectory_generator.srv import *
from geometry_msgs.msg import Twist, Vector3
from mocap_node.srv import dronestaterequest
from drone_controller.srv import SetMode
from crazyflie_driver.srv import UpdateParams

class Controller:

    def __init__(self, tf_prefix):

        self.tf_prefix = tf_prefix

        rospy.init_node('DroneController' + tf_prefix)

        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pub_trajgen_states = rospy.Publisher('targetstates', Twist, queue_size=10)
        self.pub_mopcap_states = rospy.Publisher('mocapstates', Twist, queue_size=10)
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
        self.previous_mocap_state = {
            'X': 0.0,
            'Y': 0.0,
            'Z': 0.0,
            'Yaw': 0.0,
            'Pitch': 0.0,
            'Roll': 0.0
        }

        # Safety Variable for Mocap - counter for lost mocap frames
        self.safety_mocap = 0

        # If the controller enters Take Off Mode - It will go to these states x,y,z - Yaw by default set to zero.
        # TODO: clean up how the states are expressed in the parameters
        self.takeoff_states = np.array(rospy.get_param('takeoff_states'))
        self.takeoff_states = {
            'X': self.takeoff_states[0],
            'Y': self.takeoff_states[1],
            'Z': self.takeoff_states[2],
            'Yaw': 0.0
        }
        self.landing_states = np.array(rospy.get_param('landing_states'))
        self.landing_states = {
            'X': self.landing_states[0],
            'Y': self.landing_states[1],
            'Z': self.landing_states[2],
            'Yaw': 0.0
        }
        self.fixed_state = np.array(rospy.get_param('FixedWayPointYaw'))
        self.fixed_state = {
            'X': self.fixed_state[0],
            'Y': self.fixed_state[1],
            'Z': self.fixed_state[2],
            'Yaw': self.fixed_state[3]
        }

        # Initialize flightmode - start controller in 'Stop'-mode
        self.flightmode = 'Stop'
        # Possible flightmodes available
        self.modes = [
            'TakeOff',
            'FollowWayPoint',
            'FixedWayPoint',
            'Landing',
            'Stop'
        ]

        self._update_params = rospy.ServiceProxy('update_params', UpdateParams)
        #rospy.Service('trajgen_rest' + tf_prefix, trajgenrest, self.traj_gen)
        #rospy.spin()

    def get_drone_state(self):
        rospy.wait_for_service('mocap_state')
        try:
            state_srv = rospy.ServiceProxy('mocap_state', dronestaterequest)
            resp = state_srv()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
               
        if not resp.valid:
            self.safety_mocap += 1
            if self.safety_mocap >= 50:
                rospy.loginfo('Drone Not Found For a Long Time')
                rospy.signal_shutdown('Drone Not Found For a Long Time')
        else:
            self.safety_mocap = 0   # Rest - So the shutdown logic will be applied for 10 consecutive frames

        self.previous_mocap_state = resp.state
        
        degs = [a*np.pi/180 for a in resp.state.angular]
        angular_deg = Vector3(degs[0], degs[1], degs[2])
        #state.yaw = state.yaw*np.pi/180
        #state.pitch = state.pitch*np.pi/180
        #state.roll = state.roll*np.pi/180

        return Twist(resp.state.linear, angular_deg)

    def get_target_states(self,CurrentROSTime, PrevPosX, PrevPosY, PrevPosZ): # CurrentROSTime - Started from zero when the controller is started
        rospy.wait_for_service('generate_state_' + self.tf_prefix)
        try:
            generatestate = rospy.ServiceProxy('generate_state_' + self.tf_prefix, GetStates)
            states = generatestate(CurrentROSTime, PrevPosX, PrevPosY, PrevPosZ)
            return states
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_target_state(self):

        if self.flightmode == 'TakeOff':
            return self.takeoff_states
        elif self.flightmode == 'Landing':
            return self.landing_states
        elif self.flightmode == 'FixedWayPoint':
            return self.fixed_state
        elif self.flightmode == 'FollowWayPoint':
            return self.get_target_states(
                rospy.get_time() - self.trajectory_tstart,
                self.previous_trajgen_state['X'],
                self.previous_trajgen_state['Y'],
                self.previous_trajgen_state['Z']
            )
        return self.previous_mocap_state

    def set_flight_mode(self, args):
        if args.mode in self.modes:
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
            current_state = self.get_drone_state()
            target_state = self.get_target_state()

            if not self.flightmode == 'Stop':
                out_pitch = self.pitchcontrol.pid_calculate(target_state['X'], current_state.x)
                out_roll = self.rollcontrol.pid_calculate(target_state['Y'], current_state.y)
                out_yaw = self.yawcontrol.pid_calculate(target_state['Yaw'], current_state.yaw*180/np.pi)
                out_thrust = self.thrustcontrol.pid_calculate(target_state['Z'], current_state.z)

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
            # Publish MoCap States
            linear_mocap = Vector3(current_state.x, current_state.y, current_state.z)
            angular_mocap = Vector3(current_state.yaw, current_state.pitch, current_state.roll)
            twist_mocap = Twist(linear_mocap, angular_mocap)
            self.pub_mopcap_states.publish(twist_mocap)
            self.rate.sleep()
    
    def pid_rest_controller(self):
        self.pitchcontrol.reset()
        self.rollcontrol.reset()
        self.yawcontrol.reset()
        self.thrustcontrol.reset()     

    

class pid_controller:

    def __init__(self, tf_prefix, direction):

        # Load parameters from ROS
        param_dir = '/' + tf_prefix + '/PIDs/' + direction + '/'
        self.Kp = rospy.get_param(param_dir + 'kp')
        self.Kd = rospy.get_param(param_dir + 'kd')
        self.Ki = rospy.get_param(param_dir + 'ki')
        self.integratorMin = rospy.get_param(param_dir + 'integratorMin')
        self.integratorMax = rospy.get_param(param_dir + 'integratorMax')
        self.minOutput = rospy.get_param(param_dir + 'minOutput')
        self.maxOutput = rospy.get_param(param_dir + 'maxOutput')

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

