#!/usr/bin/env python

import numpy as np
import rospy
import sys
from trajectory_generator.srv import *
from geometry_msgs.msg import Twist, Vector3
from mocap_node.srv import dronestaterequest

class Controller:

    def __init__(self,tf_prefix):
# Intialize the node, Create a serviceclient to accept from trajectory generator, publish to cm_vel,
#  subscribe to MoCap node, set the PID gains in Self
        rospy.init_node('DroneController' + tf_prefix)
        self.pub_cmd_vel = rospy.Publisher(tf_prefix + '/cmd_vel', Twist, queue_size=10)
        self.state_srv = rospy.ServiceProxy('drone_states_' + tf_prefix,dronestaterequest)
        self.tf_prefix = tf_prefix
        self.rate = rospy.Rate(50)
        zerosstop = Vector3(0.0, 0.0, 0.0)
        self.twist_stop = Twist(zerosstop,zerosstop)
        self.pitchcontrol = pid_controller(rospy.get_param('/PIDs/X/kp'),rospy.get_param('/PIDs/X/kd'),rospy.get_param('/PIDs/X/ki'),rospy.get_param('/PIDs/X/integratorMin'),rospy.get_param('/PIDs/X/integratorMax'),rospy.get_param('/PIDs/X/minOutput'),rospy.get_param('/PIDs/X/maxOutput'))
        self.rollcontrol = pid_controller(rospy.get_param('/PIDs/Y/kp'),rospy.get_param('/PIDs/Y/kd'),rospy.get_param('/PIDs/Y/ki'),rospy.get_param('/PIDs/Y/integratorMin'),rospy.get_param('/PIDs/Y/integratorMax'),rospy.get_param('/PIDs/Y/minOutput'),rospy.get_param('/PIDs/Y/maxOutput'))
        self.yawcontrol = pid_controller(rospy.get_param('/PIDs/Yaw/kp'),rospy.get_param('/PIDs/Yaw/kd'),rospy.get_param('/PIDs/Yaw/ki'),rospy.get_param('/PIDs/Yaw/integratorMin'),rospy.get_param('/PIDs/Yaw/integratorMax'),rospy.get_param('/PIDs/Yaw/minOutput'),rospy.get_param('/PIDs/Yaw/maxOutput'))
        self.thrustcontrol = pid_controller(rospy.get_param('/PIDs/Z/kp'),rospy.get_param('/PIDs/Z/kd'),rospy.get_param('/PIDs/Z/ki'),rospy.get_param('/PIDs/Z/integratorMin'),rospy.get_param('/PIDs/Z/integratorMax'),rospy.get_param('/PIDs/Z/minOutput'),rospy.get_param('/PIDs/Z/maxOutput'))
        self.previousTime = rospy.get_time()
    def get_drone_state(self):
        state = self.state_srv(drone_name=self.tf_prefix)
        return state.x, state.y, state.z, (state.yaw*np.pi)/180, (state.pitch*np.pi)/180, (state.roll*np.pi)/180

    def get_target_states(self,CurrentROSTime, PrevPosX, PrevPosY, PrevPosZ): # CurrentROSTime - Started from zero when the controller is started
        rospy.wait_for_service('generate_state_' + self.tf_prefix)
        try:
            generatestate = rospy.ServiceProxy('generate_state_' + self.tf_prefix, GetStates)
            states = generatestate(CurrentROSTime, PrevPosX,PrevPosY,PrevPosZ)
            return states
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def controller_run(self):
        while not rospy.is_shutdown():
                x, y, z, yaw, pitch, roll = self.get_drone_state()
                targetstates = self.get_target_states(rospy.get_time() - self.previousTime,x,y,z)
                out_pitch = self.pitchcontrol.pid_calculate(rospy.get_time(),targetstates.PosX,x)
                out_roll = self.rollcontrol.pid_calculate(rospy.get_time(),targetstates.PosY,y)
                out_yaw = self.yawcontrol.pid_calculate(rospy.get_time(),0.0,(yaw*180)/np.pi)
                out_thrust = self.thrustcontrol.pid_calculate(rospy.get_time(),targetstates.PosZ,z)
                # Create rotation matrix 3-2-1 DCM - Euler Angles, Z-Y-X -> Yaw, Pitch, Roll -> Psi, theta, phi
                ctheta = np.cos(pitch)
                stheta = np.sin(pitch)
                cpsi = np.cos(yaw)
                spsi = np.sin(yaw)
                cphi = np.cos(roll)
                sphi = np.sin(roll)
                #rot_matrx = np.matrix([[ctheta*cpsi,ctheta*spsi,-stheta], [-cphi*spsi+sphi*stheta*cpsi, -cphi*cpsi+sphi*stheta*spsi, sphi*ctheta], [sphi*spsi+cphi*stheta*cpsi,-sphi*cpsi+cphi*stheta*spsi,cphi*ctheta]])
                rot_matrx = np.matrix([[cpsi,spsi,0.0],[-spsi,cpsi,0.0],[0.0,0.0,1.0]])
                # rot_matrx takes from original frame to transformed frame. 
                rotated_values = np.matmul(rot_matrx,np.matrix([[out_pitch],[out_roll],[out_yaw]]))
                rotated_values[2,0] = 0
                linear = Vector3(rotated_values[0,0], -rotated_values[1,0], out_thrust+28000.0)
                angular = Vector3(0.0, 0.0, rotated_values[2,0])
                twist_fly = Twist(linear, angular)
                self.pub_cmd_vel.publish(twist_fly)
                self.rate.sleep()

class pid_controller:
    def __init__(self,Kp,Kd,Ki,integratorMin,integratorMax,minOutput,maxOutput):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.integratorMin = integratorMin
        self.integratorMax = integratorMax
        self.minOutput = minOutput
        self.maxOutput = maxOutput
        self.integral = 0
        self.previousTime = rospy.get_time()
        self.previousError = 0

    def pid_calculate(self,time,target,current):

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
    control = Controller(sys.argv[1])
    control.controller_run()

