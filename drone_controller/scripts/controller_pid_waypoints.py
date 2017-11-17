#!/usr/bin/env python

import numpy as np
import rospy
import sys
from trajectory_generator.srv import *
from geometry_msgs.msg import Twist, Vector3
from mocap_node.srv import dronestaterequest
from drone_controller.srv import trajgenrest

class Controller:

    def __init__(self,tf_prefix):
        self.tf_prefix = tf_prefix
        rospy.init_node('DroneController' + tf_prefix)                                                  # Set the Controller Node.
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)                # Publisher node for crazyflie - Also used for recording in Rosbag
        self.pub_trajgen_states = rospy.Publisher('targetstates', Twist, queue_size=10)    # Publishing Trajgen States received by the controller - Used for recording in Rosbag
        self.pub_mopcap_states = rospy.Publisher('mocapstates', Twist, queue_size=10)      # Publishing MoCap States received by the controller - Used for recording in Rosbag
        self.rate = rospy.Rate(50)                                                                      # Set Controller frequency
        zerosstop = Vector3(0.0, 0.0, 0.0)
        self.twist_stop = Twist(zerosstop,zerosstop)                                                    # Intialize with Zeros - Can be used to send zeros signals to Crazyflie if required.
        # Intialize Yaw,Pitch, Roll, Thrust Controllers. 
        # !!! Change the PID's to /tf_prefix/PIDs - Or find a way to call rosparam such that this is taken into account.
        self.pitchcontrol = pid_controller(rospy.get_param('/' + tf_prefix + '/PIDs/X/kp'),rospy.get_param('/' + tf_prefix + '/PIDs/X/kd'),rospy.get_param('/' + tf_prefix + '/PIDs/X/ki'),rospy.get_param('/' + tf_prefix + '/PIDs/X/integratorMin'),rospy.get_param('/' + tf_prefix + '/PIDs/X/integratorMax'),rospy.get_param('/' + tf_prefix + '/PIDs/X/minOutput'),rospy.get_param('/' + tf_prefix + '/PIDs/X/maxOutput'))
        self.rollcontrol = pid_controller(rospy.get_param('/' + tf_prefix + '/PIDs/Y/kp'),rospy.get_param('/' + tf_prefix + '/PIDs/Y/kd'),rospy.get_param('/' + tf_prefix + '/PIDs/Y/ki'),rospy.get_param('/' + tf_prefix + '/PIDs/Y/integratorMin'),rospy.get_param('/' + tf_prefix + '/PIDs/Y/integratorMax'),rospy.get_param('/' + tf_prefix + '/PIDs/Y/minOutput'),rospy.get_param('/' + tf_prefix + '/PIDs/Y/maxOutput'))
        self.yawcontrol = pid_controller(rospy.get_param('/' + tf_prefix + '/PIDs/Yaw/kp'),rospy.get_param('/' + tf_prefix + '/PIDs/Yaw/kd'),rospy.get_param('/' + tf_prefix + '/PIDs/Yaw/ki'),rospy.get_param('/' + tf_prefix + '/PIDs/Yaw/integratorMin'),rospy.get_param('/' + tf_prefix + '/PIDs/Yaw/integratorMax'),rospy.get_param('/' + tf_prefix + '/PIDs/Yaw/minOutput'),rospy.get_param('/' + tf_prefix + '/PIDs/Yaw/maxOutput'))
        self.thrustcontrol = pid_controller(rospy.get_param('/' + tf_prefix +'/PIDs/Z/kp'),rospy.get_param('/' + tf_prefix +'/PIDs/Z/kd'),rospy.get_param('/' + tf_prefix +'/PIDs/Z/ki'),rospy.get_param('/' + tf_prefix +'/PIDs/Z/integratorMin'),rospy.get_param('/' + tf_prefix +'/PIDs/Z/integratorMax'),rospy.get_param('/' + tf_prefix +'/PIDs/Z/minOutput'),rospy.get_param('/' + tf_prefix +'/PIDs/Z/maxOutput'))
        # Intialize Trajectory Generator States - Will only be used if the trajectory gen. function is invoked.
        self.previous_trajgen_state = {'X':0.0,'Y':0.0,'Z':0.0}
        # Intialize previous mocap states - Incase if the drone is not found the previous state are returned
        self.previous_mocap_state = {'X':0.0,'Y':0.0,'Z':0.0,'Yaw':0.0,'Pitch':0.0,'Roll':0.0}
        # Safety Variable for Mocap - If the mocap returns no drone found for more than certain number of times - The entire controller is shut down
        self.safety_mocap = 0
        # If the controller enters Take Off Mode - It will go to these states x,y,z - Yaw by default set to zero.
        self.takeoff_states = np.array(rospy.get_param('/' + tf_prefix +'/takeoff_states'))
        self.landing_states = np.array(rospy.get_param('/' + tf_prefix +'/landing_states')) 
        # Keeping track of which mode is active
        self.Modes = {'TakeOff':False,'FollowWayPoint':False,'FixedWayPoint':False,'Landing':False,'Unknown':False}
        #rospy.Service('trajgen_rest' + tf_prefix, trajgenrest, self.traj_gen)
        #rospy.spin()
 	
 	def traj_gen(self):
 		self.previousTime_trajegen = rospy.get_time()
 		output = 1.0
 		return trajgenrestResponse(output)


    def get_drone_state(self):
        rospy.wait_for_service('/' + self.tf_prefix + '/drone_states_' + self.tf_prefix)
        try:
        	state_srv = rospy.ServiceProxy('drone_states_' + self.tf_prefix,dronestaterequest)                                                                      # Used to create Node name and to speak with some other services
        	state = state_srv(drone_name=self.tf_prefix,prev_x=self.previous_mocap_state['X'],prev_y=self.previous_mocap_state['Y'],prev_z=self.previous_mocap_state['Z'],prev_yaw=self.previous_mocap_state['Yaw'],prev_pitch=self.previous_mocap_state['Pitch'],prev_roll=self.previous_mocap_state['Roll'])
       	except rospy.ServiceException, e:
           	print "Service call failed: %s" % e
           	
        if(state.notvalidflag):
            self.safety_mocap += 1
            if(self.safety_mocap >= 50):
                rospy.loginfo('Drone Not Found For a Long Time')
                rospy.signal_shutdown('Drone Not Found For a Long Time')
        else:
            self.safety_mocap = 0   # Rest - So the shutdown logic will be applied for 10 consecutive frames

        self.previous_mocap_state = {'X':state.x,'Y':state.y,'Z':state.z,'Yaw':state.yaw,'Pitch':state.pitch,'Roll':state.roll}          
        return state.x, state.y, state.z, (state.yaw*np.pi)/180, (state.pitch*np.pi)/180, (state.roll*np.pi)/180

    def get_target_states(self,CurrentROSTime, PrevPosX, PrevPosY, PrevPosZ): # CurrentROSTime - Started from zero when the controller is started
        rospy.wait_for_service('/' + self.tf_prefix +'/generate_state_' + self.tf_prefix)
        try:
            generatestate = rospy.ServiceProxy('generate_state_' + self.tf_prefix, GetStates)
            states = generatestate(CurrentROSTime, PrevPosX,PrevPosY,PrevPosZ)
            return states
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def controller_run(self):
        while not rospy.is_shutdown():
            self.flightmode = rospy.get_param('/' + self.tf_prefix +'/FlightMode')    # Check what is current Flight mode from Param Workspace
            x, y, z, yaw, pitch, roll = self.get_drone_state()


            compute_control = True
            if(self.flightmode == 'TakeOff'):             # Go to Fixed WayPoint - Defined in takeoff_states - Hard Coded in yaml file
                if not self.Modes['TakeOff']:
                    self.Modes = {'TakeOff':True,'FollowWayPoint':False,'FixedWayPoint':False,'Landing':False,'Unknown':False} 
                    inputcontroller_x = float(self.takeoff_states[0])
                    inputcontroller_y = float(self.takeoff_states[1])
                    inputcontroller_z = float(self.takeoff_states[2])
                    inputcontroller_yaw = 0.0
                    rospy.loginfo("Take Off Mode Entered")
                

            elif(self.flightmode == 'FollowWayPoint'):   # Follow way points given by trajectory generator
                if not self.Modes['FollowWayPoint']:
                    self.previousTime_trajegen = rospy.get_time() # To Store the time when the traj. gen. started for the first time. As the input time starts from t = 0 seconds.
                    self.Modes = {'TakeOff':False,'FollowWayPoint':True,'FixedWayPoint':False,'Landing':False,'Unknown':False}
                    rospy.loginfo("Follow WayPoint Mode Entered")
                targetstates = self.get_target_states(rospy.get_time() - self.previousTime_trajegen,self.previous_trajgen_state['X'],self.previous_trajgen_state['Y'],self.previous_trajgen_state['Z'])
                inputcontroller_x = targetstates.PosX
                inputcontroller_y = targetstates.PosY
                inputcontroller_z = targetstates.PosZ
                inputcontroller_yaw = 0.0
                # Publish Trajectory Generator States
                linear_trajgen = Vector3(targetstates.PosX,targetstates.PosY,targetstates.PosZ)
                angular_trajgen = Vector3(0.0,0.0,0.0)
                twist_trajgen = Twist(linear_trajgen, angular_trajgen)
                self.pub_trajgen_states.publish(twist_trajgen)
                self.previous_trajgen_state = {'X':targetstates.PosX,'Y':targetstates.PosY,'Z':targetstates.PosZ}
                

            elif(self.flightmode == 'FixedWayPoint'):     # Keep Reading the rosparam workspace to see if the fixedwaypoint has changed
                if not self.Modes['FixedWayPoint']:
                    self.Modes = {'TakeOff':False,'FollowWayPoint':False,'FixedWayPoint':True,'Landing':False,'Unknown':False} 
                    rospy.loginfo("Fixed WayPoint Mode Entered")
                fixedwaypoint = np.array(rospy.get_param( '/' + self.tf_prefix + '/FixedWayPointYaw'))
                inputcontroller_x = float(fixedwaypoint[0])
                inputcontroller_y = float(fixedwaypoint[1])
                inputcontroller_z = float(fixedwaypoint[2])
                inputcontroller_yaw = float(fixedwaypoint[3])
                linear_fixedwaypoint = Vector3(inputcontroller_x,inputcontroller_y,inputcontroller_z)
                angular_fixedwaypoint = Vector3(0.0,0.0,0.0)
                twist_fixedwaypoint = Twist(linear_fixedwaypoint, angular_fixedwaypoint)
                self.pub_trajgen_states.publish(twist_fixedwaypoint)
                

            elif(self.flightmode == 'Landing'):           # Hover at the landing point
                if not self.Modes['Landing']:
                    self.Modes = {'TakeOff':False,'FollowWayPoint':False,'FixedWayPoint':False,'Landing':True,'Unknown':False}
                    rospy.loginfo("Landing Mode Entered")
                inputcontroller_x = float(self.landing_states[0])
                inputcontroller_y = float(self.landing_states[1])
                inputcontroller_z = float(self.landing_states[2])
                inputcontroller_yaw = 0.0
                distance_to_landingpoint = np.linalg.norm([(inputcontroller_x-x),(inputcontroller_y - y),(inputcontroller_z - z)]) # Check distance to Landing Point
                if(distance_to_landingpoint < 0.1):
                    compute_control = False
                    twist_fly = self.twist_stop
                    self.pid_rest_controller()     # Rest all values in PID.  
                 
            else:
                compute_control = False
                twist_fly = self.twist_stop
                if not self.Modes['Unknown']:
                    self.Modes = {'TakeOff':False,'FollowWayPoint':False,'FixedWayPoint':False,'Landing':False,'Unknown':True}
                    rospy.loginfo("Unknown Mode Entered")
                self.pid_rest_controller()     # Rest all values in PID.  
                     
                

            if(compute_control):
                out_pitch = self.pitchcontrol.pid_calculate(rospy.get_time(),inputcontroller_x,x)
                out_roll = self.rollcontrol.pid_calculate(rospy.get_time(),inputcontroller_y,y)
                out_yaw = self.yawcontrol.pid_calculate(rospy.get_time(),inputcontroller_yaw,(yaw*180)/np.pi)
                out_thrust = self.thrustcontrol.pid_calculate(rospy.get_time(),inputcontroller_z,z)
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
                #rotated_values[2,0] = 0
                linear = Vector3(rotated_values[0,0], -rotated_values[1,0], out_thrust+43000.0)
                angular = Vector3(0.0, 0.0, -rotated_values[2,0])
                twist_fly = Twist(linear, angular)
            
            self.pub_cmd_vel.publish(twist_fly) # Publisher For CrazyFlie
            # Publish MoCap States
            linear_mocap = Vector3(x,y,z)
            angular_mocap = Vector3(yaw,pitch,roll)
            twist_mocap = Twist(linear_mocap, angular_mocap)
            self.pub_mopcap_states.publish(twist_mocap)
            self.rate.sleep()
    
    def pid_rest_controller(self):
        self.pitchcontrol.rest_controller()
        self.rollcontrol.rest_controller()
        self.yawcontrol.rest_controller()
        self.thrustcontrol.rest_controller()     

	

class pid_controller:
    def __init__(self,Kp,Kd,Ki,integratorMin,integratorMax,minOutput,maxOutput):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.integratorMin = integratorMin
        self.integratorMax = integratorMax
        self.minOutput = minOutput
        self.maxOutput = maxOutput
        self.integral = 0.0
        self.previousTime = rospy.get_time()
        self.previousError = 0.0

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

    def rest_controller(self):
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
    control = Controller(sys.argv[1])
    control.controller_run()

