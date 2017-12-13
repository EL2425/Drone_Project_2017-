#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import numpy.matlib as npmat
from scipy.optimize import minimize
import time
import matplotlib.pyplot as plt
import os as os
from copy import copy
from itertools import combinations, cycle
from mpl_toolkits.mplot3d import Axes3D

from mocap_node.msg import State, MocapResp
from mocap_node.srv import *
from trajectory_generator.srv import *
from planner.msg import *
from std_msgs.msg import Bool

class TrajectoryGenerator(object):
    '''Calculates a trajectory based on a optimization. The objective function to minimize
    is a weigthed sum between the distance between the current state and the target and the
    distance between different drones'''
    def __init__(self,drones,time_steps):
        self.Q1 = 1             # Weight for the norm of distance between target and position
        self.Q2 = 15            # Weigth for the collision avoidance
        self.Q3 = [1,1,0.1]      # Weight factor each dimension of colllision avoidance
        self.d1 = [0.3, 0.3, 0.1]           # Distance a drone can move in each direction in a timestep [m]
        self.drones = drones
        self.N = len(drones)
        self.active_drones = []
        self.active_drones_safe = []
        self.T = time_steps
        self.dim = 3
        self.x_prev = np.array(self.get_target_states()*self.T)
        self.x_target = []

        rospy.init_node('trajectory_generator')
        self.target_server = rospy.Service('set_target', SetTarget, self.set_target)
        self.pub_exit_mode = rospy.Publisher('exit_mode', Bool, queue_size=10)
        self.sub_planner = rospy.Subscriber('action_dictionary', action_dict, self.set_plan)
        self.srv_add_traj = rospy.Service('add_drone_traj', AddDroneTraj, self.add_drone_traj)
        self.rate = rospy.Rate(5)

    def set_plan(self, data):
        for action in data.actionmessages:
            for d in self.drones:
                if d.tf_prefix == action.drone_name:
                    if action.drone_mode == 'move':
                        if d not in self.active_drones:
                            self.active_drones.append(d)
                    else:
                        if d in self.active_drones:
                            self.active_drones.remove(d)
                    d.set_action(action)
                    break

    def add_drone_traj(self, data):
        for d in self.drones:
            if data.drone == d.tf_prefix and d not in self.active_drones:
                self.active_drones.append(d)
        return True

    def obj_func(self,X):
        x=np.array(X)
        dist = np.linalg.norm(x - npmat.repmat(self.x_target,1,self.T))
        #pos_last = x[-self.N*self.dim:]
        #s = 0
        #for i in range(3):
        #    s += (pos_last[i] - self.x_target[i])**2
        #dist = np.sqrt(s)
        #dist = pos_last - self.x_target)

        #penalty for two drones being close to each other.
        collision_sum=0
        N = len(self.active_drones)
        D = self.dim
        inactive_drones = []
        for drone in self.drones:
            if drone not in self.active_drones_safe:
                inactive_drones.append(drone.get_state())
        for t in range(self.T):
            curr_t = X[t*D:(t+1)*N*D]
            d = [curr_t[n*D:(n+1)*D] for n in range(N)]
            d = d + inactive_drones
            for i,j in combinations(d,2):
                collision_sum += 1/(self.Q2*(self.norm(np.array(i)-np.array(j))**2)+1**-18)
        return self.Q1*dist + collision_sum

    def norm(self,vector):
        norm = 0
        for e,i in zip(vector,range(len(vector))):
            norm += self.Q3[i]*e**2
        return np.sqrt(norm)

    def distance_change_constraint(self,x):
        constraint=[]
        N = len(self.active_drones_safe)
        D = self.dim
        prev_t = self.x_prev[0:N*D]
        for t in range(self.T):
            curr_t = x[t*N*D:(t+1)*D*N]
            for n in range(N):
                curr_pos = curr_t[n*D:(n+1)*D]
                prev_pos = prev_t[n*D:(n+1)*D]
                con_norm = np.linalg.norm(prev_pos-curr_pos)

                constraint.append(self.d1-con_norm)
            prev_t = curr_t
        return np.array(constraint)

    def inequality_constraints(self,x):
        distance = self.distance_change_constraint(x)
        return distance

    def calculate_boundaries(self):
        boundaries = []
        for t in range(1,self.T+1):
            for d in self.active_drones_safe:
                state = d.get_state()
                boundaries.append(tuple([state[0]-t*self.d1[0],
                                            state[0]+t*self.d1[0]]))
                boundaries.append(tuple([state[1]-t*self.d1[1],
                                            state[1]+t*self.d1[1]]))
                boundaries.append(tuple([max(state[2]-t*self.d1[2],0),
                                            max(state[2]+t*self.d1[2],self.d1[2])]))
        return boundaries

    def get_target_states(self):
        states_lists = [d.get_target() for d in self.active_drones_safe]
        state_one_list = [y for x in states_lists for y in x]
        return state_one_list

    def get_current_states(self):
        states_lists = [d.get_state() for d in self.drones]
        state_one_list = [y for x in states_lists for y in x]
        return state_one_list

    def get_active_states(self):
        states_lists = [d.get_state() for d in self.active_drones_safe]
        state_one_list = [y for x in states_lists for y in x]
        return state_one_list

    def set_target(self, data):
        [d.set_target(data.x, data.y, data.z) for d in drones if d.tf_prefix == data.drone]
        return True

    def calc_trajectory(self):
        [d.update_state() for d in self.drones]
        self.active_drones_safe = copy(self.active_drones)
        self.x_target = self.get_target_states()
        self.x_prev = self.get_active_states()*self.T
        start = time.time()

        if len(self.x_prev) != 0:
            con = (
                {
                    'type': 'ineq',
                    'fun': self.inequality_constraints
                }
            )

            bnd = tuple(self.calculate_boundaries())

            res = minimize(
                self.obj_func,
                self.x_prev,
                bounds=bnd,
                #constraints=con,
                method='SLSQP',
                options={
                    'disp': False,
                    'maxiter': 1000,
                    #'ftol': 0.00001
                }
            )

            self.pub_exit_mode.publish(bool(res.success))
            #self.x_prev = res.x

            for d, n in zip(self.active_drones, range(len(self.active_drones))):
                x = res.x[n*self.dim]
                y = res.x[n*self.dim + 1]
                z = res.x[n*self.dim + 2]
                d.set_wp(x, y, z)

        [(d.publish_wp(), d.check_action_done()) for d in self.drones]
        end=time.time()

    def calculate_multiple(self,number_of_iter):
        start= time.time()
        #for i in range(number_of_iter):
        while not rospy.is_shutdown():
            self.calc_trajectory()
            self.rate.sleep()
            #self.plot_states() # Be careful, opens a lot of plots

    def plot_states(self):
        #self.print_states()
        fig = plt.figure()
        ax = fig.add_subplot(111,projection='3d')
        possible_colors=['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
        colors = cycle(possible_colors[0:self.N])
        for t in range(0,self.T):
            for n in range(self.N):
                ax.scatter(self.drones[n].x,self.drones[n].y,self.drones[n].z,color = next(colors))
                #ax.scatter(self.x_prev[t*(self.dim*self.N)+n*self.dim],self.x_prev[t*(self.dim*self.N)+n*self.dim+1],self.x_prev[t*(self.dim*self.N)+n*self.dim+2], color = next(colors))
        for n in range(self.N):
            ax.scatter(self.x_target[n*self.dim],self.x_target[n*self.dim+1],self.x_target[n*self.dim+2],marker="v",color=next(colors))
        ax.set_xlim3d([-2,2])
        ax.set_ylim3d([-2,2])
        ax.set_zlim3d([-2,2])
        plt.show()
        #plt.pause(0.0001)


    def print_states(self):
        for d in self.drones:
            print('Position of '+str(d.tf_prefix) + ' is \nx: ' + str(d.x) +'\ny: ' +str(d.y)+'\nz: '+str(d.z)+'\n')


class Drone(object):

    def __init__(self, tf_prefix):

        self.pub_controller = rospy.Publisher('/' + tf_prefix + '/waypoints', Twist, queue_size=10)
        self.pub_target = rospy.Publisher('/' + tf_prefix + '/target', Twist, queue_size=10)
        self.sub_mocap = rospy.ServiceProxy('/' + tf_prefix + '/mocap_srv', dronestaterequest)
        self.srv_target = rospy.Service('/' + tf_prefix + '/set_traj_target', SetTrajTarget, self.set_target_srv)

        self.tf_prefix = tf_prefix
        self.action = 'idle'
        self.action_id = -1
        self.timer = 0

        mocap_state = self.sub_mocap()

        self.x = mocap_state.state.x
        self.y = mocap_state.state.y
        self.z = mocap_state.state.z
        self.target_x = mocap_state.state.x
        self.target_y = mocap_state.state.y
        self.target_z = mocap_state.state.z
        self.wp_x = mocap_state.state.x
        self.wp_y = mocap_state.state.y
        self.wp_z = mocap_state.state.z


    def set_action(self, action):
        if action.action_id != self.action_id:
            self.action = action.drone_mode
            self.action_id = action.action_id
            if action.drone_mode == 'move':
                self.set_target(action.x, action.y, action.z)
            if action.drone_mode in ['load', 'unload']:
                self.timer = rospy.get_time()
            if action.drone_mode == 'takeoff':
                self.set_wp(self.x, self.y, 0.7)
                self.set_target(self.x, self.y, 0.7)
            if action.drone_mode == 'land':
                self.set_wp(self.x, self.y, 0.25)
                self.set_target(self.x, self.y, 0.25)

    def update_state(self):
        resp = self.sub_mocap()
        if resp.valid:
            self.x = resp.state.x
            self.y = resp.state.y
            self.z = resp.state.z

    def set_wp(self,x,y,z):
        self.wp_x = x
        self.wp_y = y
        self.wp_z = z

    def set_target(self,x,y,z):
        self.target_x = x
        self.target_y = y
        self.target_z = z

    def set_target_srv(self, data):
        self.target_x = data.x
        self.target_y = data.y
        self.target_z = data.z
        return True

    def check_action_done(self):
        if self.action == 'move':
            dist = np.linalg.norm(np.array(self.get_state()) - np.array(self.get_target()))
            if dist < 0.1:
                rospy.set_param('/completed_actions/action_id_' + str(self.action_id), 1)
        elif self.action in ['load', 'unload']:
            curr_time = rospy.get_time()
            print(curr_time - self.timer)
            if curr_time - self.timer > 3.0:
                rospy.set_param('/completed_actions/action_id_' + str(self.action_id), 1)
        elif self.action in ['takeoff', 'land']:
            if np.sqrt((self.target_z - self.z)**2) < 0.05:
                rospy.set_param('/completed_actions/action_id_' + str(self.action_id), 1)

    def publish_wp(self):
        linear = Vector3(self.wp_x, self.wp_y, self.wp_z)
        target = Vector3(self.target_x, self.target_y, self.target_z)
        angular = Vector3(0.0, 0.0, 0.0)
        next_state = Twist(linear, angular)
        target_state = Twist(target, angular)
        self.pub_controller.publish(next_state)
        self.pub_target.publish(target_state)

    def get_state(self):
        return self.x, self.y, self.z

    def get_target(self):
        return self.target_x, self.target_y, self.target_z


if __name__ == '__main__':
    drones = [
        # Drone("crazyflie1", -2, -2, 0, 2, 2, 1),
        # Drone("crazyflie2", 2, -2, 0, -2, 2, 1),
        # Drone('crazyflie3', 1.1, -2.5, 1, -0.5, 1.5, 1.2),
        # Drone('crazyflie1'),
        Drone('crazyflie4'),
        Drone('crazyflie5')
    ]
    trajgen = TrajectoryGenerator(drones, 4)
    trajgen.calculate_multiple(100)
