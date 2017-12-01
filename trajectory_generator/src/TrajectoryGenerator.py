#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import numpy.matlib as npmat
from scipy.optimize import minimize
import time
import matplotlib.pyplot as plt
import os as os
from itertools import combinations, cycle
from mpl_toolkits.mplot3d import Axes3D

from mocap_node.msg import State, MocapResp
from mocap_node.srv import *
from std_msgs.msg import String

class TrajectoryGenerator(object):
    '''Calculates a trajectory based on a optimization. The objective function to minimize
    is a weigthed sum between the distance between the current state and the target and the
    distance between different drones'''
    def __init__(self,drones,time_steps):
        self.Q1 = 1             # Weight for the norm of distance between target and position
        self.Q2 = 10            # Weigth for the collision avoidance
        self.Q3 = [1,1,1]      # Weight factor each dimension of colllision avoidance
        self.d1 = 0.05           # Distance a drone can move in each direction in a timestep [m]
        self.drones = drones
        self.N = len(drones)
        self.T = time_steps
        self.dim = 3
        self.x_prev = np.array(self.get_current_states()*self.T)
        self.x_target = []

        rospy.init_node('trajectory_generator')

        self.pub_exit_mode = rospy.Publisher('exit_mode', String, queue_size=10)
        self.rate = rospy.Rate(5)


    def obj_func(self,X):
        x=np.array(X)
        dist = np.linalg.norm(x - npmat.repmat(self.x_target,1,self.T))

        #penalty for two drones to be close to each other.
        collision_sum=0
        N = self.N
        D = self.dim
        for t in range(self.T):
            curr_t = X[t*D:(t+1)*N*D]
            d = [curr_t[n*D:(n+1)*D] for n in range(N)]
            for i,j in combinations(d,2):
                collision_sum += 1/(self.Q2*(self.norm(i-j)**2)+1**-18)
        return self.Q1*dist + collision_sum

    def norm(self,vector):
        norm = 0
        for e,i in zip(vector,range(len(vector))):
            norm += self.Q3[i]*e**2
        return np.sqrt(norm)

    def distance_change_constraint(self,x):
        constraint=[]
        N = self.N
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
            for n in range(self.N):
                boundaries.append(tuple([self.x_prev[n*self.dim]-t*self.d1,
                                            self.x_prev[n*self.dim]+t*self.d1]))
                boundaries.append(tuple([self.x_prev[n*self.dim+1]-t*self.d1,
                                            self.x_prev[n*self.dim+1]+t*self.d1]))
                boundaries.append(tuple([max(self.x_prev[n*self.dim+2]-t*self.d1,0),
                                            max(self.x_prev[n*self.dim+2]+t*self.d1,self.d1)]))
        return boundaries

    def get_target_states(self):
        states_lists = [d.get_target() for d in self.drones]
        state_one_list = [y for x in states_lists for y in x]
        return state_one_list

    def get_current_states(self):
        states_lists = [d.get_state() for d in self.drones]
        state_one_list = [y for x in states_lists for y in x]
        return state_one_list

    def calc_trajectory(self):

        [d.update_state() for d in self.drones]
        self.x_target = np.array(self.get_target_states())
        #self.x_prev = self.get_current_states()*self.T
        start = time.time()

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
                'disp': True,
                'maxiter': 1000,
                'ftol': 0.00001
            }
        )

        self.pub_exit_mode.publish(res.message)
        self.x_prev = res.x

        for d, n in zip(self.drones, range(self.N)):
            x = res.x[n*self.dim]
            y = res.x[n*self.dim + 1]
            z = res.x[n*self.dim + 2]
            d.set_wp(x, y, z)

        [d.publish_wp() for d in self.drones]
        end=time.time()
        print('Time Elapsed: '+ str((end-start)))

    def calculate_multiple(self,number_of_iter):
        start= time.time()
        #for i in range(number_of_iter):
        while not rospy.is_shutdown():
            self.calc_trajectory()
            self.rate.sleep()
            #self.plot_states() # Be careful, opens a lot of plots
        print('Time Elapsed: '+str(time.time()-start))

    def plot_states(self):
        self.print_states()
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

    def __init__(self,tf_prefix,x,y,z,target_x,target_y,target_z):
        self.tf_prefix = tf_prefix
        self.x = x
        self.y = y
        self.z = z
        self.target_x = target_x
        self.target_y = target_y
        self.target_z = target_z
        self.wp_x = 0
        self.wp_y = 0
        self.wp_z = 0

        self.pub_controller = rospy.Publisher('/' + self.tf_prefix + '/waypoints', Twist, queue_size=10)
        self.sub_mocap = rospy.ServiceProxy('/' + self.tf_prefix + '/mocap_srv', dronestaterequest)

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

    def publish_wp(self):
        linear = Vector3(self.wp_x, self.wp_y, self.wp_z)
        angular = Vector3(0.0, 0.0, 0)
        next_state = Twist(linear, angular)
        self.pub_controller.publish(next_state)

    def get_state(self):
        return self.x, self.y, self.z

    def get_target(self):
        return self.target_x, self.target_y, self.target_z


if __name__ == '__main__':
    # drone1 = Drone("crazyflie1", -2, -2, 0, 2, 2, 1)
    # drone2 = Drone("crazyflie2", 2, -2, 0, -2, 2, 1)
    #drone3 = Drone("crazyflie3", -2, -2, 1, -1, -2, 1.2)
    # drone4 = Drone("crazyflie4", 2, 2, -2, -2, -2, 1)
    drone5 = Drone("crazyflie5", 0, 0, 1, 2, 1, 1.2)


    trajgen = TrajectoryGenerator([drone5], 2 )
    trajgen.calculate_multiple(100)
