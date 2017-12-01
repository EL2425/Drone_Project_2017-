import numpy as np
import numpy.matlib as npmat
from scipy.optimize import minimize
import time
import matplotlib.pyplot as plt
import os as os
from itertools import combinations, cycle
from mpl_toolkits.mplot3d import Axes3D

class TrajectoryGenerator(object):
    '''Calculates a trajectory based on a optimization. The objective function to minimize
    is a weigthed sum between the distance between the current state and the target and the 
    distance between different drones'''
    def __init__(self,drones,time_steps):
        self.Q1 = 1             # Weight for the norm of distance between target and position
        self.Q2 = 20            # Weigth for the collision avoidance
        self.Q3 = [0.1,0.1,10]      # Weight factor each dimension of colllision avoidance 
        self.d1 = 0.1           # Distance a drone can move in each direction in a timestep [m]
        self.drones = drones
        self.N = len(drones)
        self.T = time_steps
        self.dim = 3
        self.x_prev = np.array(self.get_current_states()*self.T)
        self.x_target = []
    
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
        self.x_target = np.array(self.get_target_states())
        self.x_prev = self.get_current_states()*self.T
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
        self.x_prev = res.x
        
        #set state in each drone
        for d,n in zip(self.drones,range(self.N)):
            d.set_state(res.x[n*self.dim],res.x[n*self.dim+1],res.x[n*self.dim+2])

        end=time.time()
        print('Time Elapsed: '+ str((end-start)))

    def calculate_multiple(self,number_of_iter):
        plt.ion()
        start= time.time()
        for i in range(number_of_iter):
            self.calc_trajectory()
            self.plot_states()
        print('Time Elapsed: '+str(time.time()-start))

    def plot_states(self):
        self.print_states()
        fig = plt.figure()
        ax = fig.add_subplot(111,projection='3d')
        possible_colors=['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
        colors = cycle(possible_colors[0:self.N])
        for t in range(0,self.T):
            for n in range(self.N):
                #ax.scatter(self.drones[n].x,self.drones[n].y,self.drones[n].z,color = next(colors))
                ax.scatter(self.x_prev[t*(self.dim*self.N)+n*self.dim],self.x_prev[t*(self.dim*self.N)+n*self.dim+1],self.x_prev[t*(self.dim*self.N)+n*self.dim+2], color = next(colors))
        for n in range(self.N):
            ax.scatter(self.x_target[n*self.dim],self.x_target[n*self.dim+1],self.x_target[n*self.dim+2],marker="v",color=next(colors))
        ax.set_xlim3d([-2,2])
        ax.set_ylim3d([-2,2])
        ax.set_zlim3d([-2,2])    
        plt.show()
        plt.pause(0.0001)


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

    def set_state(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z
    
    def set_target(self,x,y,z):
        self.target_x = x
        self.target_y = y
        self.target_z = z

    def get_state(self):
        return self.x, self.y, self.z

    def get_target(self):
        return self.target_x, self.target_y, self.target_z


if __name__ == '__main__':
    drone1 = Drone("crazyflie1", -2, -2, 0, 2, 2, 1)
    drone2 = Drone("crazyflie2", 2, -2, 0, -2, 2, 1)
    drone3 = Drone("crazyflie3", -2, 2, 0, 2, -2, 1)
    drone4 = Drone("crazyflie4", 2, 2, -2, -2, -2, 1)
    drone5 = Drone("crazyflie5", 0, 0, 1, 0, 0, 1)


    trajgen = TrajectoryGenerator([drone1,drone2,drone3,drone4,drone5],3)
    trajgen.calculate_multiple(50)









    
