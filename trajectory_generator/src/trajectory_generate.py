#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from sympy import *
from std_msgs.msg import String
from geometry_msgs.msg import Point
from trajectory_generator.srv import *
import sys


class GenerateTrajectory:
    def __init__(self,waypoints,timeconstant,tf_prefix):
        self.waypoints = waypoints
        self.TimeConstant = timeconstant
        self.generate_coeff()
        rospy.init_node('trajectory_generate_' + tf_prefix)
        s = rospy.Service('generate_state_' + tf_prefix, GetStates, self.generate_state)
        print "Ready to Generate States."
        rospy.spin()

    def generate_coeff(self):
        # Assuming I have WayPoints - Generate Coeffients for a particular drone
        # !! Create a Safety Logic to check the shape of the waypoints.
        waypoints = self.waypoints
        TimeConstant = self.TimeConstant
        NoOfTrajectors = waypoints.shape[0] - 1  # Get number of waypoints, As number of rows define number of waypoints.
        S = np.zeros((NoOfTrajectors + 1, 1))
        Time = np.zeros((NoOfTrajectors, 1))
        PrevWayPoint = waypoints[0,:]  # InitialStartPoint, By Default we assume everything starts from Zero Point
        Coeffients = np.zeros((8*NoOfTrajectors,3))
        for i in range(0, NoOfTrajectors):
            WayPoint = waypoints[i + 1, :]
            S[i] = np.sum(Time)
            Time[i] = TimeConstant * (np.linalg.norm(WayPoint - PrevWayPoint))
            PrevWayPoint = WayPoint

        S[NoOfTrajectors] = np.sum(Time)

        for j in range(0, 3):  # X,Y,Z
            A = np.zeros((8 * NoOfTrajectors, 8 * NoOfTrajectors))
            B = np.zeros((8 * NoOfTrajectors, 1))
            Counter = 0
            for k in range(0, NoOfTrajectors):
                A[Counter, (8*(k+1) - 7)-1:8*(k+1)] = self.poly_try(k, S, Time, S[k], 0)
                B[k + NoOfTrajectors * 0] = waypoints[k, j]
                Counter = Counter + 1

            for k in range(0, NoOfTrajectors):
                A[Counter,(8*(k+1) - 7)-1:8*(k+1)] = self.poly_try(k, S, Time, S[k + 1], 0)
                B[k + NoOfTrajectors * 1] = waypoints[k + 1, j]
                Counter = Counter + 1

            for count in range(1,4): # Three Derivatives
                A[Counter, (8*1 - 7)-1:8*1] = self.poly_try(0, S, Time, S[0], count)
                Counter = Counter + 1

            for count in range(1,4): # Three Derivatives
                A[Counter, (8*NoOfTrajectors - 7)-1:8*NoOfTrajectors] = self.poly_try(NoOfTrajectors-1, S, Time, S[NoOfTrajectors], count)
                Counter = Counter + 1

            for count in range(1,7): # Six Derivatives
                for Inner in range(0,NoOfTrajectors - 1):
                    A[Counter, (8*(Inner+1) - 7)-1:8 *(Inner+1)] = self.poly_try(Inner, S, Time, S[Inner + 1], count)
                    A[Counter, (8*(Inner + 2) - 7)-1:8 * (Inner + 2)] = -1 * self.poly_try(Inner + 1, S, Time, S[Inner + 1],count)
                    Counter = Counter + 1

            Coeffients[0:8*NoOfTrajectors,j] = np.asarray(np.matmul(np.linalg.inv(A),B)).reshape(-1)
        self.Coeffients = Coeffients
        self.S = S
        self.Time = Time

    @staticmethod
    def poly_try(tnumber, s, time, t_value, derivative):
        a, b, t = symbols("a b t")
        a_value = s[tnumber]
        b_value = time[tnumber]

        dcoeff = [1, ((t - a) / b), ((t - a) / b)**2, ((t - a) / b)**3, ((t - a) / b)**4, ((t - a) / b)**5, ((t - a) / b)**6, ((t - a) / b)**7]
        for i in range(0,derivative):
            for j in range(0,8):
                dcoeff[j] = diff(dcoeff[j], t)

        tsubs = np.zeros((1, 8))
        tsubs[0,0]= float(dcoeff[0])
        for i in range(0,7):
            tsubs[0, i + 1] = float(((dcoeff[i+1].subs(a, a_value)).subs(b,b_value)).subs(t,t_value))

        return tsubs

    def generate_state(self, data):
        S = self.S
        Time = self.Time
        Coeffients = self.Coeffients
        t = data.time
        PosX_Prev = data.PrevPosX
        PosY_Prev = data.PrevPosY
        PosZ_Prev = data.PrevPosZ
        CoeffToUse = np.zeros((8, 3))
        i = 1
        StopComputing = 0
        if (t < S[1]):
            i = 1
            CoeffToUse = Coeffients[(8 * i - 7) - 1:8 * i, 0:3]
        elif (t < S[2]):
            i = 2
            CoeffToUse = Coeffients[(8 * i - 7) - 1:8 * i, 0:3]
        elif (t < S[3]):
            i = 3
            CoeffToUse = Coeffients[(8 * i - 7) - 1:8 * i, 0:3]
        elif (t < S[4]):
            i = 4
            CoeffToUse = Coeffients[(8 * i - 7) - 1:8 * i, 0:3]
        elif (t > S[4]):
            StopComputing = 1

        if (StopComputing == 0):
            A = S[i - 1]
            B = Time[i - 1]

            Axes = 0
            PosX = CoeffToUse[0, Axes] + CoeffToUse[1, Axes] * ((t - A) / B) + CoeffToUse[2, Axes] * ((t - A) / B) ** 2 + \
                   CoeffToUse[3, Axes] * ((t - A) / B) ** 3 + CoeffToUse[4, Axes] * ((t - A) / B) ** 4 + CoeffToUse[
                                                                                                             5, Axes] * ((
                                                                                                                         t - A) / B) ** 5 + \
                   CoeffToUse[6, Axes] * ((t - A) / B) ** 6 + CoeffToUse[7, Axes] * ((t - A) / B) ** 7
            VelX = CoeffToUse[0, Axes] * 0 + CoeffToUse[1, Axes] * (1 / B) - CoeffToUse[2, Axes] * (
            (2 * A - 2 * t) / B ** 2) + 3 * CoeffToUse[3, Axes] * ((A - t) ** 2 / B ** 3) - 4 * CoeffToUse[4, Axes] * (
            (A - t) ** 3 / B ** 4) + 5 * CoeffToUse[5, Axes] * ((A - t) ** 4 / B ** 5) - 6 * CoeffToUse[6, Axes] * (
            (A - t) ** 5 / B ** 6) + 7 * CoeffToUse[7, Axes] * ((A - t) ** 6 / B ** 7)
            AccX = CoeffToUse[0, Axes] * 0 + CoeffToUse[2, Axes] * (2 / (B ** 2)) - 3 * CoeffToUse[3, Axes] * (
            (2 * A - 2 * t) / B ** 3) + 12 * CoeffToUse[4, Axes] * ((A - t) ** 2 / B ** 4) - 20 * CoeffToUse[5, Axes] * (
            (A - t) ** 3 / B ** 5) + 30 * CoeffToUse[6, Axes] * ((A - t) ** 4 / B ** 6) - 42 * CoeffToUse[7, Axes] * (
            (A - t) ** 5 / B ** 7)

            Axes = 1
            PosY = CoeffToUse[0, Axes] + CoeffToUse[1, Axes] * ((t - A) / B) + CoeffToUse[2, Axes] * ((t - A) / B) ** 2 + \
                   CoeffToUse[3, Axes] * ((t - A) / B) ** 3 + CoeffToUse[4, Axes] * ((t - A) / B) ** 4 + CoeffToUse[
                                                                                                             5, Axes] * ((
                                                                                                                         t - A) / B) ** 5 + \
                   CoeffToUse[6, Axes] * ((t - A) / B) ** 6 + CoeffToUse[7, Axes] * ((t - A) / B) ** 7
            VelY = CoeffToUse[0, Axes] * 0 + CoeffToUse[1, Axes] * (1 / B) - CoeffToUse[2, Axes] * (
            (2 * A - 2 * t) / B ** 2) + 3 * CoeffToUse[3, Axes] * ((A - t) ** 2 / B ** 3) - 4 * CoeffToUse[4, Axes] * (
            (A - t) ** 3 / B ** 4) + 5 * CoeffToUse[5, Axes] * ((A - t) ** 4 / B ** 5) - 6 * CoeffToUse[6, Axes] * (
            (A - t) ** 5 / B ** 6) + 7 * CoeffToUse[7, Axes] * ((A - t) ** 6 / B ** 7)
            AccY = CoeffToUse[0, Axes] * 0 + CoeffToUse[2, Axes] * (2 / (B ** 2)) - 3 * CoeffToUse[3, Axes] * (
            (2 * A - 2 * t) / B ** 3) + 12 * CoeffToUse[4, Axes] * ((A - t) ** 2 / B ** 4) - 20 * CoeffToUse[5, Axes] * (
            (A - t) ** 3 / B ** 5) + 30 * CoeffToUse[6, Axes] * ((A - t) ** 4 / B ** 6) - 42 * CoeffToUse[7, Axes] * (
            (A - t) ** 5 / B ** 7)

            Axes = 2
            PosZ = CoeffToUse[0, Axes] + CoeffToUse[1, Axes] * ((t - A) / B) + CoeffToUse[2, Axes] * ((t - A) / B) ** 2 + \
                   CoeffToUse[3, Axes] * ((t - A) / B) ** 3 + CoeffToUse[4, Axes] * ((t - A) / B) ** 4 + CoeffToUse[
                                                                                                             5, Axes] * ((
                                                                                                                         t - A) / B) ** 5 + \
                   CoeffToUse[6, Axes] * ((t - A) / B) ** 6 + CoeffToUse[7, Axes] * ((t - A) / B) ** 7
            VelZ = CoeffToUse[0, Axes] * 0 + CoeffToUse[1, Axes] * (1 / B) - CoeffToUse[2, Axes] * (
            (2 * A - 2 * t) / B ** 2) + 3 * CoeffToUse[3, Axes] * ((A - t) ** 2 / B ** 3) - 4 * CoeffToUse[4, Axes] * (
            (A - t) ** 3 / B ** 4) + 5 * CoeffToUse[5, Axes] * ((A - t) ** 4 / B ** 5) - 6 * CoeffToUse[6, Axes] * (
            (A - t) ** 5 / B ** 6) + 7 * CoeffToUse[7, Axes] * ((A - t) ** 6 / B ** 7)
            AccZ = CoeffToUse[0, Axes] * 0 + CoeffToUse[2, Axes] * (2 / (B ** 2)) - 3 * CoeffToUse[3, Axes] * (
            (2 * A - 2 * t) / B ** 3) + 12 * CoeffToUse[4, Axes] * ((A - t) ** 2 / B ** 4) - 20 * CoeffToUse[5, Axes] * (
            (A - t) ** 3 / B ** 5) + 30 * CoeffToUse[6, Axes] * ((A - t) ** 4 / B ** 6) - 42 * CoeffToUse[7, Axes] * (
            (A - t) ** 5 / B ** 7)

            PosX_Prev = PosX
            PosY_Prev = PosY
            PosZ_Prev = PosZ
        else:
            PosX = PosX_Prev
            PosY = PosY_Prev
            PosZ = PosZ_Prev
            VelX = 0
            VelY = 0
            VelZ = 0
            AccX = 0
            AccY = 0
            AccZ = 0

        #PosVelAcc = {'PosX': PosX, 'PosY': PosY, 'PosZ': PosZ, 'VelX': VelX, 'VelY': VelY, 'VelZ': VelZ, 'AccX': AccX,
         #            'AccY': AccY, 'AccZ': AccZ}
        return GetStatesResponse(PosX,PosY,PosZ,VelX,VelY,VelZ,AccX,AccY,AccZ)

if __name__ == '__main__':
    #waypoints = np.array([[0,0,0],[1,1,1],[2,0,2],[3,-1,1],[4,0,0]])
    #TimeConstant = 1
    waypoints = np.array(rospy.get_param('/'+sys.argv[1]))
    TimeConstant = rospy.get_param('/'+sys.argv[2])
    GenerateTrajectory(waypoints,TimeConstant,sys.argv[3])
    #PosX_Prev=0
    #PosY_Prev=0
    #PosZ_Prev=0
    #for t in range(0,100,1):
        #prevstatesdic = {'PosX_Prev': PosX_Prev, 'PosY_Prev': PosY_Prev, 'PosZ_Prev': PosZ_Prev}
        #PosVelAcc=trajectoryclass.generate_state(t/10.0,prevstatesdic)
        #print(PosVelAcc)
        #PosX_Prev = PosVelAcc['PosX']
        #PosY_Prev = PosVelAcc['PosY']
        #PosZ_Prev = PosVelAcc['PosZ']
	#try:
   	#	GenerateCoeff()
    #except rospy.ROSInterruptException:
    #    pass
