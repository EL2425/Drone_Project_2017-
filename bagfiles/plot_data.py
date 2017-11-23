#!/usr/bin/env python3

import rosbag
import rospy
import sys
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def trajectoryPlot( bagfile, crazyflie):
	bag=rosbag.Bag(bagfile)
	mocapstatesDir='/'+str(crazyflie)+'/mocapstates'
	targetstatesDir='/'+str(crazyflie)+'/targetstates'
	cmd_vel = '/'+str(crazyflie)+'/cmd_vel'
	msg=[]
	msg_t=[]
	msg_tg=[]
	msg_tg_t=[]
	msg_cmd = []
	msg_cmdt = []
	for topic, m, t in bag.read_messages(topics=[mocapstatesDir]):
		msg.append(m)
		msg_t.append(float(t.secs) + float(t.nsecs) / 1e9)
	for topic, m, t in bag.read_messages(topics=[targetstatesDir]):
		msg_tg.append(m)
		msg_tg_t.append(float(t.secs) + float(t.nsecs) / 1e9)
	for topic, m, t in bag.read_messages(topics=[cmd_vel]):
		msg_cmdt.append(float(t.secs) + float(t.nsecs) / 1e9)
		msg_cmd.append(m)

	#t0=msg_t[0]
	#msg_t = [t-t0 for t in msg_t] #Removing the initial time to start at 0
	#t_tg0=msg_tg_t[0]
	#msg_tg_t = [t-t_tg0 for t in msg_tg_t] 
	bag.close()
	#print(msg_t)
	#print(type(msg_t[0]))

	x = [tw.linear.x for tw in msg]
	y = [tw.linear.y for tw in msg]
	z = [tw.linear.z for tw in msg]

	x_tg = [tw.linear.x for tw in msg_tg]
	y_tg = [tw.linear.y for tw in msg_tg]
	z_tg = [tw.linear.z for tw in msg_tg]

	thrust = [tw.linear.z for tw in msg_cmd]
	pitch = [tw.linear.x for tw in msg_cmd]
	roll = [tw.linear.y for tw in msg_cmd]

	fig=plt.figure()
	ax=fig.add_subplot(111, projection='3d')
	ax.plot(x, y, z)
	ax.plot(x_tg, y_tg, z_tg)

	fig2=plt.figure()
	#ax2=fig2.add_subplot(311)
	plt.subplot(311)
	plt.plot(msg_t,x)
	plt.plot(msg_tg_t, x_tg)
	plt.ylabel('x')
	#ax3=fig2.add_subplot(312)
	plt.subplot(312)
	plt.plot(msg_t, y)
	plt.plot(msg_tg_t, y_tg)
	plt.ylabel('y')
	#ax4=fig2.add_subplot(313)
	plt.subplot(313)
	plt.plot(msg_t, z)
	plt.plot(msg_tg_t, z_tg)
	plt.ylabel('z')
	plt.xlabel('time (s)')

	fig3 = plt.figure()

	plt.subplot(311)
	plt.plot(msg_cmdt, pitch)
	plt.ylabel('pitch')
	plt.subplot(312)
	plt.plot(msg_cmdt, roll)
	plt.ylabel('roll')
	plt.subplot(313)
	plt.plot(msg_cmdt, thrust)
	plt.ylabel('thrust')


	plt.show()
	return()


if __name__ == '__main__':
	bagfile = sys.argv[1]
	crazyflie = sys.argv[2]
	trajectoryPlot( bagfile, crazyflie)





