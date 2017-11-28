#!/usr/bin/env python3

import rosbag
import rospy
import sys
import os
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def trajectoryPlot( bagfile, crazyflie):
	bag=rosbag.Bag(bagfile)
	mocapstatesDir='/'+str(crazyflie)+'/mocap_state'
	targetstatesDir='/'+str(crazyflie)+'/targetstates'
	msg=[]
	msg_t=[]
	msg_tg=[]
	msg_tg_t=[]
	for topic, m, t in bag.read_messages(topics=[mocapstatesDir]):
		msg.append(m)
		msg_t.append(float(t.secs) + float(t.nsecs) / 1e9)
	for topic, m, t in bag.read_messages(topics=[targetstatesDir]):
		msg_tg.append(m)
		msg_tg_t.append(float(t.secs) + float(t.nsecs) / 1e9)


	#t0=msg_t[0]
	#msg_t = [t-t0 for t in msg_t] #Removing the initial time to start at 0
	#t_tg0=msg_tg_t[0]
	#msg_tg_t = [t-t_tg0 for t in msg_tg_t] 
	bag.close()
	#print(msg_t)
	#print(type(msg_t[0]))

	


	x = [s.state.x for s in msg]
	y = [s.state.y for s in msg]
	z = [s.state.z for s in msg]

	x_tg = [s.x for s in msg_tg]
	y_tg = [s.y for s in msg_tg]
	z_tg = [s.z for s in msg_tg]

	print(len(msg_t))
	print(len(msg_tg_t))

	errX=[]
	errY=[]
	errZ=[]

	commonTime=[min(range(len(msg_t)),key=lambda i: abs(msg_t[i]-time)) for time in msg_tg_t]

	for i in range(len(msg_tg_t)):
	#errX.append(abs(x[i]-x_tg[i]) np.where (msg_t==msg_tg_t))
		errX.append(abs(x[commonTime[i]]-x_tg[i]))
		errY.append(abs(y[commonTime[i]]-y_tg[i]))
		errZ.append(abs(z[commonTime[i]]-z_tg[i]))

	maxErrX = np.max(errX)
	maxErrY = np.max(errY)
	maxErrZ = np.max(errZ)

	MSEx = np.mean([err*err for err in errX])
	MSEy = np.mean([err*err for err in errY])
	MSEz = np.mean([err*err for err in errZ])

	
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


	fig3=plt.figure()
	#ax2=fig2.add_subplot(311)
	plt.subplot(311)
	plt.plot(msg_tg_t,errX)
	plt.ylabel('Absolute error on x')
	plt.title('Max err = '+ str(maxErrX)+'    MSE = '+str(MSEx))
	#ax3=fig2.add_subplot(312)
	plt.subplot(312)
	plt.plot(errY)
	plt.ylabel('Absolute error on y')
	plt.title('Max err = '+ str(maxErrY)+'    MSE = '+str(MSEy))
	#ax4=fig2.add_subplot(313)
	plt.subplot(313)
	plt.plot(errZ)
	plt.ylabel('Absolute error on z')
	plt.title('Max err = '+ str(maxErrZ)+'    MSE = '+str(MSEz))
	plt.xlabel('time (s)')


	plt.show()
	return()


if __name__ == '__main__':
	if len(sys.argv) == 2:
		bagfiles = [f for f in os.listdir('.') if f.endswith('.bag')]
		bagfiles.sort()
		bagfile = bagfiles[-1]
		crazyflie = sys.argv[1]
	else:
		bagfile = sys.argv[1]
		crazyflie = sys.argv[2]
	trajectoryPlot( bagfile, crazyflie)





