#!/usr/bin/env python3

import rosbag
import sys
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def trajectoryPlot( bagfile, crazyflie):
	bag=rosbag.Bag(bagfile)
	mocapstatesDir='/'+str(crazyflie)+'/mocapstates'
	targetstatesDir='/'+str(crazyflie)+'/targetstates'
	msg = [m for topic, m, t in bag.read_messages(topics=[mocapstatesDir])]
	msg_tg = [m for topic, m, t in bag.read_messages(topics=[targetstatesDir])]
	bag.close()
	#print(msg)
	#print(type(msg[0]))

	x = [tw.linear.x for tw in msg]
	y = [tw.linear.y for tw in msg]
	z = [tw.linear.z for tw in msg]

	x_tg = [tw.linear.x for tw in msg_tg]
	y_tg = [tw.linear.y for tw in msg_tg]
	z_tg = [tw.linear.z for tw in msg_tg]

	fig=plt.figure()
	ax=fig.add_subplot(111, projection='3d')
	ax.plot(x, y, z)
	ax.plot(x_tg, y_tg, z_tg)

	fig2=plt.figure()
	ax2=fig2.add_subplot(311)
	ax2.plot(x)
	ax2.plot(x_tg)
	ax3=fig2.add_subplot(312)
	ax3.plot(y)
	ax3.plot(y_tg)
	ax4=fig2.add_subplot(313)
	ax4.plot(z)
	ax4.plot(z_tg)

	print(bagfile)
	print(crazyflie)

	plt.show()
	return()


if __name__ == '__main__':
	bagfile = sys.argv[1]
	crazyflie = sys.argv[2]
	trajectoryPlot( bagfile, crazyflie)
