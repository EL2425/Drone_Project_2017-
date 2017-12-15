import pickle

drone1_takeoff = {'x': 0,'y': 0,'z': 1}
drone2_takeoff = {'x': 1,'y': 0,'z': 1}
drone3_takeoff = {'x': 0.5,'y': 1,'z': 1}
drone4_takeoff = {'x': -0.5,'y': 1,'z': 1}
area1 = {'x': 0.551,'y': 0.1395,'z': 1}
area2 = {'x': 1.471,'y': 1.121,'z': 1}
area3 = {'x': 0.633,'y': 1.178,'z': 1}
area4 = {'x': -0.376,'y': 1.254,'z': 1}
area5 = {'x': 1.479,'y': 0.113,'z': 1}

rz1 = {'x': 0.993,'y': -1.1,'z': 0.7}
rz2 = {'x': -0.223,'y': -1.639,'z': 0.7}
rz3 = {'x': -1.607,'y': -0.923,'z': 0.7}
rz4 = {'x': -1.217,'y': 0.263,'z': 1.0}


name_waypoints = {'crazyflie5_takeoff': drone1_takeoff, 'crazyflie2_takeoff': drone2_takeoff,
'crazyflie3_takeoff': drone3_takeoff, 'crazyflie4_takeoff': drone4_takeoff, 'area1': area1, 'area2': area2, 'area3': area3, 'area4': area4,
'rz1': rz1, 'rz2': rz2, 'rz3': rz3, 'rz4': rz4}


pickle_out = open("waypoints.pickle","wb")
pickle.dump(name_waypoints,pickle_out)
pickle_out.close()

#pickle_in = open("dic.pickle","rb")
#example_dict = pickle.load(pickle_in)

#print(example_dict)
