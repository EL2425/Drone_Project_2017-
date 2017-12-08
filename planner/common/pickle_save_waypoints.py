import pickle

drone1_takeoff = {'x': 0,'y': 0,'z': 1}
drone2_takeoff = {'x': 1,'y': 0,'z': 1}
drone3_takeoff = {'x': 0.5,'y': 1,'z': 1}
area1 = {'x': -0.7,'y': 1,'z': 1}
area2 = {'x': 1.5,'y': 0.8,'z': 1}
area3 = {'x': 0.65,'y': 0.9,'z': 1}
area4 = {'x': 0.5,'y': 0.5,'z': 1}

rz1 = {'x': -1.5,'y': -1,'z': 0.7}
rz2 = {'x': 1.4,'y': -1.5,'z': 0.7}
rz3 = {'x': -1.5,'y': 1.3,'z': 0.7}
rz4 = {'x': 0.0,'y': 0.0,'z': 1.0}


name_waypoints = {'crazyflie1_takeoff': drone1_takeoff, 'crazyflie2_takeoff': drone2_takeoff,
'crazyflie3_takeoff': drone3_takeoff, 'area1': area1, 'area2': area2, 'area3': area3, 'area4': area4,
'rz1': rz1, 'rz2': rz2, 'rz3': rz3, 'rz4': rz4}


pickle_out = open("waypoints.pickle","wb")
pickle.dump(name_waypoints,pickle_out)
pickle_out.close()

#pickle_in = open("dic.pickle","rb")
#example_dict = pickle.load(pickle_in)

#print(example_dict)
