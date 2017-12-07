import pickle

drone1_takeoff = {'x': 0,'y': 0,'z': 1}
drone2_takeoff = {'x': 1,'y': 0,'z': 1}
drone3_takeoff = {'x': 0.5,'y': 1,'z': 1}
warehouse1 = {'x': 1,'y': 1,'z': 1}
warehouse2 = {'x': 1.5,'y': 0.5,'z': 1}
customer1 = {'x': -1.5,'y': -1,'z': 0.7}
customer2 = {'x': 1.4,'y': -1.5,'z': 0.7}
customer3 = {'x': -1.5,'y': 1.3,'z': 0.7}

name_waypoints = {'crazyflie1_takeoff': drone1_takeoff, 'crazyflie2_takeoff': drone2_takeoff,
'crazyflie3_takeoff': drone3_takeoff, 'warehouse1': warehouse1, 'warehouse2': warehouse2,
'customer1': customer1, 'customer2': customer2, 'customer3': customer3}


pickle_out = open("waypoints.pickle","wb")
pickle.dump(name_waypoints,pickle_out)
pickle_out.close()

#pickle_in = open("dic.pickle","rb")
#example_dict = pickle.load(pickle_in)

#print(example_dict)
