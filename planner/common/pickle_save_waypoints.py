import pickle 

drone1_takeoff = {'x': 0,'y': 0,'z': 1}
drone2_takeoff = {'x': -1,'y': 0,'z': 1}
drone3_takeoff = {'x': -1,'y': -1,'z': 1}
warehouse1 = {'x': -3,'y': -3,'z': 1}
warehouse2 = {'x': -3,'y': 3,'z': 1}
customer1 = {'x': -2,'y': -1,'z': 1}
customer2 = {'x': -1,'y': -2,'z': 1}
customer3 = {'x': -2,'y': -2,'z': 1}

name_waypoints = {'drone1_takeoff': drone1_takeoff, 'drone2_takeoff': drone2_takeoff, 
'drone3_takeoff': drone3_takeoff, 'warehouse1': warehouse1, 'warehouse2': warehouse2, 
'customer1': customer1, 'customer2': customer2, 'customer3': customer3}


pickle_out = open("waypoints.pickle","wb")
pickle.dump(name_waypoints,pickle_out)
pickle_out.close()

#pickle_in = open("dic.pickle","rb")
#example_dict = pickle.load(pickle_in)

#print(example_dict)