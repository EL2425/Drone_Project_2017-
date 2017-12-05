#! /usr/bin/env python
from pddl_functions import *

supplies = ['package1', 'package2', 'package3']
drones = ['drone1','drone2']
location = ['warehouse1','warehouse2','customer1','customer2','customer3']

def add_knowledge():
    # Add items
    for item in supplies:
    	add_instance('package', item)

    for item in drones:
        add_instance('drone',item)

    for item in location:
        add_instance('location',item)

    add_fact('at-drone',{'d':'drone1', 'l':'warehouse1'})
    add_fact('at-drone',{'d':'drone2', 'l':'warehouse2'})
    add_fact('at-package',{'p':'package1', 'l':'warehouse1'})
    add_fact('at-package',{'p':'package2', 'l':'warehouse2'})
    add_fact('at-package',{'p':'package3', 'l':'warehouse1'})
    #add_fact('at-package',{'p':'package4', 'l':'warehouse2'})
    add_fact('drone-empty',{'d':'drone1'})
    add_fact('drone-ground',{'d':'drone1'})
    add_fact('drone-empty',{'d':'drone2'})
    add_fact('drone-ground',{'d':'drone2'})

    # Goals
    add_goal('at-package',{'p':'package1','l':'customer1'})
    add_goal('at-package',{'p':'package2','l':'customer2'})
    add_goal('at-package',{'p':'package3','l':'customer3'})
    #add_goal('at-package',{'p':'package4','l':'customer3'})
    add_goal('at-drone',{'d':'drone1','l':'warehouse1'})
    add_goal('at-drone',{'d':'drone2','l':'warehouse2'})
    add_goal('drone-ground',{'d':'drone1'})
    add_goal('drone-ground',{'d':'drone2'})

    



"""

(at-package package1 customer1)
(at-package package2 customer2)
(at-package package3 customer3)
(at-drone drone1 warehouse1)
(at-drone drone2 warehouse2)
(drone-ground drone1)
(drone-ground drone2)


(at-drone drone1 warehouse1)
(at-drone drone2 warehouse2)
(at-package package1 warehouse1)
(at-package package2 warehouse2)
(at-package package3 warehouse1)
(drone-empty drone1)
(drone-ground drone1)
(drone-empty drone2)
(drone-ground drone2)

    humans = [Human('linnea', (3,1)), Human('batman',(2,2))]
    crates = [Crate('crate1','food',(1,0)),Crate('crate2', 'water')]

    drone  = Drone('ardrone', (0,0))
    robot  = Robot('turtlebot', (0,0))


    # Add waypoints
    connecting_distance = 1.42
    path = os.path.dirname(os.path.abspath(__file__))
    waypoints = load_waypoints(path + '/waypoints.txt')
    xdim = 3;
    ydim = 3;
    z = 0;
    j = 0;
    for x in np.linspace(-xdim,xdim,6):
        i = 0;
        for y in np.linspace(-ydim,ydim,6):
            pos = [x, y, z]
            name = 'wp_' + str(i) + '_' + str(j)
            add_instance('waypoint', name)
            add_waypoint(name, pos, connecting_distance)
            i = i + 1;
        j = j+1

    # GOALS
    humans[1].needs("water")
    humans[0].needs("food")
    rospy.sleep(1)
"""
if __name__ == '__main__':
    start_ppdl_node()
    add_knowledge()


