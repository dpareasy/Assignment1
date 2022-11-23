#! /usr/bin/env python3
"""
.. module:: load_onotology
   :platform: Unix
   :synopsis: Python module for defining function used by the state machine

.. moduleauthor:: Davide Leo Parisi <davide.parisi1084@gmail.com>

ROS node for creating the ontology
"""

# Import the armor client class
import time
import rospy
import os
from armor_client import ArmorClient
from os.path import dirname, realpath
client = ArmorClient("assignment", "my_ontology")


#def LoadMap():
path = dirname(realpath(__file__))
# Put the path of the file.owl
path = path + "/../../topological_map/"


# Initializing with buffered manipulation and reasoning
client.utils.load_ref_from_file(path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", False, False)

client.utils.mount_on_ref()
client.utils.set_log_to_terminal(True)

def LoadMap():
    """
    Function initializing the environment in which the robot should move.
    This mechanism is generalized in a way in which every envirnoment can be 
    created. The creation of the environment is done via call to armor server.
    
    """
    # declaration of lists used to store all the individuals
    room_list = []
    door_list = []
    corridor_list = []
    #for num in range(1,100):
    #    num += 1
    #    print("Loading Map: " + str(num) + "%")
    #    rospy.sleep(0.04)
    #    os.system("clear")

    # Input asking the user the number of corridors and rooms for each of them
    corridor_number = int(input('Specify the number of corridors: '))
    room_for_corridors = int(input('Specify the number of rooms for corridors: '))
    room_number = corridor_number*room_for_corridors

    # ADD ALL OUR AXIOMS
    door_number = room_number + 2*corridor_number - 1

    # for cycle for creating all the rooms
    for i in range(0, room_number):
        room_list.append('R'+ str(i+1))
        client.manipulation.add_ind_to_class(room_list[i], "LOCATION")
        client.manipulation.add_dataprop_to_ind("visitedAt", room_list[i], "Long", str(int(time.time())))
        print("Added " + room_list[i] + " to LOCATION")
    
    # for cycle for creating all the corridors
    for j in range(0,corridor_number):
        corridor_list.append('C' + str(j+1))
        client.manipulation.add_ind_to_class(corridor_list[j], "LOCATION")
        print("Added " + corridor_list[j] + " to LOCATION")

    for d in range(0,door_number):
        door_list.append('D'+str(d+1))
        client.manipulation.add_ind_to_class(door_list[d], "DOOR")
        print("Added " + door_list[d] + " to DOOR")
    
    #creation of the recharging location
    corridor_list.append('E')
    client.manipulation.add_ind_to_class("E", "LOCATION")
    print("Added E to LOCATION")
    client.manipulation.add_objectprop_to_ind("hasDoor", "E", "D6")
    client.manipulation.add_objectprop_to_ind("hasDoor", "E", "D7")

    # list to store all the individuals
    ind_list = room_list + corridor_list + door_list


    # DISJOINT OF THE INDIVIDUALS OF THE CLASSES
    client.manipulation.disjoint_all_ind(ind_list)
    print("All individuals are disjointed")

    # make all the connections between locations by assigning the doors to each location
    n_room_for_corridor = int(len(room_list)/(len(corridor_list)-1))
    door_index = 0
    room_index = 0
    for l in range(0,len(corridor_list)-1):
        for c in range(0, n_room_for_corridor):
            client.manipulation.add_objectprop_to_ind('hasDoor', room_list[room_index], door_list[door_index])
            client.manipulation.add_objectprop_to_ind('hasDoor', corridor_list[l], door_list[door_index])
            print('corridor ' + corridor_list[l] + ' connected to ' + room_list[room_index] + ' trough '+ door_list[door_index])
            door_index = door_index + 1
            room_index = room_index + 1
        client.manipulation.add_objectprop_to_ind('hasDoor', corridor_list[l], door_list[door_index])
        client.manipulation.add_objectprop_to_ind('hasDoor', 'E', door_list[door_index])
        print('corridor ' + corridor_list[l] + ' connected to corridor E trough ' + door_list[door_index])
        door_index = door_index + 1

    for k in range(0, len(corridor_list)-2):
        client.manipulation.add_objectprop_to_ind('connectedTo', corridor_list[k], corridor_list[k+1])
        client.manipulation.add_objectprop_to_ind('hasDoor', corridor_list[k], door_list[door_index])
        client.manipulation.add_objectprop_to_ind('hasDoor', corridor_list[k+1], door_list[door_index])
        print('corridor ' + corridor_list[k] + ' connected to corridor ' + corridor_list[k+1] + ' trough '+ door_list[door_index])
        door_index = door_index + 1

    # INITIALIZE ROBOT POSITION
    client.manipulation.add_objectprop_to_ind("isIn", "Robot1", "E")
    print("Robot in its initial position!")

    # APPLY CHANGES AND QUERY
    client.utils.apply_buffered_changes()
    client.utils.sync_buffered_reasoner()