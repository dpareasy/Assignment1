#! /usr/bin/env python3

# Import the armor client class
from armor_client import ArmorClient
from os.path import dirname, realpath

client = ArmorClient("assignment", "my_ontology2")


#def LoadMap():
path = dirname(realpath(__file__))
# Put the path of the file.owl
path = path + "/../../topological_map/"

# Initializing with buffered manipulation and reasoning
client.utils.load_ref_from_file(path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", True, False)

client.utils.mount_on_ref()
client.utils.set_log_to_terminal(True)    

client.manipulation.add_ind_to_class("R1", "LOCATION")
print("Added R1 to LOCATION")
client.manipulation.add_ind_to_class("R2", "LOCATION")
print("Added R2 to LOCATION")
client.manipulation.add_ind_to_class("R3", "LOCATION")
print("Added R3 to LOCATION")
client.manipulation.add_ind_to_class("R4", "LOCATION")
print("Added R4 to LOCATION")
client.manipulation.add_ind_to_class("C1", "LOCATION")
print("Added C1 to LOCATION")
client.manipulation.add_ind_to_class("C2", "LOCATION")
print("Added C2 to LOCATION")
client.manipulation.add_ind_to_class("E", "LOCATION")
print("Added E to LOCATION")
client.manipulation.add_ind_to_class("D1", "DOOR")
print("Added D1 to DOOR")
client.manipulation.add_ind_to_class("D2", "DOOR")
print("Added D2 to DOOR")
client.manipulation.add_ind_to_class("D3", "DOOR")
print("Added D3 to DOOR")
client.manipulation.add_ind_to_class("D4", "DOOR")
print("Added D4 to DOOR")
client.manipulation.add_ind_to_class("D5", "DOOR")
print("Added D5 to DOOR")
client.manipulation.add_ind_to_class("D6", "DOOR")
print("Added D6 to DOOR")
client.manipulation.add_ind_to_class("D7", "DOOR")
print("Added D7 to DOOR")

# DISJOINT OF THE INDIVIDUALS OF THE CLASSES

client.manipulation.disj_inds_of_class("LOCATION")
client.manipulation.disj_inds_of_class("DOOR")
print("All individuals are disjointed")

# ADD PROPERTIES TO OBJECTS
# Distinction between rooms and corridors
client.manipulation.add_objectprop_to_ind("hasDoor", "R1", "D1")
client.manipulation.add_objectprop_to_ind("hasDoor", "R2", "D2")
client.manipulation.add_objectprop_to_ind("hasDoor", "R3", "D3")
client.manipulation.add_objectprop_to_ind("hasDoor", "R4", "D4")
client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D1")
client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D2")
client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D5")
client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D7")
client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D3")
client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D4")
client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D5")
client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D6")
client.manipulation.add_objectprop_to_ind("hasDoor", "E", "D6")
client.manipulation.add_objectprop_to_ind("hasDoor", "E", "D7")

print("All properties added!")

# Connections between locations
client.manipulation.add_objectprop_to_ind("connectedTo", "R1", "C1")
client.manipulation.add_objectprop_to_ind("connectedTo", "R2", "C1")
client.manipulation.add_objectprop_to_ind("connectedTo", "C1", "C2")
client.manipulation.add_objectprop_to_ind("connectedTo", "R3", "C2")
client.manipulation.add_objectprop_to_ind("connectedTo", "R4", "C2")
client.manipulation.add_objectprop_to_ind("connectedTo", "E", "C1")
client.manipulation.add_objectprop_to_ind("connectedTo", "E", "C2")

print("All connections declared!")

# INITIALIZE ROBOT POSITION
client.manipulation.add_objectprop_to_ind("isIn", "Robot1", "E")
print("Robot in its initial position!")

client.manipulation.replace_objectprop_b2_ind("isIn", "Robot1", "C1", "E")

# SAVE AND EXIT
#client.utils.save_ref_with_inferences(path + "topological_map.owl")


#h = 0
    #for l in range(0,len(corridor_list)):
    #    for c in range(0, n_room_for_corridor):
    #        client.manipulation.add_objectprop_to_ind('connectedTo', room_list[h], corridor_list[l])
    #        print('corridor ' + corridor_list[l] + ' connected to ' + room_list[h])
    #        h = h+1
#######################################################à
#######################################################

#! /usr/bin/env python3

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

    room_list = []
    door_list = []
    corridor_list = []
    #for num in range(1,100):
    #    num += 1
    #    print("Loading Map: " + str(num) + "%")
    #    rospy.sleep(0.04)
    #    os.system("clear")


    # input
    # ADD ALL OUR AXIOMS
    for i in range(0,4):
        room_list.append('R'+ str(i+1))
        client.manipulation.add_ind_to_class(room_list[i], "LOCATION")
        print("Added " + room_list[i] + " to LOCATION")
    
    for j in range(0,2):
        corridor_list.append('C' + str(j+1))
        client.manipulation.add_ind_to_class(corridor_list[j], "LOCATION")
        print("Added " + corridor_list[j] + " to LOCATION")

    corridor_list.append('E')
    client.manipulation.add_ind_to_class("E", "LOCATION")
    print("Added E to LOCATION")

    for d in range(0,7):
        door_list.append('D'+str(d+1))
        client.manipulation.add_ind_to_class(door_list[d], "DOOR")
        print("Added " + door_list[d] + " to DOOR")
    
    ind_list = room_list + corridor_list + door_list
    # DISJOINT OF THE INDIVIDUALS OF THE CLASSES
    #ArmorClient.call('DISJOINT', 'IND', '', ind_list)
    client.manipulation.disjoint_all_ind(ind_list)
    #client.manipulation.disj_inds_of_class("DOOR")
    print("All individuals are disjointed")

    # ADD PROPERTIES TO OBJECTS
    # Distinction between rooms and corridors
    client.manipulation.add_objectprop_to_ind("hasDoor", "R1", "D1")
    client.manipulation.add_objectprop_to_ind("hasDoor", "R2", "D2")
    client.manipulation.add_objectprop_to_ind("hasDoor", "R3", "D3")
    client.manipulation.add_objectprop_to_ind("hasDoor", "R4", "D4")
    client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D1")
    client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D2")
    client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D5")
    client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D7")
    client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D3")
    client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D4")
    client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D5")
    client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D6")
    client.manipulation.add_objectprop_to_ind("hasDoor", "E", "D6")
    client.manipulation.add_objectprop_to_ind("hasDoor", "E", "D7")
    print("All properties added!")

    # ADD DATAPROPERTIES TO OBJECTS
    client.manipulation.add_dataprop_to_ind("visitedAt", "R1", "Long", str(int(time.time())))
    client.manipulation.add_dataprop_to_ind("visitedAt", "R2", "Long", str(int(time.time())))
    client.manipulation.add_dataprop_to_ind("visitedAt", "R3", "Long", str(int(time.time())))
    client.manipulation.add_dataprop_to_ind("visitedAt", "R4", "Long", str(int(time.time())))

    # Connections between locations
    client.manipulation.add_objectprop_to_ind("connectedTo", "R1", "C1")
    client.manipulation.add_objectprop_to_ind("connectedTo", "R2", "C1")
    client.manipulation.add_objectprop_to_ind("connectedTo", "C1", "C2")
    client.manipulation.add_objectprop_to_ind("connectedTo", "R3", "C2")
    client.manipulation.add_objectprop_to_ind("connectedTo", "R4", "C2")
    client.manipulation.add_objectprop_to_ind("connectedTo", "E", "C1")
    client.manipulation.add_objectprop_to_ind("connectedTo", "E", "C2")

    print("All connections declared!")

    # INITIALIZE ROBOT POSITION
    client.manipulation.add_objectprop_to_ind("isIn", "Robot1", "E")
    print("Robot in its initial position!")

    # APPLY CHANGES AND QUERY
    client.utils.apply_buffered_changes()
    client.utils.sync_buffered_reasoner()