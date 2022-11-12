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

    for num in range(1,100):
        num += 1
        print("Loading Map: " + str(num) + "%")
        rospy.sleep(0.04)
        os.system("clear")

    # ADD ALL OUR AXIOMS
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

    # ADD DATAPROPERTIES TO OBJECTS
    client.manipulation.add_dataprop_to_ind("visitedAt", "R1", "Long", str(int(time.time())))
    client.manipulation.add_dataprop_to_ind("visitedAt", "R2", "Long", str(int(time.time())))
    client.manipulation.add_dataprop_to_ind("visitedAt", "R3", "Long", str(int(time.time())))
    client.manipulation.add_dataprop_to_ind("visitedAt", "R4", "Long", str(int(time.time())))
    client.manipulation.add_dataprop_to_ind("visitedAt", "C1", "Long", str(int(time.time())))
    client.manipulation.add_dataprop_to_ind("visitedAt", "C2", "Long", str(int(time.time())))
    client.manipulation.add_dataprop_to_ind("visitedAt", "E", "Long", str(int(time.time())))

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




