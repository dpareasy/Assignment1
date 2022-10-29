#! /usr/bin/env python3

# Import the armor client class
from armor_client import ArmorClient
from os.path import dirname, realpath

def LoadMap():
    path = dirname(realpath(__file__))
    # Put the path of the file.owl
    path = path + "/../../topological_map/"

    client = ArmorClient("test", "ontology")
    # Initializing with buffered manipulation and reasoning
    client.utils.load_ref_from_file(path + "my_topological_map.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", True, False)

    client.utils.mount_on_ref()
    client.utils.set_log_to_terminal(True)

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


    client.manipulation.remove_ind_from_class("G", "LOCATION")
    print("G has been removed")
    
    # DISJOINT OF THE INDIVIDUALS OF THE CLASSES
    client.manipulation.disj_inds_of_class("R1", "LOCATION")
    client.manipulation.disj_inds_of_class("R2", "LOCATION")
    client.manipulation.disj_inds_of_class("R3", "LOCATION")
    client.manipulation.disj_inds_of_class("R4", "LOCATION")
    client.manipulation.disj_inds_of_class("R5", "LOCATION")
    client.manipulation.disj_inds_of_class("R6", "LOCATION")
    client.manipulation.disj_inds_of_class("C1", "LOCATION")
    client.manipulation.disj_inds_of_class("C2", "LOCATION")
    client.manipulation.disj_inds_of_class("D1", "DOOR")
    client.manipulation.disj_inds_of_class("D2", "DOOR")
    client.manipulation.disj_inds_of_class("D1", "DOOR")
    client.manipulation.disj_inds_of_class("D4", "DOOR")
    client.manipulation.disj_inds_of_class("D5", "DOOR")
    client.manipulation.disj_inds_of_class("D6", "DOOR")
    client.manipulation.disj_inds_of_class("D7", "DOOR")

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

    # APPLY CHANGES AND QUERY
    client.utils.apply_buffered_changes()
    client.utils.sync_buffered_reasoner()

    """
    # Launch reasoner
    client.manipulation.launch_reasoner()
    print("REASONER LAUNCHED")
    
    """

    # SAVE AND EXIT
    client.utils.save_ref_with_inferences(path + "my_topological_map.owl")




