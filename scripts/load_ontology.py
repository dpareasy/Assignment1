#! /usr/bin/env python3
"""
.. module:: load_onotology
   :platform: Unix
   :synopsis: Python module for defining function used by the state machine

.. moduleauthor:: Davide Leo Parisi <davide.parisi1084@gmail.com>

ROS node for creating the ontology.
"""

# Import the armor client class
import time
import rospy
import os
from armor_api.armor_client import ArmorClient
from os.path import dirname, realpath

client = ArmorClient("assignment", "my_ontology")

class CreateMap:
    """
    This class contains all the methods used to create the topological map.
    """

    def __init__(self):
        print()

    #def LoadMap():
    path = dirname(realpath(__file__))
    # Put the path of the file.owl
    path = path + "/../../topological_map/"

    # Initializing with buffered manipulation and reasoning
    client.utils.load_ref_from_file(path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", False, False)

    client.utils.mount_on_ref()
    client.utils.set_log_to_terminal(True)

    # check if a value is an integer
    def not_int(self,value):
        """
        Function checking if the value is an integer or not.

        Args:
            value: input value

        Returns:
            1: if not an int

        Returns:
            0: if an int

        """
        try:
            int(value)
        except:
            return 1
        else:
            return 0

    # check if the input is an integer number 
    def get_input(self, location_type):
        """
        Function checking if the value taken as input is an integer.

        Args:
            location(int): 0 if corrdiors are request, 1 if rooms are request
        
        Returns:
            number(int): An integern number
        """
        number = ''
        if location_type == 0:
            while self.not_int(number):
                number = input('Enter the number of corridors: ')
        if location_type == 1:
            while self.not_int(number):
                number = input('Enter the number of room for each corridor: ')
        return int(number)

    def LoadMap(self):
        """
        Function initializing the environment in which the robot should move.
        This mechanism is generalized in a way in which many envirnoments can be 
        created. The creation of the environment is done via call to armor server.
        
        """
        # declaration of lists used to store all the individuals
        room_list = []
        door_list = []
        corridor_list = []

        # Input asking the user the number of corridors and rooms for each of them
        #corridor_number = int(input('Specify the number of corridors: '))
        #room_for_corridors = int(input('Specify the number of rooms for corridors: '))
        corridor_number = self.get_input(0)
        room_for_corridors = self.get_input(1)
        room_number = corridor_number*room_for_corridors

        # Create the initial position
        client.manipulation.add_ind_to_class("E", "LOCATION")
        print("Added E to LOCATION")

        # INITIALIZE ROBOT POSITION
        client.manipulation.add_objectprop_to_ind("isIn", "Robot1", "E")
        print("Robot in its initial position!")

        # ADD ALL OUR AXIOMS
        door_number = room_number + 2*corridor_number - 1

        for num in range(1,100):
            num += 1
            print("Loading Map: " + str(num) + "%")
            rospy.sleep(0.04)
            os.system("clear")

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
            client.manipulation.add_dataprop_to_ind("visitedAt", corridor_list[j], "Long", str(int(time.time())))
            print("Added " + corridor_list[j] + " to LOCATION")

        # for cycle for creating all the doors
        for d in range(0,door_number):
            door_list.append('D'+str(d+1))
            client.manipulation.add_ind_to_class(door_list[d], "DOOR")
            print("Added " + door_list[d] + " to DOOR")

        # list to store all the individuals
        ind_list = room_list + corridor_list + door_list
        # Add corridor E to the list of individuals
        ind_list.append('E')

        # DISJOINT OF THE INDIVIDUALS OF THE CLASSES
        client.manipulation.disjoint_all_ind(ind_list)
        print("All individuals are disjointed")

        # make all the connections between locations by assigning the doors to each location
        door_index = 0
        room_index = 0
        # Assigning doors to each location in order to make all the connections
        for l in range(0, corridor_number):
            # The number of doors is higher than the number of rooms for this reason we can assign DOOR[i] to ROOM[i].
            # Here we make also the connections between each corridor with the rooms
            for c in range(0, room_for_corridors):
                client.manipulation.add_objectprop_to_ind('hasDoor', room_list[room_index], door_list[door_index])
                client.manipulation.add_objectprop_to_ind('hasDoor', corridor_list[l], door_list[door_index])
                print('corridor ' + corridor_list[l] + ' connected to ' + room_list[room_index] + ' trough '+ door_list[door_index])
                door_index = door_index + 1
                room_index = room_index + 1
            # Here a create the connections between each corridor with corridor E.
            client.manipulation.add_objectprop_to_ind('hasDoor', corridor_list[l], door_list[door_index])
            client.manipulation.add_objectprop_to_ind('hasDoor', 'E', door_list[door_index])
            print('corridor ' + corridor_list[l] + ' connected to corridor E trough ' + door_list[door_index])
            door_index = door_index + 1

        # It remains to connect the corridor[i+1] with corridor[i] for all the corridors
        for k in range(0, corridor_number-1):
            client.manipulation.add_objectprop_to_ind('hasDoor', corridor_list[k], door_list[door_index])
            client.manipulation.add_objectprop_to_ind('hasDoor', corridor_list[k+1], door_list[door_index])
            print('corridor ' + corridor_list[k] + ' connected to corridor ' + corridor_list[k+1] + ' trough '+ door_list[door_index])
            door_index = door_index + 1

        # APPLY CHANGES AND QUERY
        client.utils.apply_buffered_changes()
        client.utils.sync_buffered_reasoner()

