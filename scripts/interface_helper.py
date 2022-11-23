#!/usr/bin/env python3

"""
.. module:: interface_helper
   :platform: Unix
   :synopsis: Python module for defining function used by the state machine

.. moduleauthor:: Luca Buoncompagni, Davide Leo Parisi <davide.parisi1084@gmail.com>

ROS node for defining the behavior of the state machine

Subscribes to:
    /state/battery_low where the robot_state publishes the battery status
"""

# Import ROS libraries.
import rospy
import random
import time
import os
import simple_colors
from actionlib import SimpleActionClient

# Import mutex to manage synchronization among ROS-based threads (i.e., node loop and subscribers)
from threading import Lock
from armor_client import ArmorClient

# Import ROS-based messages.
from std_msgs.msg import Bool
from Assignment1.msg import PlanAction, ControlAction
from Assignment1.srv import SetPose

client = ArmorClient("assignment", "my_ontology")

class ActionClientHelper:
    """
    A class to simplify the implementation of a client for ROS action servers. It is used by the `InterfaceHelper` class.

    Class constructor, i.e., class initializer. Input parameters are:

    - `service_name`: it is the name of the server that will be invoked by this client.

    - `action_type`: it is the message type that the server will exchange.

    - `done_callback`: it is the name of the function called when the action server completed its computation. If this parameter is not set (i.e., set to `None`), then only the `self._done_callback` function will be called when the server completes its computation.

    - `feedback_callback`: it is the name of the function called when the action server sends a feedback message. If this parameter is not set (i.e., set to `None`), then only the `self._feedback_callback` functions will be called when the server sends a feedback message.

    - `mutex`: it is a `Lock` object synchronised with the `done_callback` and `feedback_callback`. If it is not set (i.e., set to `None`), then a new mutex instance is considered. Set this variable if you want to extends the synchronization with other classes.
    """
    def __init__(self, service_name, action_type, done_callback=None, feedback_callback=None, mutex=None):
        # Initialise the state of this client, i.e.,  `_is_running`, `_is_done`, and `_results`.
        self.reset_client_states()
        # Set the name of the server to be invoked.
        self._service_name = service_name
        # Get or create a new mutex.
        if mutex is None:
            self._mutex = Lock()
        else:
            self._mutex = mutex
        # Instantiate a simple ROS-based action client.
        self._client = SimpleActionClient(service_name, action_type)
        # Set the done and feedback callbacks defined by the class using this client.
        self._external_done_cb = done_callback
        self._external_feedback_cb = feedback_callback
        # Wait for the action server to be alive.
        self._client.wait_for_server()

    def send_goal(self, goal):
        """
        Start the action server with a new `goal`. Note this call is not blocking (i.e., asynchronous performed).

        Args:
            goal(Point): Goal point

        """
        # A new goal can be given to the action server only if it is not running. This simplification implies that
        # within the ROS architecture no more than one client can use the same server at the same time.
        if not self._is_running:
            # Start the action server.
            self._client.send_goal(goal,
                                   done_cb = self._done_callback,
                                   feedback_cb = self._feedback_callback)
            # Set the client's states.
            self._is_running = True
            self._is_done = False
            self._results = None
        else:
            print("Warning send a new goal, cancel the current request first!")

    def cancel_goals(self):
        """
        Stop the computation of the action server.
        """
        # The computation can be stopped only if the server is actually computing.
        if self._is_running:
            # Stop the computation.
            self._client.cancel_all_goals()
            # Reset the client's state.
            self.reset_client_states()
        else:
            print("Warning cannot cancel a not running service!")

    def reset_client_states(self):
        """
        Reset the client state variables stored in this class.
        """
        self._is_running = False
        self._is_done = False
        self._results = None

    def _feedback_callback(self, feedback):
        """
        This function is called when the action server send some `feedback` back to the client.

        Args:
            feedback: feedbacks from the action server

        """
        # Acquire the mutex to synchronise the computation concerning the `feedback` message with the other nodes of the architecture.
        self._mutex.acquire()
        try:
            # Eventually, call the method provided by the node that uses this action client to manage a feedback.
            if self._external_feedback_cb is not None:
                self._external_feedback_cb(feedback)
        finally:
            # Realise the mutex to (eventually) unblock ROS-based thread waiting on the same mutex.
            self._mutex.release()

    def _done_callback(self, status, results):
        """
        This function is called when the action server finish its 
        computation, i.e., it provides a `done` message.

        Args:
            status: 

            results(): results from the action server

        """
        # Acquire the mutex to synchronise the computation concerning the `done` message with the other nodes of the architecture.
        self._mutex.acquire()
        try:
            # Set the client's state
            self._is_running = False
            self._is_done = True
            self._results = results
            # Eventually, call the method provided by the node that uses this action client to manage a result.
            if self._external_done_cb is not None:
                self._external_done_cb(status, results)
        finally:
            self._mutex.release()

    def is_done(self):  # they should be mutex safe
        """
        Get `True` if the action server finished is computation, or `False` otherwise.
        Note that use this method should do it in a `self._mutex` safe manner.

        Results:
            self._is_done(bool): If the reasoner has finished its calculations.

        """
        return self._is_done

    def is_running(self):
        """
        Get `True` if the action server is running, or `False` otherwise.
        A note that use this method should do it in a `self._mutex` safe manner.

        Returns:
            self._is_running: If the server is still running.

        """
        return self._is_running
 
    def get_results(self):
        """
        Get the results of the action server, if any, or `None` and return this value.

        Returns:
            self._results(): Some results have arrived.
        Returns:
            None: No results arrived

        """
        if self._is_done:
            return self._results
        else:
            print("Error: cannot result")
            return None

class InterfaceHelper:
    """
    A class to decouple the implementation of the Finite State Machine to the stimulus might that
    lead to state transitions. This class manages the synchronization with subscribers and action
    servers.
    """

    # Class constructor, i.e., class initializer.
    def __init__(self):
        # Create a shared mutex to synchronize action clients and subscribers.
        # Note that, based on different assumptions, further optimization can be done to make the different threads
        # blocking for a less amount of time in the same mutex.
        self.mutex = Lock()
        # Set the initial state involving the `self._battery_low`, `self._start_interaction` and `self._gesture` variables.
        self.reset_states()
        # Define the callback associated with the speech, gesture, and battery low ROS subscribers.
        rospy.Subscriber('state/battery_low', Bool, self._battery_callback)
        # Define the clients for the the plan and control action servers.
        self.planner_client = ActionClientHelper('motion/planner', PlanAction, mutex=self.mutex)
        self.controller_client = ActionClientHelper('motion/controller', ControlAction, mutex=self.mutex)

    def reset_states(self):
        """
        Reset the stimulus, which are stored as states variable fo this class.
        This function assumes that no states of the Finite State Machine run concurrently.
        """
        self._battery_low = False
        self._start_interaction = False
        self._gesture = None

    def _battery_callback(self, msg):
        """
        The subscriber to get messages published from the `robot-state` node into the `/state/battery_low/` topic.
        """
        # Acquire the mutex to assure the synchronization with the other subscribers and action clients (this assure data consistency).
        self.mutex.acquire()
        try:
            # Get the battery level and set the relative state variable encoded in this class.
            self._battery_low = msg.data
        finally:
            # Release the mutex to eventually unblock the other subscribers or action servers that are waiting.
            self.mutex.release()

    def is_battery_low(self):
        """
        Get the state variable encoded in this class that concerns the battery level.
        The returning value will be `True` if the battery is low, `False` otherwise.
        Note that the node using this class might exploit the `reset_state` function to improve robustness.
        Also note that this function should be used when the `mutex` has been acquired. This assures the
        synchronization  with the threads involving the subscribers and action clients.

        Returns:
            self._battery_low(bool): `True` if the battery is low, `False` otherwise

        """
        return self._battery_low

    # Update the current robot pose stored in the `robot-state` node.
    @staticmethod
    def init_robot_pose(point):
        # Eventually, wait for the server to be initialised.
        rospy.wait_for_service('state/set_pose')
        try:
            # Call the service and set the current robot position.
            service = rospy.ServiceProxy('state/set_pose', SetPose)
            service(point)  # None that the service `response` is not used.
            print("Setting initial robot position")
        except rospy.ServiceException as e:
            print("Cannot set current robot position")

class BehaviorHelper:
    """
    A class to define the methods used for implementing the behavior of the robot. 
    """

    def __init__(self):
        self.agent = "Robot1"
        self.current_position = "isIn"
        self.urgent_class = "URGENT"
        self.reachable = "canReach"
        self.last_visit = "visitedAt"
        self.robot_timestamp = "now"
        self.charging_location = "E"
        self.time_stamps_type = "Long"
        self.corridors = 'CORRIDOR'

        self.urgent = 'urgent'
        self.corridor = 'corridor'
        self.robot_position = 'robot_position'
        self.reachable_destinations = 'reachable_destinations'

    def inters_b2_lists(self, list1, list2):
        """
        Intersection between the urgent rooms and the reacheble rooms.

        Args:
            list1(str): first list to intersect

            list2(str): second list to intersect

        Returns:
            intersection(str): list of the intersection

        """
        intersection = []
        for element in list1:
            if element in list2:
                intersection.append(element)
        return intersection

    def clean_strings(self, string_type, list_):
        """
        clean the strings obtained by the query.

        Args:
            string_type(int): an integer specifying if the string is obtained from
            
            list_(str): list of string to be cleaned 

        Returns:
            list_(str): list of cleaned strings

        """
        if string_type == 1:
            if len(list_) == 1:
                list_[0] = list_[0][32:-1]
            else:
                for x in range (len(list_)):
                    list_[x] = list_[x][32:-1]
        if string_type == 2:
            list_ = list_[0][1:-11]
        return list_

    def get_locations(self, location):
        """
        function for location's query

        Args:
            location(str): a string to select the query to ask

        Returns:
            location_list(str): list of queried location

        """

        if location == self.corridor:
            list_of_corridors = client.query.ind_b2_class(self.corridors)
            location_list = self.clean_strings(1, list_of_corridors)
            
        if location == self.urgent:
            urgent_rooms = client.query.ind_b2_class(self.urgent_class)
            location_list = self.clean_strings(1, urgent_rooms)

        if location == self.robot_position:
            current_pose = client.query.objectprop_b2_ind(self.current_position, self.agent)
            location = self.clean_strings(1, current_pose)
            location_list = current_pose[0]

        if location == self.reachable_destinations:
            possible_destinations = client.query.objectprop_b2_ind(self.reachable, self.agent)
            location_list = self.clean_strings(1, possible_destinations)

        return location_list


    def decide_target(self):
        """
        Plan the next robot movement.

        Returns:
            current_pose(str): The current robot position obtained from the ontology

        Returns:
            target(str): The target position chosen from a list of reachable and urgent locations
        
        Returns:
            list_of_corridors(str): List of corridors in the map

        """

        client.utils.sync_buffered_reasoner()
        list_of_corridors = self.get_locations(self.corridor)
        current_pose = self.get_locations(self.robot_position)
        print(simple_colors.cyan("current pose is: ["+(current_pose)+"]"))
        possible_destinations = self.get_locations(self.reachable_destinations)
        print(simple_colors.cyan("Possible destinations: [" + ", ".join(possible_destinations) +"]"))
        urgent_rooms = self.get_locations(self.urgent)
        print(simple_colors.red("URGENT ROOMS: [" + ", " .join(urgent_rooms) + "]"))
        reachable_urgent = self.inters_b2_lists(possible_destinations, urgent_rooms)
        print(simple_colors.green("URGENT AND REACHABLE LOCATIONS: ["+", ".join(reachable_urgent)+"]"))
        # if the list is empty
        if not reachable_urgent:
            # if the list of possible destination 
            # contains only one element
            if len(possible_destinations) == 1:
                target = possible_destinations[0]
            else:
                target = random.choice(self.inters_b2_lists(possible_destinations, list_of_corridors))
        # if chosen_target list is not empty
        else:
            # save the first element of the list as the oldest timestamp
            oldest = client.query.dataprop_b2_ind(self.last_visit, reachable_urgent[0])
            # clean the string
            oldest = self.clean_strings(2, oldest)
            for i in range (len(reachable_urgent)):
                choice_last_visit = client.query.dataprop_b2_ind(self.last_visit, reachable_urgent[i])
                choice_last_visit = self.clean_strings(2, choice_last_visit)
                if choice_last_visit <= oldest:
                    target = reachable_urgent[i]
        print(simple_colors.cyan("Moving to " + target))
        return current_pose, target, list_of_corridors


    def move_to_target(self, chosen_target, current_pose, list_of_corridors):
        """
        Moving to the target room.

        Args:
            chosen_target(str): The target position chosen from a list of reachable and urgent locations
            current_pose(str): The current robot position obtained from the ontology
            list_of_corridors(str): List of corridors  in the map

        """
        last_visit = client.query.dataprop_b2_ind(self.last_visit, chosen_target)

        if chosen_target not in list_of_corridors:
            last_visit = self.clean_strings(2, last_visit)

        client.manipulation.replace_objectprop_b2_ind(self.current_position, self.agent, chosen_target, current_pose)
        client.utils.sync_buffered_reasoner()
        last_change = client.query.dataprop_b2_ind(self.robot_timestamp, self.agent)
        last_change = self.clean_strings(2, last_change)
        print(simple_colors.cyan("last change was " + last_change))
        current_time = str(int(time.time()))

        if chosen_target not in list_of_corridors:
            client.manipulation.replace_dataprop_b2_ind(self.last_visit, chosen_target, self.time_stamps_type, current_time, last_visit)

        client.manipulation.replace_dataprop_b2_ind(self.robot_timestamp, self.agent, self.time_stamps_type, current_time, last_change)
        client.utils.sync_buffered_reasoner()
        print(simple_colors.cyan("Now the robot is in " + chosen_target))


    def go_to_recharge(self, current_location):
        """
        Going to charging location.

        Args:
            current_location(str): The current robot position obtained from the ontology

        """
        client.manipulation.replace_objectprop_b2_ind(self.current_position, self.agent, self.charging_location, current_location)
        client.utils.sync_buffered_reasoner()