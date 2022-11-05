#!/usr/bin/env python3 

import roslib
import smach
import time
import rospy
from pydoc import Helper
import random
import string
import os
import smach_ros
from threading import Lock
from std_msgs.msg import Bool
from smach import StateMachine, State
from armor_client import ArmorClient
from load_ontology import LoadMap
from armor_client import ArmorClient
from Assignment1 import architecture_name_mapper as anm
from interface_helper import InterfaceHelper
from Assignment1.msg import Point, ControlGoal, PlanGoal
from os.path import dirname, realpath

#list of states in the machine
STATE_INIT = 'INITIALIZE_MAP'
STATE_DECISION = 'DECIDE_LOCATION'
STATE_MOVING = "MOVING_TO_LOCATION"
STATE_PLAN_TO_RANDOM_POSE = 'PLAN_TO_RANDOM_POSE'
STATE_GO_TO_RANDOM_POSE = 'GO_TO_RANDOM_POSE'
STATE_NORMAL = 'NORMAL'
STATE_RECHARGING = 'RECHARGING'

# list of transition states
TRANS_INITIALIZED = 'everithing_loaded'
TRANS_DECIDED = 'target_acquired'
TRANS_MOVED = 'robot_moved'
TRANS_BATTERY_LOW = 'battery_low'
TRANS_PLANNED_TO_RANDOM_POSE = 'planned_to_random_pose'
TRANS_RECHARGING = 'recharging'
TRANS_WENT_RANDOM_POSE = 'went_to_random_pose'
TRANS_RECHARGED = 'recharged'

# Sleeping time (in seconds) of the waiting thread to allow the computations
# for getting stimulus from the other components of the architecture.
LOOP_SLEEP_TIME = 0.3

client = ArmorClient("assignment", "my_ontology")

class LoadOntology(smach.State):
    def __init__(self, interface_helper):
        State.__init__(self, outcomes = [TRANS_INITIALIZED])

    def execute(self, userdata):
        LoadMap()
        print("MAP ACQUIRED")
        return TRANS_INITIALIZED

class DecideTarget(smach.State):
    def __init__(self, interface_helper):
        State.__init__(self, outcomes = [TRANS_DECIDED], output_keys = ['current_pose', 'choice'])

    def execute(self, userdata):
        
        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.controller_client.cancel_goals()
                    return TRANS_RECHARGING
                # If the controller finishes its computation, then take the `went_random_pose` transition, which is related to the `repeat` transition.
                if self._helper.controller_client.is_done():
                    current_pose = client.query.objectprop_b2_ind("isIn","Robot1")
                    current_pose = current_pose[0][32:]
                    current_pose = current_pose[:len(current_pose) - 1]
                    userdata.current_pose = current_pose
                    print("\ncurrent pose is " + current_pose)
                    destination = client.query.objectprop_b2_ind("canReach", "Robot1")
                    print(destination)
                    if len(destination) == 1:
                        choice = destination[0]
                    else:
                        choice = random.choice(destination)
                    choice = choice[32:]
                    choice = choice[:len(choice) - 1]
                    userdata.choice = choice
                    print(choice)
                    return TRANS_DECIDED
                # Note that if unexpected stimulus comes from the other nodes of the architecture through the
                # `self._helper` class, then this state will not take any transitions. This is equivalent to have a
                # loop-like transition in the behavioural UML diagram for all the other stimulus except
                # `TRANS_RECHARGING`, `TRANS_START_INTERACTION` and `TRANS_WENT_RANDOM_POSE`.
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)
        return TRANS_DECIDED

class MoveToTarget(smach.State):
    def __init__(self, interface_helper):
        State.__init__(self, outcomes = [TRANS_MOVED], input_keys = ['current_pose', 'choice'])

    def execute(self, userdata):
        client.utils.sync_buffered_reasoner()
        choice = userdata.choice
        current_pose = userdata.current_pose
        client.manipulation.replace_objectprop_b2_ind("isIn", "Robot1", choice, current_pose)
        last_visit = client.query.dataprop_b2_ind("visitedAt", choice)
        last_visit = last_visit[0][:11]
        last_visit = last_visit[1:]
        current_time = str(int(time.time()))
        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.controller_client.cancel_goals()
                    return TRANS_RECHARGING
                # If the controller finishes its computation, then take the `went_random_pose` transition, which is related to the `repeat` transition.
                if self._helper.controller_client.is_done():
                    client.manipulation.replace_dataprop_b2_ind("visitedAt", choice, "Long", current_time, last_visit)
                    client.utils.apply_buffered_changes()
                    client.utils.sync_buffered_reasoner()
                    last_change = client.query.dataprop_b2_ind("now", "Robot1")
                    last_change = last_change[0][:11]
                    last_change = last_change[1:]
                    print("last change was")
                    print(last_change)
                    client.manipulation.replace_dataprop_b2_ind("now", "Robot1", "Long", current_time, last_change)
                    client.utils.sync_buffered_reasoner()
                    print("Now the robot is in " + choice)
                    print("Last visit for " + choice + " was " + last_visit + " new visit at " + current_time)
                    return TRANS_MOVED
                    # Note that if unexpected stimulus comes from the other nodes of the architecture through the
                    # `self._helper` class, then this state will not take any transitions. This is equivalent to have a
                    # loop-like transition in the behavioural UML diagram for all the other stimulus except
                    # `TRANS_RECHARGING`, `TRANS_START_INTERACTION` and `TRANS_WENT_RANDOM_POSE`.
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)

class Recharging(State):
    # Construct this class, i.e., initialise this state.
    def __init__(self, interface_helper):
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        # Initialise this state with possible transitions (i.e., valid outputs of the `execute` function).
        State.__init__(self, outcomes=[TRANS_RECHARGED])

    # Define the function performed each time a transition is such to enter in this state.
    # Note that the input parameter `userdata` is not used since no data is required from the other states.
    def execute(self, userdata):
        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is no low anymore take the `charged` transition.
                if not self._helper.is_battery_low():
                    self._helper.reset_states()  # Reset the state variable related to the stimulus.
                    return TRANS_RECHARGED
                # Note that if unexpected stimulus comes from the other nodes of the architecture through the
                # `self._helper` class, then this state will not take any transitions. This is equivalent to have a
                # loop-like transition in the behavioural UML diagram for all the other stimulus except `TRAMS_RECHARGED`.
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)

"""
class Recharging(smach.State):
    def __init__(self):
        State.__init__(self, outcomes=[TRANS_RECHARGED])

    def execute(self, userdata):
        rospy.sleep(5)
        print("Return to decide state")
        return TRANS_RECHARGED
"""

def main():
    rospy.init_node('state_machine', log_level = rospy.INFO)

    # Initialise an helper class to manage the interfaces with the other nodes in the architectures, i.e., it manages external stimulus.
    helper = InterfaceHelper()
    sm_main = smach.StateMachine([])
    with sm_main:
        smach.StateMachine.add(STATE_INIT, LoadOntology(helper),
                         transitions = {TRANS_INITIALIZED: STATE_DECISION})
        smach.StateMachine.add(STATE_DECISION, DecideTarget(helper),
                         transitions = {TRANS_DECIDED: STATE_MOVING})
        smach.StateMachine.add(STATE_MOVING, MoveToTarget(helper),
                         transitions = {TRANS_MOVED: STATE_RECHARGING})
        smach.StateMachine.add(STATE_RECHARGING, Recharging(helper),
                         transitions = {TRANS_RECHARGED: STATE_DECISION})
    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm_main, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm_main.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
   
if __name__ == "__main__":
    main()