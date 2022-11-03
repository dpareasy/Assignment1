#!/usr/bin/env python3 

import roslib
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
STATE_PLAN_TO_RANDOM_POSE = 'PLAN_TO_RANDOM_POSE'
STATE_GO_TO_RANDOM_POSE = 'GO_TO_RANDOM_POSE'
STATE_NORMAL = 'NORMAL'
STATE_RECHARGING = 'RECHARGING'

# list of transition states
TRANS_INITIALIZED = 'everithing_loaded'
TRANS_DECIDED = 'target_acquired'
TRANS_BATTERY_LOW = 'battery_low'
TRANS_PLANNED_TO_RANDOM_POSE = 'planned_to_random_pose'
TRANS_RECHARGING = 'recharging'
TRANS_WENT_RANDOM_POSE = 'went_to_random_pose'
TRANS_RECHARGED = 'recharged'

client = ArmorClient("assignment", "my_ontology")

client.utils.mount_on_ref()
client.utils.set_log_to_terminal(True)

def decide():
    current_pose = client.query.objectprop_b2_ind("isIn","Robot1")
    current_pose = current_pose[0][32:]
    current_pose = current_pose[:len(current_pose) - 1]
    print("\ncurrent pose is " + current_pose)
    destination = client.query.objectprop_b2_ind("canReach", "Robot1")
    print(destination)
    if len(destination) == 1:
        choice = destination[0]
    else:
        choice = random.choice(destination)
    choice = choice[32:]
    choice = choice[:len(choice) - 1]
    print(choice)
    return choice, current_pose

class LoadOntology(State):
    def __init__(self):
        State.__init__(self, outcomes = [TRANS_INITIALIZED])
    def execute(self, userdata):
        LoadMap()
        print("MAP ACQUIRED")
        return TRANS_INITIALIZED

class MoveToTarget(State):
    def __init__(self):
        State.__init__(self, outcomes = [TRANS_DECIDED])

    def execute(self, userdata):
        client.utils.sync_buffered_reasoner()
        choice, current_pose = decide()
        client.manipulation.replace_objectprop_b2_ind("isIn", "Robot1", choice, current_pose)
        last_visit = client.query.dataprop_b2_ind("visitedAt", choice)
        last_visit = last_visit[0][:11]
        last_visit = last_visit[1:]
        current_time = str(int(time.time()))
        client.manipulation.replace_dataprop_b2_ind("visitedAt", choice, "Long", current_time, last_visit)
        print("Last visit for " + choice + " was " + last_visit + " new visit at " + current_time)
        client.utils.apply_buffered_changes()
        client.utils.sync_buffered_reasoner()

        rospy.sleep(5)
        print("target decided")
        return TRANS_DECIDED

class Recharging(State):
    def __init__(self):
        State.__init__(self, outcomes=[TRANS_RECHARGED])

    def execute(self, userdata):
        rospy.sleep(5)
        print("Return to decide state")
        return TRANS_RECHARGED


def main():
    rospy.init_node('state_machine', log_level = rospy.INFO)

    sm_main = StateMachine([])
    with sm_main:
        StateMachine.add(STATE_INIT, LoadOntology(),
                         transitions = {TRANS_INITIALIZED: STATE_DECISION})
        StateMachine.add(STATE_DECISION, MoveToTarget(),
                         transitions = {TRANS_DECIDED: STATE_RECHARGING})
        StateMachine.add(STATE_RECHARGING, Recharging(),
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
     

