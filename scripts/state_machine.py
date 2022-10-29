#!/usr/bin/env python3

import roslib
import rospy 
from pydoc import Helper
import random
import os
import smach_ros
from threading import Lock
from std_msgs.msg import Bool
from smach import StateMachine, State
from Assignment1 import architecture_name_mapper as anm
from interface_helper import InterfaceHelper
from Assignment1.msg import Point, ControlGoal, PlanGoal

#from Assignment1.srv import 


# TODO make global dictionary class with ros topic and node names

#list of states in the machine
STATE_INIT = "INITIALIZE_MAP"
STATE_DECISION = "DECIDE_LOCATION"
STATE_PLAN_TO_RANDOM_POSE = 'PLAN_TO_RANDOM_POSE'
STATE_GO_TO_RANDOM_POSE = 'GO_TO_RANDOM_POSE'
STATE_NORMAL = 'NORMAL'
STATE_RECHARGING = 'RECHARGING'

# list of transition states
TRANS_INITIALIZED = "everithing_loaded"
TRANS_DECIDED = "target_aquired"
TRANS_BATTERY_LOW = 'battery_low'
TRANS_PLANNED_TO_RANDOM_POSE = 'planned_to_random_pose'
TRANS_RECHARGING = 'recharging'
TRANS_WENT_RANDOM_POSE = 'went_to_random_pose'
TRANS_RECHARGED = 'recharged'

class LoadOntology(State):
    def __init__(self):
        State.__init__(self, outcomes = [TRANS_INITIALIZED])

    def execute(self, userdata):
        os.system("roslaunch Assignment1 launch_file.launch")
        print("MAP ACQUIRED")
        return TRANS_INITIALIZED

class DecideTarget(State):
    def __init__(self):
        State.__init__(self, outcomes = TRANS_RECHARGING)

    def execute(self, userdata):
        rospy.sleep(5)
        print("decided the target")
        return TRANS_RECHARGING

class Recharging(State):
    def __init__(self):
        State.__init__(self, outcomes=[TRANS_RECHARGED])

    def execute(self, userdata):
        rospy.sleep(5)
        print("Return to decide state")
        return TRANS_RECHARGED
"""
class Recharging(State):
    def __init__(self, interface_helper):
        self._helper = interface_helper
        State.__init__(self, outcomes=[TRANS_RECHARGED])

    def execute(self, userdata):
        while not rospy.is_shutdown():
            self._helper.mutex.acquire()
            try:
                if not self._helper.is_battery_low():
                    self._helper.reset_states()
                    return TRANS_RECHARGED
            finally:
                self._helper.mutex.release()
            rospy.sleep(0.3)

class PlanToRandomPose(State):
    def __init__(self, interface_helper):
        self._helper = interface_helper
        #may be we need the planimetry from the ontology 
        self.environment_size = rospy.get_param('config/environment_size')
        State.__init__(self, outcomes = [TRANS_PLANNED_TO_RANDOM_POSE, TRANS_RECHARGING], output_keys=['random_plan'])
    def execute(self, userdata):
        goal = PlanGoal()
        goal.target = Point(x = random.uniform(0, self.environment_size[0]), y = random.uniform(0, self.environment_size[1]))
        self._helper.planner_client.send_goal(goal)
        print("plannin to go in a new random position")

        while not rospy.is_shutdown():
            self._helper.mutex.acquire()
            try:
                if self._helper.is_battery_low(): #higher priority
                    self._helper.planner_client.cancel_goals()
                    return TRANS_RECHARGING
                if self._helper.planner_client.is_done():
                    userdata.random_plan = self._helper.planner_client.get_results().via_points
                    return TRANS_PLANNED_TO_RANDOM_POSE
            finally:
                self._helper.mutex.release()
            rospy.sleep(0.3)

class GoToRandomPose(State):
    def __init__(self, interface_helper):
        self._helper = interface_helper
        self.environment_size = rospy.get_param('config/environment_size')
        State.__init__(self, outcomes = [TRANS_WENT_RANDOM_POSE, TRANS_RECHARGING], input_keys=['random_plan'])
    def execute(self, userdata):
        plan = userdata.random_plan
        goal = ControlGoal(via_points = plan)
        self._helper.planner_client.send_goal(goal)
        print("following the plan to reach a random position")

        while not rospy.is_shutdown():
            self._helper.mutex.acquire()
            try:
                if self._helper.is_battery_low(): #higher priority
                    self._helper.planner_client.cancel_goals()
                    return TRANS_RECHARGING
                if self._helper.planner_client.is_done():
                    userdata.random_plan = self._helper.planner_client.get_results().via_points
                    return TRANS_WENT_RANDOM_POSE
            finally:
                self._helper.mutex.release()
            rospy.sleep(0.3)
"""
def main():
    rospy.init_node('state_machine', log_level = rospy.INFO)

    sm_main = StateMachine([])
    with sm_main:
        StateMachine.add(STATE_INIT, LoadOntology(),
                         transitions = {TRANS_INITIALIZED: STATE_DECISION})
        StateMachine.add(STATE_DECISION, DecideTarget(),
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
        
"""        
def main():
    rospy.init_node('state_machine', log_level = rospy.INFO)

    helper = InterfaceHelper()

    robot_pose_param = rospy.get_param(anm.PARAM_INITIAL_POSE, [0,0])
    helper.initialise_robot_pose(Point(x=robot_pose_param[0], y = robot_pose_param[1]))

    sm_main = StateMachine([])
    with sm_main:
        sm_normal = StateMachine(outcome = [TRANS_BATTERY_LOW])
        with sm_normal:
            StateMachine.add(STATE_PLAN_TO_RANDOM_POSE, PlanToRandomPose(helper),
                            transitions = {TRANS_PLANNED_TO_RANDOM_POSE: STATE_GO_TO_RANDOM_POSE,
                                           TRANS_RECHARGING: TRANS_BATTERY_LOW})
            StateMachine.add(STATE_GO_TO_RANDOM_POSE, GoToRandomPose(helper),
                            transitions = {TRANS_RECHARGING: TRANS_BATTERY_LOW})
        StateMachine.add(STATE_NORMAL, sm_normal,
                       transitions = {TRANS_RECHARGING: STATE_RECHARGING})
        StateMachine.add(STATE_RECHARGING, Recharging(helper),
                        transitions = {TRANS_RECHARGED: STATE_NORMAL})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer("sm_Introspection", sm_main, "/SM_ROOT")
    sis.start()

    # Execute state machine
    outcome = sm_main.execute()

    # wait for ctrl-c to stop application
    rospy.spin()
    sis.stop()
    """


if __name__ == "__main__":
    main()

