#!/usr/bin/env python3 

import smach
import rospy
from pydoc import Helper
import random
import smach_ros
from threading import Lock
from std_msgs.msg import Bool
from smach import StateMachine, State
from load_ontology import LoadMap
from Assignment1 import architecture_name_mapper as anm
from interface_helper import InterfaceHelper, BehaviorHelper
from Assignment1.msg import Point, ControlGoal, PlanGoal

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

class LoadOntology(smach.State):
    def __init__(self, interface_helper, behavior_helper):
        State.__init__(self, outcomes = [TRANS_INITIALIZED])

    def execute(self, userdata):
        LoadMap()
        print("MAP ACQUIRED")
        return TRANS_INITIALIZED

class DecideTarget(smach.State):
    def __init__(self, interface_helper, behavior_helper):
        State.__init__(self, outcomes = [TRANS_RECHARGING, TRANS_DECIDED], output_keys = ['current_pose', 'choice', 'random_plan'])
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        self._behavior = behavior_helper
        # Get the environment size from ROS parameters.
        self.environment_size = rospy.get_param(anm.PARAM_ENVIRONMENT_SIZE)

    def execute(self, userdata):
        # Define a random point to be reached through some via-points to be planned.
        goal = PlanGoal()
        goal.target = Point(x = random.uniform(0, self.environment_size[0]),
                            y = random.uniform(0, self.environment_size[1]))
        # Invoke the planner action server.
        self._helper.planner_client.send_goal(goal)
        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.planner_client.cancel_goals()
                    return TRANS_RECHARGING
                # If the controller finishes its computation, then take the `went_random_pose` transition, which is related to the `repeat` transition.
                if self._helper.planner_client.is_done():
                    current_pose, choice = self._behavior.decide_target()
                    userdata.current_pose = current_pose
                    userdata.choice = choice
                    userdata.random_plan = self._helper.planner_client.get_results().via_points
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

class MoveToTarget(smach.State):
    def __init__(self, interface_helper, behavior_helper):
        State.__init__(self, outcomes = [TRANS_RECHARGING, TRANS_MOVED], input_keys = [ "random_plan",'current_pose', 'choice'], output_keys= ['current_location'])
         # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        self._behavior = behavior_helper

    def execute(self, userdata):
        # Get the plan to a random position computed by the `PLAN_TO_RANDOM_POSE` state.
        plan = userdata.random_plan
        # Start the action server for moving the robot through the planned via-points.
        goal = ControlGoal(via_points = plan)
        self._helper.controller_client.send_goal(goal)
        choice = userdata.choice
        current_pose = userdata.current_pose
        userdata.current_location = current_pose

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
                    self._behavior.move_to_target(choice, current_pose)
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
    def __init__(self, interface_helper, behavior_helper):
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        self._behavior = behavior_helper
        # Initialise this state with possible transitions (i.e., valid outputs of the `execute` function).
        State.__init__(self, outcomes = [TRANS_RECHARGED], 
                       input_keys = ['current_location'])

    # Define the function performed each time a transition is such to enter in this state.
    # Note that the input parameter `userdata` is not used since no data is required from the other states.
    def execute(self, userdata):
        current_location = userdata.current_location
        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is no low anymore take the `charged` transition.
                if not self._helper.is_battery_low():
                    self._helper.reset_states()  # Reset the state variable related to the stimulus.
                    self._behavior.go_to_recharge(current_location)
                    return TRANS_RECHARGED
                    # Note that if unexpected stimulus comes from the other nodes of the architecture through the
                    # `self._helper` class, then this state will not take any transitions. This is equivalent to have a
                    # loop-like transition in the behavioural UML diagram for all the other stimulus except `TRAMS_RECHARGED`.
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)

def main():
    rospy.init_node('state_machine', log_level = rospy.INFO)
    # Initialise an helper class to manage the interfaces with the other nodes in the architectures, i.e., it manages external stimulus.
    helper = InterfaceHelper()
    behavior = BehaviorHelper()
    # Get the initial robot pose from ROS parameters.
    robot_pose_param = rospy.get_param(anm.PARAM_INITIAL_POSE, [0, 0])
    # Initialise robot position in the `robot_state`, as required by the plan anc control action servers.
    helper.init_robot_pose(Point(x = robot_pose_param[0], y = robot_pose_param[1]))

    sm_main = smach.StateMachine([])
    with sm_main:

        smach.StateMachine.add(STATE_INIT, LoadOntology(helper, behavior),
                         transitions = {TRANS_INITIALIZED: STATE_DECISION})
        smach.StateMachine.add(STATE_DECISION, DecideTarget(helper, behavior),
                         transitions = {TRANS_RECHARGING : STATE_RECHARGING,
                                        TRANS_DECIDED: STATE_MOVING})
        smach.StateMachine.add(STATE_MOVING, MoveToTarget(helper, behavior),
                         transitions = {TRANS_RECHARGING : STATE_RECHARGING,
                                        TRANS_MOVED: STATE_DECISION})
        smach.StateMachine.add(STATE_RECHARGING, Recharging(helper, behavior),
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