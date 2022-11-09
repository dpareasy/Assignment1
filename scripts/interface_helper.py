#!/usr/bin/env python3

# Import ROS libraries.
import rospy
import random
from actionlib import SimpleActionClient

# Import mutex to manage synchronization among ROS-based threads (i.e., node loop and subscribers)
from threading import Lock
from armor_client import ArmorClient

# Import constant names that define the architecture's structure.
from Assignment1 import architecture_name_mapper as anm

# Import ROS-based messages.
from std_msgs.msg import Bool
from Assignment1.msg import PlanAction, ControlAction
from Assignment1.srv import SetPose

client = ArmorClient("assignment", "my_ontology")

# A class to simplify the implementation of a client for ROS action servers. It is used by the `InterfaceHelper` class.
class ActionClientHelper:
    # Class constructor, i.e., class initializer. Input parameters are:
    #  - `service_name`: it is the name of the server that will be invoked by this client.
    #  - `action_type`: it is the message type that the server will exchange.
    #  - `done_callback`: it is the name of the function called when the action server completed its computation. If
    #     this parameter is not set (i.e., set to `None`), then only the `self._done_callback` function will be
    #     called when the server completes its computation.
    #  - `feedback_callback`: it is the name of the function called when the action server sends a feedback message. If
    #    this parameter is not set (i.e., set to `None`), then only the `self._feedback_callback` functions will be
    #    called when the server sends a feedback message.
    #  - `mutex`: it is a `Lock` object synchronised with the `done_callback` and `feedback_callback`. If it is not set
    #    (i.e., set to `None`), then a new mutex instance is considered. Set this variable if you want to extends the
    #    synchronization with other classes.
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

    # Start the action server with a new `goal`. Note this call is not blocking (i.e., asynchronous performed).
    def send_goal(self, goal):
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
            print("SENDING GOALS")
        else:
            print("Warning send a new goal, cancel the current request first!")
            #warn_msg = 'Warning send a new goal, cancel the current request first!'
            #rospy.logwarn(anm.tag_log(warn_msg, LOG_TAG))

    # Stop the computation of the action server.
    def cancel_goals(self):
        # The computation can be stopped only if the server is actually computing.
        if self._is_running:
            # Stop the computation.
            self._client.cancel_all_goals()
            # Reset the client's state.
            self.reset_client_states()
        else:
            print("Warning cannot cancel a not running service!")
            #warn_msg = 'Warning cannot cancel a not running service!'
            #rospy.logwarn(anm.tag_log(warn_msg, LOG_TAG))

    # Reset the client state variables stored in this class.
    def reset_client_states(self):
        self._is_running = False
        self._is_done = False
        self._results = None

    # This function is called when the action server send some `feedback` back to the client.
    def _feedback_callback(self, feedback):
        # Acquire the mutex to synchronise the computation concerning the `feedback` message with the other nodes of the architecture.
        self._mutex.acquire()
        try:
            # Eventually, call the method provided by the node that uses this action client to manage a feedback.
            if self._external_feedback_cb is not None:
                self._external_feedback_cb(feedback)
            # Uncomment below to log information.
            # rospy.loginfo(anm.tag_log(f'`{self._service_name}` action server provide feedback: {feedback}.', LOG_TAG))
        finally:
            # Realise the mutex to (eventually) unblock ROS-based thread waiting on the same mutex.
            self._mutex.release()

    # This function is called when the action server finish its computation, i.e., it provides a `done` message.
    def _done_callback(self, status, results):
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
            # Uncomment below to log information.
            # log_msg = f'`{self._service_name}` done with state `{self._client.get_state_txt()}` and result: {results}.'
            # rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        finally:
            self._mutex.release()

    # Get `True` if the action server finished is computation, or `False` otherwise.
    # Note that use this method should do it in a `self._mutex` safe manner.
    def is_done(self):  # they should be mutex safe
        return self._is_done

    # Get `True` if the action server is running, or `False` otherwise.
    # A note that use this method should do it in a `self._mutex` safe manner.
    def is_running(self):
        return self._is_running

    # Get the results of the action server, if any, or `None`.
    def get_results(self):
        if self._is_done:
            return self._results
        else:
            print("Error: cannot result")
            #log_err = f'Error: cannot get result for `{self._service_name}`.'
            #rospy.logerr(anm.tag_log(log_err, LOG_TAG))
            return None


# A class to decouple the implementation of the Finite State Machine to the stimulus might that
# lead to state transitions. This class manages the synchronization with subscribers and action
# servers.
class InterfaceHelper:

    # Class constructor, i.e., class initializer.
    def __init__(self):
        # Create a shared mutex to synchronize action clients and subscribers.
        # Note that, based on different assumptions, further optimization can be done to make the different threads
        # blocking for a less amount of time in the same mutex.
        self.mutex = Lock()
        # Set the initial state involving the `self._battery_low`, `self._start_interaction` and `self._gesture` variables.
        self.reset_states()
        # Define the callback associated with the speech, gesture, and battery low ROS subscribers.
        rospy.Subscriber(anm.TOPIC_BATTERY_LOW, Bool, self._battery_callback)
        # Define the clients for the the plan and control action servers.
        self.planner_client = ActionClientHelper('motion/planner', PlanAction, mutex=self.mutex)
        self.controller_client = ActionClientHelper(anm.ACTION_CONTROLLER, ControlAction, mutex=self.mutex)

    # Reset the stimulus, which are stored as states variable fo this class.
    # This function assumes that no states of the Finite State Machine run concurrently.
    def reset_states(self):
        self._battery_low = False
        self._start_interaction = False
        self._gesture = None


    # The subscriber to get messages published from the `robot-state` node into the `/state/battery_low/` topic.
    def _battery_callback(self, msg):
        # Acquire the mutex to assure the synchronization with the other subscribers and action clients (this assure data consistency).
        self.mutex.acquire()
        try:
            # Get the battery level and set the relative state variable encoded in this class.
            self._battery_low = msg.data
            # Uncomment below to log data.
            # if self._battery_low:
            #     log_msg = 'Robot with low battery.'
            # else:
            #     log_msg = 'Robot battery fully charged.'
            # rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        finally:
            # Release the mutex to eventually unblock the other subscribers or action servers that are waiting.
            self.mutex.release()

    # Get the state variable encoded in this class that concerns the battery level.
    # The returning value will be `True` if the battery is low, `False` otherwise.
    # Note that the node using this class might exploit the `reset_state` function to improve robustness.
    # Also note that this function should be used when the `mutex` has been acquired. This assures the
    # synchronization  with the threads involving the subscribers and action clients.
    def is_battery_low(self):
        return self._battery_low

    # Update the current robot pose stored in the `robot-state` node.
    @staticmethod
    def init_robot_pose(point):
        # Eventually, wait for the server to be initialised.
        rospy.wait_for_service(anm.SERVER_SET_POSE)
        try:
            # Call the service and set the current robot position.
            service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
            service(point)  # None that the service `response` is not used.
            print("Setting initial robot position")
            #log_msg = f'Setting initial robot position ({point.x}, {point.y}) to the `{anm.SERVER_SET_POSE}` node.'
            #rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        except rospy.ServiceException as e:
            print("Cannot set current robot position")
            #err_msg = f'Cannot set current robot position through `{anm.SERVER_SET_POSE}` server. Error: {e}'
            #rospy.logerr(anm.tag_log(err_msg, LOG_TAG))

class BehaviorHelper:

    def __init__(self):

        self.decide_target()

    def decide_target(self):
        current_pose = client.query.objectprop_b2_ind("isIn","Robot1")
        current_pose = current_pose[0][32:-1]
        current_pose = current_pose
        print("\ncurrent pose is " + current_pose)
        destination = client.query.objectprop_b2_ind("canReach", "Robot1")
        print("Possible destinations")
        print(destination)
        if len(destination) == 1:
            choice = destination[0]
        else:
            choice = random.choice(destination)
        choice = choice[32:-1]
        choice = choice
        print(choice)
        return current_pose, choice

    def move_to_target(self, choice, current_pose):
        client.manipulation.replace_objectprop_b2_ind("isIn", "Robot1", choice, current_pose)
        client.utils.apply_buffered_changes()
        client.utils.sync_buffered_reasoner()
        print("Now the robot is in " + choice)

    def go_to_recharge(self, current_location):
        client.manipulation.replace_objectprop_b2_ind("isIn", "Robot1", "E", current_location)
        client.utils.apply_buffered_changes()
        client.utils.sync_buffered_reasoner()
        print("Robot recharged")

