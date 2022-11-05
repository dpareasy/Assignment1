#!/usr/bin/env python3 

import rospy
from actionlib import SimpleActionClient
from threading import Lock
from Assignment1 import architecture_name_mapper as anm
from std_msgs.msg import Bool 
from Assignment1.srv import SetPose
from Assignment1.msg import PlanAction, ControlAction

# a tag for identifiyng logs producer
LOG_TAG = anm.NODE_ROBOT_STATE

class ActionClientHelper:
    def __init__(self, service_name, action_type, done_callback = None, feedback_callback = None, mutex = None):
        self.reset_sates()
        self._service_name = service_name
        if mutex is None:
            self._mutex = Lock()
        else:
            self._mutex = mutex

        self._client = SimpleActionClient(service_name, action_type)
        self._external_done_cb = done_callback
        self._external_feedback_cb = feedback_callback
        self._client.wait_for_server()


    def send_goal(self, goal):#async call 
        if not self._is_running:
            self._client.send_goal(goal, done_cb = self._done_callback, feedback_cb = self._feedback_callback)
            self._is_running = True
            self._is_done = False
            self._results = None
        else:
            print("warning send a new goal, cancelled curent request first")

    def reset_states(self):
        self._is_running = False
        self._is_done = False
        self._results = None

    def cancel_goals(self):
        if (self._is_running):
            self._client.cancel_all_goals()
            self.reset_states()
        else:
            print("warning: cannot cancel a not running service")

    def _feedback_callback(self, feedback):
        self._mutex.acquire()
        try:
            if self._external_feedback_cb is not None:
                print(self._service_name + "action server provide feedback: " + str(feedback))
                self._external_feedback_cb(feedback)
        finally:
            self._mutex.release

    def _done_callback(self, status, results):
        self._mutex.acquire()
        try:
            self._is_running = False
            self._is_done = True
            self._results = results
            if self._external_done_cb is not None:
                print(self._service_name + "done with state " + self._client.get_state_txt()+" and result: " + str(results))
                self._external_done_cb(status, results)
        finally:
            self._mutex.release()

    def is_done(self): # they should be mutex safe
        return self._is_done

    def get_results(self):
        if(self._is_done):
            return self._results

        else:
            print("Error: cannot get result for " + self._service_name + ".")

    def is_running(self):
        return self._is_running

class InterfaceHelper: #Assumes that no state runs concurrently (the mutex is shared among all states)

    def __init__(self):
        self.mutex = Lock()
        self.reset_states()
        
        rospy.Subscriber(anm.TOPIC_BATTERY_LOW, Bool, self._battery_callback)

        self.planner_client = ActionClientHelper(anm.ACTION_PLANNER, PlanAction, mutex = self.mutex)
        self.controller_client = ActionClientHelper(anm.ACTION_CONTROLLER, ControlAction, mutex = self.mutex)

    def reset_states(self):
        self._battery_low = False
    
    def _battery_callback(self, msg):
        self.mutex.acquire()
        try:
            self._battery_low = msg.data
            if self._battery_low:
                print("Robot with low battery")
            else:
                print("Robot battery fully charged")

        finally:
            self.mutex.release()

    def is_battery_low(self):
        return self._battery_low

