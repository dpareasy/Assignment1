#! /usr/bin/env python3
"""
.. module:: planner
  :platform: Unix
  :synopsis: Python module for implementing the moving behavior
  
.. moduleauthor:: Davide Leo Parisi <davide.parisi1084@gmail.com>

ROS node for implementing the reasoning behavior of the robot.

Service:
    /state/set_pose: Set the robot current pose in the simulation
    /motion/controller: Control of the robot motion
"""
import os
import random
import rospy
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from Assignment1.msg import ControlFeedback, ControlResult
from Assignment1.srv import SetPose
import Assignment1  # This is required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.

class ControllingAction(object):
    """
    An action server to simulate motion controlling.
    Given a plan as a set of via points, it simulate the movements
    to reach each point with a random delay. This server updates
    the current robot position stored in the `robot-state` node.
    """
    def __init__(self):
        # Get random-based parameters used by this server
        self._random_motion_time = rospy.get_param('test/random_motion_time', [0.1, 2.0])
        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer('motion/controller',
                                      Assignment1.msg.ControlAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._as.start()

    def execute_callback(self, goal):
        """
        The callback invoked when a client set a goal to the `controller` server.
        This function requires a list of via points (i.e., the plan), and it simulate
        a movement through each point with a delay spanning in 
        ['self._random_motion_time[0]`, `self._random_motion_time[1]`).
        As soon as each via point is reached, the related robot position is updated
        in the `robot-state` node.

        Args: 
        goal: the goal position
        """
        # Check if the provided plan is processable. If not, this service will be aborted.
        if goal is None or goal.via_points is None or len(goal.via_points) == 0:
            print('No via points provided! This service will be aborted!')
            self._as.set_aborted()
            return

        # Construct the feedback and loop for each via point.
        feedback = ControlFeedback()
        """
        ControlFeedback: Feedback coming from the control server
        """
        os.system('clear')
        for point in goal.via_points:
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                print('Server has been cancelled by the client!')
                # Actually cancel this service.
                self._as.set_preempted()
                return
                # Wait before to reach the following via point. This is just for testing purposes.
            delay = random.uniform(self._random_motion_time[0], self._random_motion_time[1])
            rospy.sleep(delay)
            # Publish a feedback to the client to simulate that a via point has been reached. 
            feedback.reached_point = point
            self._as.publish_feedback(feedback)
            # Set the new current position into the `robot-state` node.
            _set_pose_client(point)
            # Log current robot position.
            print("reaching point ", [point.x,point.y])


        # Publish the results to the client.
        result = ControlResult()
        """
        ControlResult: Result of the computation of the control server
        """
        result.reached_point = feedback.reached_point
        self._as.set_succeeded(result)
        return  # Succeeded.

def _set_pose_client(pose):
    """
    Update the current robot `pose` stored in the `robot-state` node.
    This method is performed for each point provided in the action's server feedback.

    Args:
    pose: point (x and y coordinates)
    """
    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service('state/set_pose')
    try:
        # Call the service and set the current robot position.
        service = rospy.ServiceProxy('state/set_pose', SetPose)
        service(pose)  # The `response` is not used.
    except rospy.ServiceException as e:
        print("Server cannot set current robot position")

if __name__ == '__main__':
    # Initialise the node, its action server, and wait.   
    rospy.init_node('controller', log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()