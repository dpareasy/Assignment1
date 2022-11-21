#! /usr/bin/env python3
import os
import random
import rospy
# Import constant name defined to structure the architecture.
from Assignment1 import architecture_name_mapper as anm
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from Assignment1.msg import Point, PlanFeedback, PlanResult
from Assignment1.srv import GetPose
import Assignment1  # This is required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.

# 
class PlaningAction(object):
    """
    An action server to simulate motion planning.
    Given a target position, it retrieve the current robot position from the 
    `robot-state` node, and return a plan as a set of via points.
    """

    def __init__(self):
        # Get random-based parameters used by this server
        self._random_plan_points = rospy.get_param(anm.PARAM_PLANNER_POINTS, [2, 8])
        self._random_plan_time = rospy.get_param(anm.PARAM_PLANNER_TIME, [0.1, 1])
        self._environment_size = rospy.get_param(anm.PARAM_ENVIRONMENT_SIZE)
        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(anm.ACTION_PLANNER, 
                                      Assignment1.msg.PlanAction, 
                                      execute_cb=self.execute_callback, 
                                      auto_start=False)
        self._as.start()
        
    def execute_callback(self, goal):
        """
        
        """
        # Get the input parameters to compute the plan, i.e., the start (or current) and target positions.
        start_point = _get_pose_client()
        target_point = goal.target

        # Check if the start and target positions are correct. If not, this service will be aborted.
        if start_point is None or target_point is None:
            print('Cannot have `None` start point nor target_point. This service will be aborted!.')
            # Close service by returning an `ABORT` state to the client.
            self._as.set_aborted()
            return
        if not(self._is_valid(start_point) and self._is_valid(target_point)):
            # Close service by returning an `ABORT` state to the client.
            self._as.set_aborted()
            return
        
        # Initialise the `feedback` with the starting point of the plan.
        feedback = PlanFeedback()
        feedback.via_points = []
        feedback.via_points.append(start_point)
        # Publish the feedback and wait to simulate computation.
        self._as.publish_feedback(feedback)
        delay = random.uniform(self._random_plan_time[0], self._random_plan_time[1])
        rospy.sleep(delay)

        # Get a random number of via points to be included in the plan.
        number_of_points = random.randint(self._random_plan_points[0], self._random_plan_points[1] + 1)
        
        os.system('clear')
        # Generate the points of the plan.
        for i in range(1, number_of_points):
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                print('Server has been cancelled by the client!')
                # Actually cancel this service.
                self._as.set_preempted()  
                return
            # Generate a new random point of the plan.
            new_point = Point()
            new_point.x = random.uniform(0, self._environment_size[0])
            new_point.y = random.uniform(0, self._environment_size[1])
            feedback.via_points.append(new_point)
            if i < number_of_points - 1:
                # Publish the new random point as feedback to the client.
                self._as.publish_feedback(feedback)
                # Wait to simulate computation.
                delay = random.uniform(self._random_plan_time[0], self._random_plan_time[1])
                rospy.sleep(delay)
            else:
                # Append the target point to the plan as the last point.
                feedback.via_points.append(target_point)

        # Publish the results to the client.
        result = PlanResult()
        result.via_points = feedback.via_points
        self._as.set_succeeded(result)
        for point in result.via_points:
            print('[' + str(point.x) + ',' + str(point.y) + ']')
        
    def _is_valid(self, point):
        return 0.0 <= point.x <= self._environment_size[0] and 0.0 <= point.y <= self._environment_size[1]


# 
def _get_pose_client():
    """
    Retrieve the current robot pose by the `state/get_pose` server of the `robot-state` node.
    This function calls the service to get the pose of the robot in the dummy simulation.

    Returns:
    pose: the robot position
    """
    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service('state/get_pose')
    try:
        # Call the service and get a response with the current robot position.
        service = rospy.ServiceProxy('state/get_pose', GetPose)
        response = service()
        pose = response.pose
        return pose
    except rospy.ServiceException as e:
        print("Server cannot get current robot position")


if __name__ == '__main__':
    # Initialise the node, its action server, and wait.    
    rospy.init_node(anm.NODE_PLANNER, log_level=rospy.INFO)
    server = PlaningAction()
    rospy.spin()