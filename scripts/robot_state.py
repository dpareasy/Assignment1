#!/usr/bin/env python3

import threading
import random
import rospy
# Import constant name defined to structure the architecture.
from Assignment1 import architecture_name_mapper as anm
# Import the messages used by services and publishers.
from std_msgs.msg import Bool
from Assignment1.srv import GetPose, GetPoseResponse, SetPose, SetPoseResponse


class RobotState:
    """
    The node manager class.
    This class defines two services to get and set the current 
    robot pose, and a publisher to notify that the battery is low.
    """

    def __init__(self):
        # Initialise this node.
        rospy.init_node('robot-state', log_level = rospy.INFO)
        # Initialise robot position.
        self._pose = None
        # Initialise battery level.
        self._battery_low = False
        # Initialise randomness, if enabled.
        self._randomness = rospy.get_param('test/random_sense/active', True)
        if self._randomness:
            self._random_battery_time = rospy.get_param(anm.PARAM_BATTERY_TIME, [15.0, 40.0])
        # Define services.
        rospy.Service('state/get_pose', GetPose, self.get_pose)
        rospy.Service('state/set_pose', SetPose, self.set_pose)
        # Start publisher on a separate thread.
        th = threading.Thread(target = self._is_battery_low)
        th.start()
    
    def set_pose(self, request):
        """
        The `robot/set_pose` service implementation.
        The `request` input parameter is the current robot pose to be set,
        as given by the client. This server returns an empty `response`.

        Args:
        request: 

        Returns:
        SetPoseResponse(): an empty response
        """
        if request.pose is not None:
            # Store the new current robot position.
            self._pose = request.pose
            print("Set current robot position")
            # Log information.
            #self._print_info(f'Set current robot position through `{anm.SERVER_SET_POSE}` '
                             #f'as ({self._pose.x}, {self._pose.y}).')
        else:
            print("Cannot set an unspecified robot position")
            #rospy.logerr(anm.tag_log('Cannot set an unspecified robot position', LOG_TAG))
        # Return an empty response.
        return SetPoseResponse()

    def get_pose(self, response):
        """
        The `robot/get_pose` service implementation.
        The `request` input parameter is given by the client as empty. Thus, it is not used.
        The `response` returned to the client contains the current robot pose.

        Args:
        response: the response of the server

        Returns:
        response: the position of the robot
        """
        # Log information.
        if self._pose is None:
            print("Cannot get an unspecified robot position")
            #rospy.logerr(anm.tag_log('Cannot get an unspecified robot position', LOG_TAG))
        else:
            print("Get current robot position")
        # Create the response with the robot pose and return it.
        response = GetPoseResponse()
        response.pose = self._pose
        return response

    def _is_battery_low(self):
        """
        Publish changes of battery levels. This method runs on a separate thread.
        """
        # Define a `lathed` publisher to wait for initialisation and publish immediately.
        publisher = rospy.Publisher(anm.TOPIC_BATTERY_LOW, Bool, queue_size = 1, latch = True)
        # Publish battery level changes randomly.
        self._random_battery_notifier(publisher)

    def _random_battery_notifier(self, publisher):
        """
        Publish when the battery change state (i.e., high/low) based on a random
        delay within the interval [`self._random_battery_time[0]`, `self._random_battery_time[1]`).
        The message is published through the `publisher` input parameter and is a
        boolean value, i.e., `True`: battery low, `False`: battery high.

        Args:
        publisher: ......
        """
        delay = 0  # Initialised to 0 just for logging purposes.
        while not rospy.is_shutdown():
            # Publish battery level.
            publisher.publish(Bool(self._battery_low))
            # Log state.
            if self._battery_low:
                print("Robot got low battery after" , delay , "seconds")
            else:
                print("Robot got fully charged battery after" , delay , "seconds")
            # Wait for simulate battery usage.
            delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1])
            #delay = 10
            rospy.sleep(delay)
            # Change battery state.
            self._battery_low = not self._battery_low

if __name__ == "__main__":
    # Instantiate the node manager class and wait.
    RobotState()
    rospy.spin()

