#!/usr/bin/env python3
"""
.. module:: robot_state
   :platform: Unix
   :synopsis: Python module for implementing the robot state
.. moduleauthor:: Luca Buoncompagni, Davide Leo Parisi davide.parisi1084@gmail.com

ROS node for implementing the robot state. It includes the current position and battery level.

Publishes to: 
    /state/battery_low the boolean stating if the battery is low or not

Service:
    /state/get_pose: Get the robot current pose in the simulation

    /state/set_pose: Set the robot current pose in the simulation
"""

import threading
import rospy
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
        # Define services.
        rospy.Service('state/get_pose', GetPose, self.get_pose)
        rospy.Service('state/set_pose', SetPose, self.set_pose)
        # Start publisher on a separate thread.
        th = threading.Thread(target = self.A_is_battery_low)
        th.start()
    
    def set_pose(self, request):
        """
        The `robot/set_pose` service implementation.
        The `request` input parameter is the current robot pose to be set,
        as given by the client. This server returns an empty `response`.

        Args:
            request: 

        Returns:
            empty (SetPoseResponse): an empty response

        """
        if request.pose is not None:
            # Store the new current robot position.
            self._pose = request.pose
            print("Set current robot position")
        else:
            print("Cannot set an unspecified robot position")
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
            response(GetPoseResponse): the position of the robot

        """
        # Log information.
        if self._pose is None:
            print("Cannot get an unspecified robot position")
        else:
            print("Get current robot position")
        # Create the response with the robot pose and return it.
        response = GetPoseResponse()
        """
        GetPoseResponse(): robot position
        """
        response.pose = self._pose
        return response

    def A_is_battery_low(self):
        """
        Publish changes of battery levels. This method runs on a separate thread.
        """
        # Define a `lathed` publisher to wait for initialisation and publish immediately.
        publisher = rospy.Publisher('state/battery_low', Bool, queue_size = 1, latch = True)
        # Publish battery level changes randomly.
        self.A_random_battery_notifier(publisher)

    def A_random_battery_notifier(self, publisher):
        """
        Publish when the battery change state (i.e., high/low) based on a random
        delay within the interval [`self._random_battery_time[0]`, `self._random_battery_time[1]`).
        The message is published through the `publisher` input parameter and is a
        boolean value, i.e., `True`: battery low, `False`: battery high.

        Args:
            publisher(bool): boolean value stating the battery status

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
            delay = 60
            rospy.sleep(delay)
            # Change battery state.
            self._battery_low = not self._battery_low

if __name__ == "__main__":
    # Instantiate the node manager class and wait.
    RobotState()
    rospy.spin()

