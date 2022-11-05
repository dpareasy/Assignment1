#!/usr/bin/env python3

import threading
import random
import rospy
# Import constant name defined to structure the architecture.
from arch_skeleton import architecture_name_mapper as anm
# Import the messages used by services and publishers.
from std_msgs.msg import Bool
from Assignment1.srv import GetPose, GetPoseResponse, SetPose, SetPoseResponse


# A tag for identifying logs producer.
LOG_TAG = anm.NODE_ROBOT_STATE


# The node manager class.
# This class defines two services to get and set the current 
# robot pose, and a publisher to notify that the battery is low.
class RobotState:

    def __init__(self):
        # Initialise this node.
        rospy.init_node(anm.NODE_ROBOT_STATE, log_level = rospy.INFO)
        # Initialise battery level.
        self._battery_low = False
        # Start publisher on a separate thread.
        th = threading.Thread(target = self._is_battery_low)
        th.start()

    # Publish changes of battery levels. This method runs on a separate thread.
    def _is_battery_low(self):
        # Define a `lathed` publisher to wait for initialisation and publish immediately.
        publisher = rospy.Publisher(anm.TOPIC_BATTERY_LOW, Bool, queue_size = 1, latch = True)
        # Publish battery level changes randomly.
        self._random_battery_notifier(publisher)

    # Publish when the battery change state (i.e., high/low) based on a random
    # delay within the interval [`self._random_battery_time[0]`, `self._random_battery_time[1]`).
    # The message is published through the `publisher` input parameter and is a
    # boolean value, i.e., `True`: battery low, `False`: battery high.
    def _random_battery_notifier(self, publisher):
        delay = 0  # Initialised to 0 just for logging purposes.
        while not rospy.is_shutdown():
            # Publish battery level.
            publisher.publish(Bool(self._battery_low))
            # Log state.
            if self._battery_low:
                log_msg = f'Robot got low battery after {delay} seconds.'
            else:
                log_msg = f'Robot got a fully charged battery after {delay} seconds.'
            self._print_info(log_msg)
            # Wait for simulate battery usage.
            #delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1])
            delay = 10
            rospy.sleep(delay)
            # Change battery state.
            self._battery_low = not self._battery_low


    # Print logging only when random testing is active.
    # This is done to allow an intuitive usage of the keyboard-based interface.
    def _print_info(self, msg):
        if self._randomness:
            rospy.loginfo(anm.tag_log(msg, LOG_TAG))


if __name__ == "__main__":
    # Instantiate the node manager class and wait.
    RobotState()
    rospy.spin()

