"""Example launch file for the Python ROS engine."""

import os
import sys

from publisher_example import PublisherNode
from subscriber_example import SubscriberNode

from pyros2 import LaunchDescription

# Add the examples directory to the path so we can import the example nodes
examples_dir = os.path.join(os.path.dirname(__file__), "..", "..", "examples")
sys.path.insert(0, examples_dir)


def generate_launch_description():
    """
    Generate a launch description with a publisher and subscriber node.

    Returns:
        LaunchDescription: The launch description
    """
    launch_description = LaunchDescription()

    # Add publisher node
    launch_description.add_node(PublisherNode)

    # Add subscriber node
    launch_description.add_node(SubscriberNode)

    return launch_description


# Alternative way to define a launch description
launch_description = LaunchDescription()

# Add publisher node
launch_description.add_node(PublisherNode)

# Add subscriber node
launch_description.add_node(SubscriberNode)
