#!/usr/bin/env python3
"""Test script for the launch system."""

import os
import sys

# Add the examples directory to the path
examples_dir = os.path.join(os.path.dirname(__file__), "examples")
sys.path.insert(0, examples_dir)

from publisher_example import PublisherNode
from subscriber_example import SubscriberNode

from pyros2 import LaunchSystem


def test_launch_system():
    """Test the launch system with existing examples."""
    # Create launch system
    launch_system = LaunchSystem()

    # Add nodes
    launch_system.add_node("publisher", PublisherNode)
    launch_system.add_node("subscriber", SubscriberNode)

    # Print system status
    print("System Status:")
    launch_system.print_system_status()

    print("Launch system test completed successfully!")


if __name__ == "__main__":
    test_launch_system()
