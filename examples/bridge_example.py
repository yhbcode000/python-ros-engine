"""Example bridge usage for the Python ROS engine."""

from pyros2 import Bridge
from pyros2.exceptions import BridgeConnectionError


def main():
    """Run the bridge example."""
    # Create bridge connection
    bridge = Bridge()

    try:
        # Connect to ROS master
        bridge.connect()
        print("Connected to ROS master successfully!")

        # Discover ROS nodes
        nodes = bridge.discover_ros_nodes()
        print(f"Discovered {len(nodes)} ROS nodes:")
        for node in nodes:
            print(f"  - {node}")

        # Discover ROS topics
        topics = bridge.discover_ros_topics()
        print(f"Discovered {len(topics)} ROS topics:")
        for topic in topics:
            print(f"  - {topic['name']} (type: {topic['type']})")

        # Discover ROS services
        services = bridge.discover_ros_services()
        print(f"Discovered {len(services)} ROS services:")
        for service in services:
            print(f"  - {service['name']} (providers: {service['providers']})")

    except BridgeConnectionError as e:
        print(f"Failed to connect to ROS master: {e}")
        print("Make sure ROS master is running on localhost:11311")
    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == "__main__":
    main()
