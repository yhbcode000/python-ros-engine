"""Example showing how to use the launch system programmatically."""

import time

from pyros2 import LaunchDescription, LaunchSystem
from pyros2.message import String


class SimplePublisherNode:
    """A simple publisher node for demonstration."""

    def __init__(self):
        """Initialize the publisher node."""
        from pyros2 import Node

        self.node = Node("simple_publisher")
        self.publisher = self.node.create_publisher(String, "/simple_topic")
        self.counter = 0

    def publish_message(self):
        """Publish a message to the topic."""
        msg = String()
        msg.data = f"Simple message {self.counter}"
        self.publisher.publish(msg)
        self.counter += 1


class SimpleSubscriberNode:
    """A simple subscriber node for demonstration."""

    def __init__(self):
        """Initialize the subscriber node."""
        from pyros2 import Node

        self.node = Node("simple_subscriber")
        self.subscription = self.node.create_subscription(
            String, "/simple_topic", self.message_callback
        )

    def message_callback(self, msg):
        """Handle received messages."""
        print(f"Received message: {msg.data}")


def main():
    """Demonstrate launch system usage."""
    # Method 1: Using LaunchSystem directly
    print("Method 1: Using LaunchSystem directly")
    launch_system = LaunchSystem()

    # Add nodes
    launch_system.add_node("publisher", SimplePublisherNode)
    launch_system.add_node("subscriber", SimpleSubscriberNode)

    # Print system status
    launch_system.print_system_status()

    # Start nodes (in a real implementation, this would run them)
    print("Starting nodes...")
    try:
        # Run for a few seconds
        start_time = time.time()
        while time.time() - start_time < 5:
            # Publish a message every second
            publisher_node = launch_system.nodes["publisher"]
            publisher_node.publish_message()
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        launch_system.shutdown()

    print("\n" + "=" * 50 + "\n")

    # Method 2: Using LaunchDescription
    print("Method 2: Using LaunchDescription")
    launch_description = LaunchDescription()

    # Add nodes
    launch_description.add_node(SimplePublisherNode)
    launch_description.add_node(SimpleSubscriberNode)

    # Execute the launch description
    launch_system = launch_description.execute()

    # Print system status
    launch_system.print_system_status()


if __name__ == "__main__":
    main()
