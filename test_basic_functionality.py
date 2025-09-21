"""Basic functionality tests for the Python ROS Engine."""

import time

from pyros2 import Node
from pyros2.message import String

# Test publisher and subscriber in the same process
received_messages = []


class TestNode(Node):
    """Test node for basic functionality."""

    def __init__(self):
        """Initialize the test node."""
        super().__init__("test_node")
        self.publisher = self.create_publisher(String, "/test_topic")
        self.subscription = self.create_subscription(
            String, "/test_topic", self.message_callback
        )

    def message_callback(self, msg):
        """Handle incoming messages."""
        received_messages.append(msg.data)
        print(f"Received: {msg.data}")

    def publish_test_message(self, data):
        """Publish a test message."""
        msg = String()
        msg.data = data
        self.publisher.publish(msg)
        print(f"Published: {msg.data}")


def main():
    node = TestNode()

    # Publish a few messages
    for i in range(3):
        node.publish_test_message(f"Test message {i}")
        time.sleep(0.5)

    # Check if messages were received
    print(f"Received {len(received_messages)} messages")
    if len(received_messages) == 3:
        print("SUCCESS: Publisher-Subscriber communication working correctly")
    else:
        print("FAILURE: Publisher-Subscriber communication not working correctly")

    node.destroy_node()


if __name__ == "__main__":
    main()
