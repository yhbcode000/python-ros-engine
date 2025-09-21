"""
Example subscriber node for the Python ROS Engine.
Demonstrates creating a node with a subscriber that listens to messages on a topic.
"""

from config.hydra_config import load_config
from pyros2 import Node
from pyros2.message import String


class SubscriberNode(Node):
    """Example subscriber node that listens to messages on a topic."""

    def __init__(self, config_path: str = None):
        """Initialize the subscriber node with configuration."""
        if config_path:
            config = load_config(config_path)
            node_name = config.node.name
        else:
            node_name = "subscriber_node"
            config = None

        super().__init__(node_name)

        # Create subscription
        topic = config.subscriber.topic if config else "/example_topic"
        self.subscription = self.create_subscription(
            String, topic, self.message_callback
        )

    def message_callback(self, msg):
        """Callback function for received messages."""
        self.get_logger().info(f"Received: {msg.data}")

    def get_logger(self):
        """Simple logger for demonstration."""

        class Logger:
            def info(self, message):
                print(f"[INFO] {message}")

        return Logger()


def main():
    """Main function to run the subscriber node."""
    node = SubscriberNode()

    try:
        print("Subscriber node running... Press Ctrl+C to stop.")
        node.spin()
    except KeyboardInterrupt:
        print("Shutting down subscriber node...")
        node.destroy_node()


if __name__ == "__main__":
    main()
