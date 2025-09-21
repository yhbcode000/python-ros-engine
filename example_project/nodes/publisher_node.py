"""
Example publisher node for the Python ROS Engine.
Demonstrates creating a node with a publisher that sends messages to a topic.
"""

import time
from dataclasses import dataclass

from pyros2 import Node
from pyros2.message import String
from config.hydra_config import load_config


@dataclass
class PublisherNodeConfig:
    """Configuration for the publisher node."""
    topic: str = "/example_topic"
    publish_rate: float = 1.0  # seconds


class PublisherNode(Node):
    """Example publisher node that sends messages to a topic."""

    def __init__(self, config_path: str = None):
        """Initialize the publisher node with configuration."""
        if config_path:
            config = load_config(config_path)
            node_name = config.node.name
        else:
            node_name = "publisher_node"
            config = None
            
        super().__init__(node_name)
        
        # Create publisher
        self.publisher = self.create_publisher(String, config.publisher.topic if config else "/example_topic")
        
        # Set up timer for publishing
        publish_rate = 1.0  # Default rate
        if config and hasattr(config, 'publish_rate'):
            publish_rate = config.publish_rate
        self.timer = self.create_timer(publish_rate, self.publish_message)
        
        self.counter = 0

    def publish_message(self):
        """Publish a message to the topic."""
        msg = String()
        msg.data = f"Hello World! Counter: {self.counter}"
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")
        self.counter += 1


def main():
    """Main function to run the publisher node."""
    node = PublisherNode()
    
    try:
        print("Publisher node running... Press Ctrl+C to stop.")
        node.spin()
    except KeyboardInterrupt:
        print("Shutting down publisher node...")
        node.destroy_node()


if __name__ == "__main__":
    main()