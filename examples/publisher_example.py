"""
Example publisher node for the Python ROS engine.
"""

import time
from pyros2 import Node
from pyros2.message import String
from pyros2.qos import QoSProfile


class PublisherNode(Node):
    """Example publisher node."""
    
    def __init__(self):
        """Initialize the publisher node."""
        super().__init__("publisher_node")
        self.publisher = self.create_publisher(String, "/example_topic")
        self.counter = 0
        
    def publish_message(self):
        """Publish a message to the topic."""
        msg = String()
        msg.data = f"Hello World! Counter: {self.counter}"
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")
        self.counter += 1
        
    def get_logger(self):
        """Simple logger for demonstration."""
        class Logger:
            def info(self, message):
                print(f"[INFO] {message}")
        return Logger()


def main():
    """Main function to run the publisher node."""
    node = PublisherNode()
    
    try:
        while True:
            node.publish_message()
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down publisher node...")
        node.destroy_node()


if __name__ == "__main__":
    main()