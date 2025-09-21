"""
Example subscriber node for the Python ROS engine.
"""

import time
from pyros2 import Node
from pyros2.message import String
from pyros2.qos import QoSProfile


class SubscriberNode(Node):
    """Example subscriber node."""
    
    def __init__(self):
        """Initialize the subscriber node."""
        super().__init__("subscriber_node")
        self.subscription = self.create_subscription(
            String, "/example_topic", self.message_callback)
        
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