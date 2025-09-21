"""
Example launch system for the Python ROS Engine.

Demonstrates how to launch multiple nodes together as a robot system.
"""

import threading
import time

from nodes.client_node import ClientNode
from nodes.publisher_node import PublisherNode
from nodes.service_node import ServiceNode
from nodes.subscriber_node import SubscriberNode


class RobotSystem:
    """A complete robot system with multiple nodes."""

    def __init__(self):
        """Initialize all nodes in the robot system."""
        print("Initializing robot system...")

        # Create all nodes
        self.publisher_node = PublisherNode()
        self.subscriber_node = SubscriberNode()
        self.service_node = ServiceNode()
        self.client_node = ClientNode()

        # Set up threads for each node
        self.publisher_thread = None
        self.subscriber_thread = None
        self.service_thread = None
        self.client_thread = None

    def start_nodes(self):
        """Start all nodes in separate threads."""
        print("Starting robot system nodes...")

        # Start publisher node
        self.publisher_thread = threading.Thread(target=self._run_publisher)
        self.publisher_thread.daemon = True
        self.publisher_thread.start()

        # Start subscriber node
        self.subscriber_thread = threading.Thread(target=self._run_subscriber)
        self.subscriber_thread.daemon = True
        self.subscriber_thread.start()

        # Start service node
        self.service_thread = threading.Thread(target=self._run_service)
        self.service_thread.daemon = True
        self.service_thread.start()

        # Start client node after a delay to ensure service is ready
        self.client_thread = threading.Thread(target=self._run_client)
        self.client_thread.daemon = True
        self.client_thread.start()

    def _run_publisher(self):
        """Run the publisher node."""
        try:
            self.publisher_node.spin()
        except Exception as e:
            print(f"Publisher node error: {e}")

    def _run_subscriber(self):
        """Run the subscriber node."""
        try:
            self.subscriber_node.spin()
        except Exception as e:
            print(f"Subscriber node error: {e}")

    def _run_service(self):
        """Run the service node."""
        try:
            self.service_node.spin()
        except Exception as e:
            print(f"Service node error: {e}")

    def _run_client(self):
        """Run the client node."""
        try:
            # Send a few example requests
            time.sleep(2)  # Wait for service to be ready
            self.client_node.send_request(5, 7)
            self.client_node.send_request(15, 25)
            self.client_node.send_request(1000, 2000)

            # Keep node alive
            self.client_node.spin()
        except Exception as e:
            print(f"Client node error: {e}")

    def shutdown(self):
        """Shutdown all nodes."""
        print("Shutting down robot system...")
        self.publisher_node.destroy_node()
        self.subscriber_node.destroy_node()
        self.service_node.destroy_node()
        self.client_node.destroy_node()


def main():
    """Run the complete robot system."""
    robot_system = RobotSystem()

    try:
        robot_system.start_nodes()
        print("Robot system is running... Press Ctrl+C to stop.")

        # Keep the main thread alive
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nReceived interrupt signal...")
        robot_system.shutdown()


if __name__ == "__main__":
    main()
