"""
Example client node for the Python ROS Engine.
Demonstrates creating a node that calls a service.
"""

import time

from pyros2 import Node
from config.hydra_config import load_config


class AddTwoInts:
    """Service definition for adding two integers (must match service)."""
    
    class Request:
        def __init__(self, a=0, b=0):
            self.a = a
            self.b = b
            
    class Response:
        def __init__(self, sum=0):
            self.sum = sum


class ClientNode(Node):
    """Example client node that calls a service to add two integers."""

    def __init__(self, config_path: str = None):
        """Initialize the client node with configuration."""
        if config_path:
            config = load_config(config_path)
            node_name = config.node.name
        else:
            node_name = "client_node"
            config = None
            
        super().__init__(node_name)
        
        # Create client
        service_name = config.service.name if config else "/add_two_ints"
        self.client = self.create_client(AddTwoInts, service_name)
        
        # Wait for service to be available
        while not self.client.service_is_ready():
            print("Waiting for service...")
            time.sleep(1)
            
        print("Service is ready!")

    def send_request(self, a, b):
        """Send a request to the service."""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        response = self.client.call(request)
        self.get_logger().info(f"Service response: {a} + {b} = {response.sum}")
        return response

    def get_logger(self):
        """Simple logger for demonstration."""
        class Logger:
            def info(self, message):
                print(f"[INFO] {message}")
        return Logger()


def main():
    """Main function to run the client node."""
    node = ClientNode()
    
    try:
        # Send a few example requests
        print("Sending service requests...")
        node.send_request(3, 4)
        node.send_request(10, 20)
        node.send_request(100, 200)
        
        print("Client node finished. Press Ctrl+C to stop.")
        node.spin()  # Keep node alive
    except KeyboardInterrupt:
        print("Shutting down client node...")
        node.destroy_node()


if __name__ == "__main__":
    main()