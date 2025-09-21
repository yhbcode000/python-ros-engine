"""
Example service node for the Python ROS Engine.
Demonstrates creating a node that provides a service.
"""

from pyros2 import Node
from config.hydra_config import load_config


class AddTwoInts:
    """Service definition for adding two integers."""
    
    class Request:
        def __init__(self, a=0, b=0):
            self.a = a
            self.b = b
            
    class Response:
        def __init__(self, sum=0):
            self.sum = sum


class ServiceNode(Node):
    """Example service node that provides a service for adding two integers."""

    def __init__(self, config_path: str = None):
        """Initialize the service node with configuration."""
        if config_path:
            config = load_config(config_path)
            node_name = config.node.name
        else:
            node_name = "service_node"
            config = None
            
        super().__init__(node_name)
        
        # Create service
        service_name = config.service.name if config else "/add_two_ints"
        self.service = self.create_service(AddTwoInts, service_name, self.add_two_ints_callback)

    def add_two_ints_callback(self, request):
        """Service callback function that adds two integers."""
        response = AddTwoInts.Response()
        response.sum = request.a + request.b
        self.get_logger().info(f"Service request: {request.a} + {request.b} = {response.sum}")
        return response

    def get_logger(self):
        """Simple logger for demonstration."""
        class Logger:
            def info(self, message):
                print(f"[INFO] {message}")
        return Logger()


def main():
    """Main function to run the service node."""
    node = ServiceNode()
    
    try:
        print("Service node running... Press Ctrl+C to stop.")
        node.spin()
    except KeyboardInterrupt:
        print("Shutting down service node...")
        node.destroy_node()


if __name__ == "__main__":
    main()