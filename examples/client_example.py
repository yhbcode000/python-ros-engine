"""Example client node for the Python ROS engine."""

import time

from pyros2 import Node


class AddTwoIntsService:
    """Service definition for adding two integers."""

    class Request:
        """Request for adding two integers."""

        def __init__(self, a=0, b=0):
            """Initialize the request with two integers."""
            self.a = a
            self.b = b

    class Response:
        """Response containing the sum of two integers."""

        def __init__(self, sum=0):
            """Initialize the response with the sum."""
            self.sum = sum


class ClientNode(Node):
    """Example client node."""

    def __init__(self):
        """Initialize the client node."""
        super().__init__("client_node")
        self.client = self.create_client(AddTwoIntsService, "/add_two_ints")

    def send_request(self, a, b):
        """Send a request to the service."""
        request = AddTwoIntsService.Request()
        request.a = a
        request.b = b

        try:
            response = self.client.call(request)
            self.get_logger().info(f"Result: {a} + {b} = {response.sum}")
            return response
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return None

    def get_logger(self):
        """Get a simple logger for demonstration."""

        class Logger:
            def info(self, message):
                print(f"[INFO] {message}")

            def error(self, message):
                print(f"[ERROR] {message}")

        return Logger()


def main():
    """Run the client example."""
    node = ClientNode()

    try:
        # Send a few requests
        for i in range(5):
            node.send_request(i, i + 1)
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down client node...")
        node.destroy_node()


if __name__ == "__main__":
    main()
