"""Example service node for the Python ROS engine."""

from pyros2 import Node


class AddTwoIntsService:
    """Mock service type for addition."""

    class Request:
        """Request containing two integers to add."""

        def __init__(self, a=0, b=0):
            """Initialize with two integers."""
            self.a = a
            self.b = b

    class Response:
        """Response containing the sum of two integers."""

        def __init__(self, sum=0):
            """Initialize with sum value."""
            self.sum = sum


class ServiceNode(Node):
    """Example service node."""

    def __init__(self):
        """Initialize the service node."""
        super().__init__("service_node")
        self.service = self.create_service(
            AddTwoIntsService, "/add_two_ints", self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request):
        """Handle service requests for adding two integers."""
        response = AddTwoIntsService.Response()
        response.sum = request.a + request.b
        self.get_logger().info(f"Adding {request.a} + {request.b} = {response.sum}")
        return response

    def get_logger(self):
        """Get a simple logger for demonstration."""

        class Logger:
            def info(self, message):
                print(f"[INFO] {message}")

        return Logger()


def main():
    """Run the service node."""
    node = ServiceNode()

    try:
        print("Service node running... Press Ctrl+C to stop.")
        node.spin()
    except KeyboardInterrupt:
        print("Shutting down service node...")
        node.destroy_node()


if __name__ == "__main__":
    main()
