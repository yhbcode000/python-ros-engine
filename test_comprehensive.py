import time

from pyros2 import Node
from pyros2.message import String

# Mock service type for addition


class AddTwoIntsService:
    class Request:
        def __init__(self, a=0, b=0):
            self.a = a
            self.b = b

    class Response:
        def __init__(self, sum=0):
            self.sum = sum


class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")
        # Create publisher and subscriber
        self.publisher = self.create_publisher(String, "/test_topic")
        self.subscription = self.create_subscription(
            String, "/test_topic", self.message_callback
        )

        # Create service and client
        self.service = self.create_service(
            AddTwoIntsService, "/add_two_ints", self.add_two_ints_callback
        )
        self.client = self.create_client(AddTwoIntsService, "/add_two_ints")

        self.received_messages = []

    def message_callback(self, msg):
        self.received_messages.append(msg.data)
        print(f"Received: {msg.data}")

    def publish_test_message(self, data):
        msg = String()
        msg.data = data
        self.publisher.publish(msg)
        print(f"Published: {msg.data}")

    def add_two_ints_callback(self, request):
        response = AddTwoIntsService.Response()
        response.sum = request.a + request.b
        print(f"Service adding {request.a} + {request.b} = {response.sum}")
        return response

    def test_service_call(self, a, b):
        request = AddTwoIntsService.Request(a, b)
        try:
            response = self.client.call(request)
            print(f"Client call result: {a} + {b} = {response.sum}")
            return response.sum
        except Exception as e:
            print(f"Service call failed: {e}")
            return None


def main():
    node = TestNode()

    # Test publisher/subscriber
    print("=== Testing Publisher/Subscriber ===")
    for i in range(3):
        node.publish_test_message(f"Test message {i}")
        time.sleep(0.1)

    # Test service/client
    print("\n=== Testing Service/Client ===")
    result = node.test_service_call(3, 4)

    # Check results
    print(f"\nReceived {len(node.received_messages)} messages")
    if len(node.received_messages) == 3:
        print("SUCCESS: Publisher-Subscriber communication working correctly")
    else:
        print("FAILURE: Publisher-Subscriber communication not working correctly")

    if result == 7:
        print("SUCCESS: Service-Client communication working correctly")
    else:
        print("FAILURE: Service-Client communication not working correctly")

    node.destroy_node()


if __name__ == "__main__":
    main()
