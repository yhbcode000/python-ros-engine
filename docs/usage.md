# Usage Guide

## Installation

Install the package using pip:

```bash
pip install python-ros-engine
```

Or install in development mode:

```bash
pip install -e .
```

## Basic Usage

### Creating a Node

```python
from pyros2 import Node

node = Node("my_node")
# Add publishers, subscribers, services, etc.
node.spin()
```

### Publisher

```python
from pyros2 import Node, Publisher
from pyros2.message import String

class MyNode(Node):
    def __init__(self):
        super().__init__("publisher_node")
        self.publisher = self.create_publisher(String, "/my_topic")
        self.timer = self.create_timer(1.0, self.publish_message)
    
    def publish_message(self):
        msg = String()
        msg.data = "Hello World!"
        self.publisher.publish(msg)

node = MyNode()
node.spin()
```

### Subscriber

```python
from pyros2 import Node, Subscriber
from pyros2.message import String

class MyNode(Node):
    def __init__(self):
        super().__init__("subscriber_node")
        self.subscription = self.create_subscription(
            String, "/my_topic", self.message_callback)
    
    def message_callback(self, msg):
        print(f"Received: {msg.data}")

node = MyNode()
node.spin()
```

### Service

```python
from pyros2 import Node, Service

class AddTwoInts:
    class Request:
        def __init__(self, a=0, b=0):
            self.a = a
            self.b = b
            
    class Response:
        def __init__(self, sum=0):
            self.sum = sum

class MyNode(Node):
    def __init__(self):
        super().__init__("service_node")
        self.service = self.create_service(
            AddTwoInts, "/add_two_ints", self.add_two_ints_callback)
    
    def add_two_ints_callback(self, request):
        response = AddTwoInts.Response()
        response.sum = request.a + request.b
        return response

node = MyNode()
node.spin()
```

### Client

```python
from pyros2 import Node, Client

class AddTwoInts:
    class Request:
        def __init__(self, a=0, b=0):
            self.a = a
            self.b = b
            
    class Response:
        def __init__(self, sum=0):
            self.sum = sum

class MyNode(Node):
    def __init__(self):
        super().__init__("client_node")
        self.client = self.create_client(AddTwoInts, "/add_two_ints")
    
    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        response = self.client.call(request)
        return response

node = MyNode()
# Send requests as needed
result = node.send_request(3, 4)
print(f"Result: {result.sum}")
```

## Configuration with Hydra

Create a config.yaml file:

```yaml
node:
  name: my_node
  namespace: /

publisher:
  topic: my_topic
  qos:
    reliability: reliable
    durability: volatile
    depth: 10

subscriber:
  topic: my_topic
  qos:
    reliability: reliable
    durability: volatile
    depth: 10
```

Then use it in your code:

```python
from pyros2 import Node
from config.hydra_config import load_config

config = load_config("config.yaml")
node = Node(config.node.name)
```

## Running Examples

To run the examples:

```bash
python examples/publisher_example.py
python examples/subscriber_example.py
python examples/service_example.py
python examples/client_example.py
python examples/bridge_example.py
```

Note: For the bridge example to work, you need to have a ROS master running on localhost:11311.