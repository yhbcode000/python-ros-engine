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

## Node Lifecycle

The Python ROS Engine follows a node lifecycle similar to ROS2:

<pre class="mermaid">
graph TD
    A[Node Creation] --> B[Configuration]
    B --> C[Activation]
    C --> D[Running]
    D --> E{Shutdown?}
    E -->|Yes| F[Cleanup]
    E -->|No| D
    F --> G[Node Destruction]
</pre>

## Publisher/Subscriber Communication

The publisher/subscriber pattern enables asynchronous one-to-many communication between nodes:

<pre class="mermaid">
graph LR
    A[Publishers] -- Messages --> B[(Topic)]
    B -- Messages --> C[Subscribers]
    B -- Messages --> D[Subscribers]
</pre>

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
from pyros2 import Node
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
from pyros2 import Node
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
from pyros2 import Node

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
from pyros2 import Node
import time

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
        
        # Wait for service to be available
        while not self.client.service_is_ready():
            print("Waiting for service...")
            time.sleep(1)
    
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

## Advanced Usage

### Quality of Service (QoS) Profiles

The Python ROS Engine supports QoS profiles for publishers and subscribers:

<pre class="mermaid">
graph TD
    A[QoS Profiles] --> B[Reliability]
    A --> C[Durability]
    A --> D[Depth]
    B --> B1[Reliable - Guaranteed delivery]
    B --> B2[Best Effort - No delivery guarantees]
    C --> C1[Transient Local - Replays last message]
    C --> C2[Volatile - No message replay]
    D --> D1[Queue Depth - Message history size]
</pre>

```python
from pyros2 import Node, QoSProfile
from pyros2.message import String

class MyNode(Node):
    def __init__(self):
        super().__init__("qos_example_node")
        
        # Create a custom QoS profile
        qos_profile = QoSProfile()
        qos_profile.reliability = "reliable"  # or "best_effort"
        qos_profile.durability = "transient_local"  # or "volatile"
        qos_profile.depth = 10
        
        # Use the QoS profile with publisher/subscriber
        self.publisher = self.create_publisher(String, "/my_topic", qos_profile)
        self.subscription = self.create_subscription(String, "/my_topic", self.callback, qos_profile)
    
    def callback(self, msg):
        print(f"Received: {msg.data}")
```

### Parameter Handling

Nodes can handle parameters with callbacks:

<pre class="mermaid">
graph TD
    A[Parameter Declaration] --> B[Parameter Storage]
    B --> C[Parameter Access]
    C --> D[Parameter Modification]
    D --> E[Callback Trigger]
    E --> F[Execute Callback Function]
</pre>

```python
from pyros2 import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("parameter_node")
        
        # Declare a parameter with a callback
        self.declare_parameter("my_param", "default_value", self.param_callback)
    
    def param_callback(self, param_name, old_value, new_value):
        print(f"Parameter {param_name} changed from {old_value} to {new_value}")
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

service:
  name: add_two_ints
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

## Complete Example Project

We've included a complete example project in the repository that demonstrates how to build a robot system with multiple nodes. The example project includes:

1. Configuration files using Hydra
2. Publisher, subscriber, service, and client nodes
3. A launch system to run all nodes together

You can find the complete example in the `example_project/` directory of the repository.

### Running the Complete Example

To run the complete example project:

1. Clone the repository:
   ```bash
   git clone https://github.com/yhbcode000/python-ros-engine.git
   cd python-ros-engine
   ```

2. Install the package:
   ```bash
   pip install python-ros-engine
   ```

3. Run individual nodes:
   ```bash
   python example_project/nodes/publisher_node.py
   python example_project/nodes/subscriber_node.py
   python example_project/nodes/service_node.py
   python example_project/nodes/client_node.py
   ```

4. Run the complete robot system:
   ```bash
   python example_project/launch/robot_system.py
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