# Python ROS2 Engine

A pure Python implementation of ROS2 core functionality with bridging capabilities to interact with native ROS nodes.

[![PyPI version](https://badge.fury.io/py/python-ros-engine.svg)](https://badge.fury.io/py/python-ros-engine)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Python Versions](https://img.shields.io/pypi/pyversions/python-ros-engine.svg)](https://pypi.org/project/python-ros-engine/)

## Project Structure

```mermaid
graph TD
    A[Python ROS Engine] --> B[Core Functionality]
    A --> C[Bridging Capabilities]
    B --> B1[Node Management]
    B --> B2[Publishers/Subscribers]
    B --> B3[Services/Clients]
    B --> B4[Parameters]
    B --> B5[Timers]
    C --> C1[ROS1 Bridge]
    C --> C2[Message Translation]
```

## Features

- Node creation and lifecycle management
- Publisher and subscriber patterns with Quality of Service (QoS) profiles
- Service and client communication
- Parameter handling with callbacks
- Topic and service discovery
- Timer functionality
- ROS1 bridging capabilities for node/topic/service discovery
- Message translation between Python ROS engine and native ROS
- Configuration with Hydra best practices

## Features

- Node creation and lifecycle management
- Publisher and subscriber patterns with Quality of Service (QoS) profiles
- Service and client communication
- Parameter handling with callbacks
- Topic and service discovery
- Timer functionality
- ROS1 bridging capabilities for node/topic/service discovery
- Message translation between Python ROS engine and native ROS
- Configuration with Hydra best practices

## Message Types

The Python ROS Engine supports a wide range of message types:

### Primitive Types
- `Bool`: Boolean values
- `String`: String values
- `Int8`, `Int16`, `Int32`, `Int64`: Signed integer values
- `UInt8`, `UInt16`, `UInt32`, `UInt64`: Unsigned integer values
- `Float32`, `Float64`: Floating point values
- `Empty`: Empty messages
- `Time`: Time values
- `Duration`: Duration values

### Multi-dimensional Array Types
- `ByteMultiArray`: Multi-array of bytes
- `Int8MultiArray`, `Int16MultiArray`, `Int32MultiArray`, `Int64MultiArray`: Multi-arrays of integer values
- `UInt8MultiArray`, `UInt16MultiArray`, `UInt32MultiArray`, `UInt64MultiArray`: Multi-arrays of unsigned integer values
- `Float32MultiArray`, `Float64MultiArray`: Multi-arrays of floating point values

## Installation

### From PyPI
```bash
pip install python-ros-engine
```

### From Source
```bash
git clone https://github.com/yhbcode000/python-ros-engine.git
cd python-ros-engine
pip install -e .
```

### Development Installation
For development, you can install with test dependencies:
```bash
pip install -e .[test]
```

For development with documentation building capabilities:
```bash
pip install -e .[dev]
```

## Usage

### Basic Node

```python
from pyros2 import Node

node = Node('my_node')
# Add publishers, subscribers, services, etc.
node.spin()
```

### Publisher Example

```python
from pyros2 import Node
from pyros2.message import String

class MyNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, '/my_topic')
        self.timer = self.create_timer(1.0, self.publish_message)
    
    def publish_message(self):
        msg = String()
        msg.data = 'Hello World!'
        self.publisher.publish(msg)

node = MyNode()
node.spin()
```

### Subscriber Example

```python
from pyros2 import Node
from pyros2.message import String

class MyNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String, '/my_topic', self.message_callback)
    
    def message_callback(self, msg):
        print(f'Received: {msg.data}')

node = MyNode()
node.spin()
```

### Service Example

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
        super().__init__('service_node')
        self.service = self.create_service(
            AddTwoInts, '/add_two_ints', self.add_two_ints_callback)
    
    def add_two_ints_callback(self, request):
        response = AddTwoInts.Response()
        response.sum = request.a + request.b
        return response

node = MyNode()
node.spin()
```

### Client Example

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
        super().__init__('client_node')
        self.client = self.create_client(AddTwoInts, '/add_two_ints')
    
    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        response = self.client.call(request)
        return response

node = MyNode()
# Send requests as needed
result = node.send_request(3, 4)
print(f'Result: {result.sum}')
```

### Configuration with Hydra

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

config = load_config('config.yaml')
node = Node(config.node.name)
```

## Documentation

Comprehensive documentation is available in the `docs/` directory and can be built into a static website using MkDocs.

### Building the Documentation Site

To build the documentation site locally, first install the development dependencies:

```bash
pip install mkdocs mkdocs-material
```

Or if you have the package installed in development mode:

```bash
pip install -e .[dev]
```

Then build the site:

```bash
mkdocs build
```

The static site will be generated in the `site/` directory.

### Serving the Documentation Locally

To serve the documentation site locally for development:

```bash
mkdocs serve
```

This will start a local development server at http://127.0.0.1:8000 that automatically reloads when you make changes to the documentation files.

### Online Documentation

For the online documentation, visit [https://yhbcode000.github.io/python-ros-engine/](https://yhbcode000.github.io/python-ros-engine/)

### GitHub Actions Documentation Deployment

The documentation is automatically built and deployed to GitHub Pages using GitHub Actions:

- On pushes to the main branch
- When a new release is published

No manual deployment steps are required.

## Bridging with Native ROS

The engine provides bridging capabilities to discover and interact with native ROS nodes:

```python
from pyros2 import Bridge

bridge = Bridge()
try:
    native_nodes = bridge.discover_ros_nodes()
    print(f"Discovered {len(native_nodes)} ROS nodes")
except Exception as e:
    print(f"Bridge connection failed: {e}")
```

Architecture diagram:

```mermaid
graph LR
    A[Python ROS Engine] -- Message Translation --> B((ROS Bridge))
    B -- XMLRPC/Socket --> C[ROS Master]
    B -- TCP/UDP --> D[Native ROS Nodes]
    A -- TCP/UDP --> E[Native ROS Nodes]
```

Note: Bridging requires a running ROS master (ROS1) on localhost:11311 by default.

## Examples

See the [examples](./examples) directory for complete working examples:

- [Publisher Example](./examples/publisher_example.py)
- [Subscriber Example](./examples/subscriber_example.py)
- [Service Example](./examples/service_example.py)
- [Client Example](./examples/client_example.py)
- [Bridge Example](./examples/bridge_example.py)

We've also included a [complete example project](./example_project) that demonstrates how to build a robot system with multiple interconnected nodes:

- Publisher node that sends status messages
- Subscriber node that listens to status messages
- Service node that provides calculation services
- Client node that calls calculation services
- Launch system to run all nodes together
- Configuration files using Hydra for flexible parameter management

## Testing

Run tests with pytest:

```bash
python -m pytest tests/ -v
```

To run tests with coverage:

```bash
pip install pytest-cov
python -m pytest tests/ --cov=src/ --cov-report=html
```

## GitHub Actions

The project includes GitHub Actions workflows for:

- Continuous Integration (CI) testing across Python versions 3.8-3.12
- Automatic deployment to PyPI when a release is published
- Automatic documentation deployment to GitHub Pages when a release is published or on main branch updates

To deploy to PyPI, you need to set up a `PYPI_API_TOKEN` secret in your GitHub repository.
To deploy to Test PyPI, you need to set up a `TEST_PYPI_API_TOKEN` secret in your GitHub repository.

## Contributing

We welcome contributions! Please see our [Contributing Guide](CONTRIBUTING.md) for details on how to contribute to this project.

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.