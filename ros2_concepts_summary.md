# ROS2 Core Concepts and Python Implementation

## 1. Node Creation and Lifecycle

### Core Concepts
Nodes are the fundamental building blocks of ROS2 applications. Each node represents a single process that performs computation and can communicate with other nodes. Nodes are combined together in a graph to build complex robotic systems.

Node Lifecycle:
- Nodes can be managed using lifecycle states (inactive, active, shutdown)
- Lifecycle nodes provide more control over node behavior
- States can be activated/deactivated programmatically

### Python Implementation (rclpy)
In rclpy, nodes are created by inheriting from `rclpy.Node` class or by instantiating it directly:

```python
import rclpy
from rclpy.node import Node

# Basic node creation
rclpy.init()
node = Node('my_node')
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()

# Or by inheritance
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        # Node initialization code here

rclpy.init()
node = MyNode()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
```

For lifecycle management:
```python
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn

class MyLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('my_lifecycle_node')
    
    def on_configure(self, state):
        # Configuration code
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state):
        # Activation code
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state):
        # Deactivation code
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state):
        # Cleanup code
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state):
        # Shutdown code
        return TransitionCallbackReturn.SUCCESS
```

## 2. Publisher and Subscriber Patterns

### Core Concepts
ROS2 uses a publish/subscribe model for asynchronous communication:
- **Publishers** send messages to specific topics
- **Subscribers** listen to topics and receive messages
- Communication is many-to-many (multiple publishers/subscribers per topic)
- This pattern enables loose coupling between nodes

### Python Implementation (rclpy)
```python
# Publisher
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

# Subscriber
class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
    
    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
```

## 3. Service and Client Patterns

### Core Concepts
For synchronous request/response communication:
- **Services** provide callable functions that wait for requests
- **Clients** call services and wait for responses
- One service server can handle multiple clients
- Services are for immediate, reliable operations

### Python Implementation (rclpy)
```python
# Service Server
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
    
    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: {request.a} + {request.b} = {response.sum}')
        return response

# Service Client
class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
    
    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.cli.call_async(request)
        return future
```

## 4. Parameter Handling

### Core Concepts
Parameters allow nodes to store and retrieve configuration values:
- Key/value pairs stored on the parameter server
- Accessible via command line tools or programmatically
- Support various data types (int, float, string, bool, arrays)
- Can be declared, set, and retrieved within nodes

### Python Implementation (rclpy)
```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters
        self.declare_parameter('my_parameter', 'default_value')
        self.declare_parameter('my_int_parameter', 42)
        
        # Get parameters
        my_param = self.get_parameter('my_parameter').value
        my_int_param = self.get_parameter('my_int_parameter').value
        
        # Set parameters
        self.set_parameters([rclpy.parameter.Parameter('my_parameter', rclpy.Parameter.Type.STRING, 'new_value')])
        
        # Parameter callback for changes
        self.add_on_set_parameters_callback(self.parameter_callback)
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'my_parameter':
                self.get_logger().info(f'Parameter changed: {param.value}')
        return SetParametersResult(successful=True)
```

## 5. Topic Discovery and Management

### Core Concepts
ROS2 automatically discovers nodes and topics:
- Nodes can discover what topics are available
- Publishers and subscribers match automatically
- Tools like `ros2 topic list` help with introspection
- Dynamic discovery enables plug-and-play functionality

### Python Implementation (rclpy)
```python
class DiscoveryNode(Node):
    def __init__(self):
        super().__init__('discovery_node')
        
        # Get list of topics
        topic_names_and_types = self.get_topic_names_and_types()
        for name, types in topic_names_and_types:
            self.get_logger().info(f'Topic: {name}, Types: {types}')
        
        # Get publishers for a topic
        publishers_info = self.get_publishers_info_by_topic('topic_name')
        for info in publishers_info:
            self.get_logger().info(f'Publisher: {info.node_name}')
        
        # Get subscribers for a topic
        subscribers_info = self.get_subscriptions_info_by_topic('topic_name')
        for info in subscribers_info:
            self.get_logger().info(f'Subscriber: {info.node_name}')
```

## 6. Quality of Service (QoS) Profiles

### Core Concepts
QoS profiles define communication policies:
- Control reliability, durability, and delivery guarantees
- Common profiles include:
  - **Reliable**: Guaranteed delivery (TCP-like)
  - **Best Effort**: No delivery guarantees (UDP-like)
  - **Transient Local**: Replays last message for late subscribers
  - **Volatile**: No message replay
- Critical for handling lossy networks and real-time requirements

### Python Implementation (rclpy)
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Custom QoS profile
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

# Using QoS profile with publisher
publisher = self.create_publisher(String, 'topic_name', qos_profile)

# Using QoS profile with subscriber
subscription = self.create_subscription(String, 'topic_name', callback, qos_profile)
```

## 7. ROS2 Bridging Capabilities with Native ROS

### Main Approaches

#### 1. ROS1 Bridge (ros1_bridge)
- Official bridge package provided by Open Robotics
- Enables message passing between ROS1 and ROS2 nodes
- Works by translating messages between the two systems
- Requires both ROS1 and ROS2 to be installed simultaneously
- Supports:
  - Topic publishing/subscribing
  - Service calls
  - Action servers/clients
  - Parameter synchronization

#### 2. Dual Build Support
- Nodes can be built for both ROS1 and ROS2
- Shared codebase with version-specific interfaces
- Requires maintaining separate package.xml and CMakeLists.txt configurations
- Allows gradual migration from ROS1 to ROS2

#### 3. Gateway Approach
- Using rosbridge_suite to expose ROS1 topics/services over WebSocket
- ROS2 clients can connect via rosbridge_websocket
- Useful for web-based applications and remote monitoring

### Limitations

#### Technical Constraints
- **Performance overhead**: Message translation adds latency
- **Complex deployment**: Requires running both ROS1 and ROS2 environments
- **Limited data types**: Not all message types translate perfectly
- **Synchronization issues**: Clock drift and timing differences between systems

#### Compatibility Issues
- ROS1 uses XMLRPC for master discovery, ROS2 uses DDS
- Different parameter systems (ROS1 centralized vs ROS2 distributed)
- Quality of Service (QoS) policies in ROS2 have no ROS1 equivalent
- Launch file formats differ significantly (XML vs Python/YAML)

#### Maintenance Burden
- Bridge requires updates when either ROS version changes
- Debugging becomes more complex across systems
- Security considerations double with two middleware layers

The bridge works best for transitional projects where legacy ROS1 components need to communicate with new ROS2 systems.