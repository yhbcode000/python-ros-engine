# API Documentation

## Node Class

The `Node` class is the base class for all ROS nodes.

### Constructor
```python
Node(node_name: str, namespace: str = "/")
```

- `node_name`: Name of the node
- `namespace`: Namespace for the node (default: "/")

### Methods

#### create_publisher
```python
create_publisher(msg_type: type, topic_name: str, qos_profile: QoSProfile = None)
```
Create a publisher for a topic.

- `msg_type`: The message type class for this publisher
- `topic_name`: The name of the topic to publish to
- `qos_profile`: Quality of Service profile (optional)

#### create_subscription
```python
create_subscription(msg_type: type, topic_name: str, callback: Callable, qos_profile: QoSProfile = None)
```
Create a subscription to a topic.

- `msg_type`: The message type class for this subscription
- `topic_name`: The name of the topic to subscribe to
- `callback`: Function to call when a message is received
- `qos_profile`: Quality of Service profile (optional)

#### create_service
```python
create_service(srv_type: type, service_name: str, callback: Callable, qos_profile: QoSProfile = None)
```
Create a service server.

- `srv_type`: The service type class
- `service_name`: The name of the service
- `callback`: Function to call when a service request is received
- `qos_profile`: Quality of Service profile (optional)

#### create_client
```python
create_client(srv_type: type, service_name: str, qos_profile: QoSProfile = None)
```
Create a service client.

- `srv_type`: The service type class
- `service_name`: The name of the service to call
- `qos_profile`: Quality of Service profile (optional)

#### create_timer
```python
create_timer(timer_period_sec: float, callback: Callable)
```
Create a timer that calls the callback function periodically.

- `timer_period_sec`: Timer period in seconds
- `callback`: Function to call when timer expires

#### declare_parameter
```python
declare_parameter(name: str, default_value: Any = None, parameter_type: str = None)
```
Declare a parameter with a name and default value.

- `name`: Parameter name
- `default_value`: Default value for the parameter (optional)
- `parameter_type`: Type of the parameter (optional)

#### get_parameter
```python
get_parameter(name: str) -> Parameter
```
Get a parameter by name.

- `name`: Parameter name
- Returns: Parameter object

#### set_parameter
```python
set_parameter(name: str, value: Any) -> bool
```
Set a parameter value.

- `name`: Parameter name
- `value`: New value for the parameter
- Returns: True if successful, False otherwise

#### add_on_set_parameters_callback
```python
add_on_set_parameters_callback(callback: Callable)
```
Add a callback to be called when parameters are set.

- `callback`: Function to call when parameters are set

#### get_topic_names_and_types
```python
get_topic_names_and_types() -> List[tuple]
```
Get list of topics and their types.

- Returns: List of (topic_name, topic_types) tuples

#### get_service_names_and_types
```python
get_service_names_and_types() -> List[tuple]
```
Get list of services and their types.

- Returns: List of (service_name, service_types) tuples

#### get_publishers_info_by_topic
```python
get_publishers_info_by_topic(topic_name: str) -> List[Dict[str, Any]]
```
Get information about publishers for a topic.

- `topic_name`: Name of the topic
- Returns: List of dictionaries with publisher information

#### get_subscriptions_info_by_topic
```python
get_subscriptions_info_by_topic(topic_name: str) -> List[Dict[str, Any]]
```
Get information about subscriptions for a topic.

- `topic_name`: Name of the topic
- Returns: List of dictionaries with subscription information

#### spin
```python
spin()
```
Spin the node to process callbacks.

This function blocks until the node is destroyed.

#### spin_once
```python
spin_once(timeout_sec: float = 0.1)
```
Process one callback and return.

- `timeout_sec`: Time to wait for a callback (default: 0.1)

#### destroy_node
```python
destroy_node()
```
Destroy the node and clean up resources.

## Message Classes

### Message
Base class for all ROS messages.

#### serialize
```python
serialize() -> bytes
```
Serialize the message to bytes.

- Returns: Serialized message as bytes

#### deserialize
```python
deserialize(data: bytes)
```
Deserialize bytes to a message instance.

- `data`: Serialized message data as bytes

### String
String message type.

#### Fields
- `data`: str - String data

### Int8
8-bit signed integer message type.

#### Fields
- `data`: int - Integer data (range: -128 to 127)

### Int16
16-bit signed integer message type.

#### Fields
- `data`: int - Integer data (range: -32,768 to 32,767)

### Int32
32-bit signed integer message type.

#### Fields
- `data`: int - Integer data (range: -2,147,483,648 to 2,147,483,647)

### Int64
64-bit signed integer message type.

#### Fields
- `data`: int - Integer data (range: -9,223,372,036,854,775,808 to 9,223,372,036,854,775,807)

### UInt8
8-bit unsigned integer message type.

#### Fields
- `data`: int - Integer data (range: 0 to 255)

### UInt16
16-bit unsigned integer message type.

#### Fields
- `data`: int - Integer data (range: 0 to 65,535)

### UInt32
32-bit unsigned integer message type.

#### Fields
- `data`: int - Integer data (range: 0 to 4,294,967,295)

### UInt64
64-bit unsigned integer message type.

#### Fields
- `data`: int - Integer data (range: 0 to 18,446,744,073,709,551,615)

### Float32
32-bit floating point message type.

#### Fields
- `data`: float - Floating point data

### Float64
64-bit floating point message type.

#### Fields
- `data`: float - Floating point data

### Bool
Boolean message type.

#### Fields
- `data`: bool - Boolean data (True or False)

### Empty
Empty message type.

#### Fields
- None

### Time
Time message type.

#### Fields
- `secs`: int - Seconds
- `nsecs`: int - Nanoseconds

### Duration
Duration message type.

#### Fields
- `secs`: int - Seconds
- `nsecs`: int - Nanoseconds

### MultiArrayDimension
Dimension description for multi-dimensional arrays.

#### Fields
- `label`: str - Label for the dimension
- `size`: int - Size of the dimension
- `stride`: int - Stride of the dimension

### MultiArrayLayout
Layout description for multi-dimensional arrays.

#### Fields
- `dim`: List[MultiArrayDimension] - Array of dimension properties
- `data_offset`: int - Offset to first data point in the array

### ByteMultiArray
Multi-array of bytes.

#### Fields
- `layout`: MultiArrayLayout - Message layout
- `data`: List[int] - Array of bytes

### Int8MultiArray
Multi-array of Int8 values.

#### Fields
- `layout`: MultiArrayLayout - Message layout
- `data`: List[int] - Array of Int8 values

### Int16MultiArray
Multi-array of Int16 values.

#### Fields
- `layout`: MultiArrayLayout - Message layout
- `data`: List[int] - Array of Int16 values

### Int32MultiArray
Multi-array of Int32 values.

#### Fields
- `layout`: MultiArrayLayout - Message layout
- `data`: List[int] - Array of Int32 values

### Int64MultiArray
Multi-array of Int64 values.

#### Fields
- `layout`: MultiArrayLayout - Message layout
- `data`: List[int] - Array of Int64 values

### UInt8MultiArray
Multi-array of UInt8 values.

#### Fields
- `layout`: MultiArrayLayout - Message layout
- `data`: List[int] - Array of UInt8 values

### UInt16MultiArray
Multi-array of UInt16 values.

#### Fields
- `layout`: MultiArrayLayout - Message layout
- `data`: List[int] - Array of UInt16 values

### UInt32MultiArray
Multi-array of UInt32 values.

#### Fields
- `layout`: MultiArrayLayout - Message layout
- `data`: List[int] - Array of UInt32 values

### UInt64MultiArray
Multi-array of UInt64 values.

#### Fields
- `layout`: MultiArrayLayout - Message layout
- `data`: List[int] - Array of UInt64 values

### Float32MultiArray
Multi-array of Float32 values.

#### Fields
- `layout`: MultiArrayLayout - Message layout
- `data`: List[float] - Array of Float32 values

### Float64MultiArray
Multi-array of Float64 values.

#### Fields
- `layout`: MultiArrayLayout - Message layout
- `data`: List[float] - Array of Float64 values

## QoS Classes

### ReliabilityPolicy
Reliability policy for message delivery.

- `RELIABLE`: Guaranteed delivery
- `BEST_EFFORT`: No guarantee of delivery

### DurabilityPolicy
Durability policy for message persistence.

- `TRANSIENT_LOCAL`: Replays last message for late subscribers
- `VOLATILE`: No message replay

### QoSProfile
Quality of Service profile defining communication policies.

#### Constructor
```python
QoSProfile(reliability: ReliabilityPolicy = ReliabilityPolicy.RELIABLE,
           durability: DurabilityPolicy = DurabilityPolicy.VOLATILE,
           depth: int = 10)
```

#### Fields
- `reliability`: ReliabilityPolicy - Message delivery reliability
- `durability`: DurabilityPolicy - Message persistence policy
- `depth`: int - Queue depth for messages

## Publisher Class

### Constructor
```python
Publisher(node: Node, msg_type: type, topic_name: str, qos_profile: QoSProfile)
```

- `node`: Node - Parent node
- `msg_type`: type - Message type for this publisher
- `topic_name`: str - Topic name to publish to
- `qos_profile`: QoSProfile - Quality of Service profile

### Methods

#### publish
```python
publish(message: Message)
```
Publish a message to the topic.

- `message`: Message - Message to publish

## Subscriber Class

### Constructor
```python
Subscriber(node: Node, msg_type: type, topic_name: str, callback: Callable, qos_profile: QoSProfile)
```

- `node`: Node - Parent node
- `msg_type`: type - Message type for this subscription
- `topic_name`: str - Topic name to subscribe to
- `callback`: Callable - Callback function for received messages
- `qos_profile`: QoSProfile - Quality of Service profile

## Service Class

### Constructor
```python
Service(node: Node, srv_type: type, service_name: str, callback: Callable, qos_profile: QoSProfile)
```

- `node`: Node - Parent node
- `srv_type`: type - Service type
- `service_name`: str - Service name
- `callback`: Callable - Callback function for service requests
- `qos_profile`: QoSProfile - Quality of Service profile

## Client Class

### Constructor
```python
Client(node: Node, srv_type: type, service_name: str, qos_profile: QoSProfile)
```

- `node`: Node - Parent node
- `srv_type`: type - Service type
- `service_name`: str - Service name to call
- `qos_profile`: QoSProfile - Quality of Service profile

### Methods

#### call
```python
call(request)
```
Call a service synchronously.

- `request`: Service request object
- Returns: Service response object

#### call_async
```python
call_async(request)
```
Call a service asynchronously.

- `request`: Service request object
- Returns: Future object for the service response

## Discovery Class

### Methods

#### get_all_topics
```python
get_all_topics() -> List[str]
```
Get all available topics.

- Returns: List of topic names

#### get_all_services
```python
get_all_services() -> List[str]
```
Get all available services.

- Returns: List of service names

#### get_topic_types
```python
get_topic_types(topic_name: str) -> List[str]
```
Get the types of a topic.

- `topic_name`: Name of the topic
- Returns: List of topic types

#### get_service_types
```python
get_service_types(service_name: str) -> List[str]
```
Get the types of a service.

- `service_name`: Name of the service
- Returns: List of service types

#### get_nodes
```python
get_nodes() -> List[str]
```
Get all available nodes.

- Returns: List of node names

## Bridge Class

The bridge class provides functionality to interact with native ROS systems.

### Constructor
```python
Bridge(host: str = "localhost", port: int = 11311)
```

- `host`: str - ROS master host (default: "localhost")
- `port`: int - ROS master port (default: 11311)

### Methods

#### connect
```python
connect()
```
Connect to the ROS master.

Raises BridgeConnectionError if connection fails.

#### discover_ros_nodes
```python
discover_ros_nodes() -> List[str]
```
Discover all native ROS nodes.

- Returns: List of node names

#### discover_ros_topics
```python
discover_ros_topics() -> List[Dict[str, Any]]
```
Discover all native ROS topics.

- Returns: List of dictionaries with topic information

#### discover_ros_services
```python
discover_ros_services() -> List[Dict[str, Any]]
```
Discover all native ROS services.

- Returns: List of dictionaries with service information

## MessageTranslator Class

Handles translation between Python ROS engine messages and native ROS messages.

### Methods

#### pyros_to_ros
```python
pyros_to_ros(message: Message, ros_message_type: str) -> Dict[str, Any]
```
Translate a Python ROS engine message to a native ROS message format.

- `message`: Python ROS engine message
- `ros_message_type`: Native ROS message type (e.g., "std_msgs/String")
- Returns: Dict - Native ROS message in dictionary format

#### ros_to_pyros
```python
ros_to_pyros(ros_message: Dict[str, Any], pyros_message_type: Type[Message]) -> Message
```
Translate a native ROS message to a Python ROS engine message.

- `ros_message`: Native ROS message in dictionary format
- `pyros_message_type`: Python ROS engine message type
- Returns: Message - Python ROS engine message

#### get_pyros_type
```python
get_pyros_type(ros_message_type: str) -> Type[Message]
```
Get the corresponding Python ROS engine message type for a native ROS message type.

- `ros_message_type`: Native ROS message type (e.g., "std_msgs/String")
- Returns: Type[Message] - Corresponding Python ROS engine message type

#### get_ros_type
```python
get_ros_type(pyros_message_type: Type[Message]) -> str
```
Get the corresponding native ROS message type for a Python ROS engine message type.

- `pyros_message_type`: Python ROS engine message type
- Returns: str - Corresponding native ROS message type