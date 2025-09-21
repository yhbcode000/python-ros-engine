# Bridging with Native ROS

The Python ROS engine provides capabilities to discover and interact with native ROS nodes.

## Bridge Connection

The bridge connects to the ROS master using XMLRPC protocol:

```python
from pyros2 import Bridge

# Default connection (localhost:11311)
bridge = Bridge()

# Custom connection
bridge = Bridge("192.168.1.100", 11312)
```

## Discovering ROS Nodes

```python
from pyros2 import Bridge

bridge = Bridge()
try:
    bridge.connect()
    nodes = bridge.discover_ros_nodes()
    print("Discovered ROS nodes:")
    for node in nodes:
        print(f"  - {node}")
except BridgeConnectionError as e:
    print(f"Failed to connect: {e}")
```

## Discovering ROS Topics

```python
from pyros2 import Bridge

bridge = Bridge()
try:
    bridge.connect()
    topics = bridge.discover_ros_topics()
    print("Discovered ROS topics:")
    for topic in topics:
        print(f"  - {topic['name']} (type: {topic['type']})")
except BridgeConnectionError as e:
    print(f"Failed to connect: {e}")
```

## Discovering ROS Services

```python
from pyros2 import Bridge

bridge = Bridge()
try:
    bridge.connect()
    services = bridge.discover_ros_services()
    print("Discovered ROS services:")
    for service in services:
        print(f"  - {service['name']} (providers: {service['providers']})")
except BridgeConnectionError as e:
    print(f"Failed to connect: {e}")
```

## Requirements for Bridging

To use the bridging capabilities, you need:

1. A running ROS master (ROS1)
2. Network connectivity to the ROS master
3. Proper permissions to access the ROS master

## Setting up ROS Master

If you don't have ROS installed, you can run a simple ROS master using roscore:

```bash
roscore
```

This will start the ROS master on localhost:11311 by default.

## Limitations

The current bridging implementation:

1. Only supports ROS1 master discovery
2. Does not support message translation between Python ROS engine and native ROS
3. Does not support parameter synchronization
4. Does not support service request/response translation

Future versions will expand these capabilities to provide full interoperability with native ROS systems.