# Examples

The Python ROS Engine comes with several examples demonstrating its core functionality. These examples can be found in the `examples/` directory of the repository.

## System Architecture

The examples demonstrate various aspects of the Python ROS Engine architecture:

<pre class="mermaid">
graph TD
    A[Examples System] --> B[Publisher Node]
    A --> C[Subscriber Node]
    A --> D[Service Node]
    A --> E[Client Node]
    A --> F[Bridge Node]
    A --> G[Launch System]
    B --> H[(Topics)]
    C --> H
    E --> I[(Services)]
    D --> I
</pre>

Here's a sequence diagram showing the interaction between the publisher and subscriber nodes in the examples:

<pre class="mermaid">
sequenceDiagram
    participant PublisherNode
    participant Topic
    participant SubscriberNode
    PublisherNode->>Topic: Publish message
    Topic->>SubscriberNode: Deliver message
    SubscriberNode->>SubscriberNode: Process message in callback
</pre>

And here's a sequence diagram showing the service/client interaction:

<pre class="mermaid">
sequenceDiagram
    participant ClientNode
    participant ServiceNode
    ClientNode->>ServiceNode: Service request
    ServiceNode->>ServiceNode: Process request in callback
    ServiceNode->>ClientNode: Return response
    ClientNode->>ClientNode: Handle response
</pre>

## Publisher Example

This example demonstrates how to create a node with a publisher that sends messages to a topic.

```python
--8<-- "examples/publisher_example.py"
```

## Subscriber Example

This example demonstrates how to create a node with a subscriber that listens to messages on a topic.

```python
--8<-- "examples/subscriber_example.py"
```

## Service Example

This example demonstrates how to create a node that provides a service.

```python
--8<-- "examples/service_example.py"
```

## Client Example

This example demonstrates how to create a node that calls a service.

```python
--8<-- "examples/client_example.py"
```

## Bridge Example

This example demonstrates how to use the bridge functionality to discover native ROS nodes, topics, and services.

```python
--8<-- "examples/bridge_example.py"
```

## Launch System Examples

These examples demonstrate how to use the launch system to manage multiple nodes.

### Launch Description Example

This example shows how to create a launch description programmatically:

```python
--8<-- "examples/launch_system_example.py"
```

### Launch File Example

This example shows how to create a launch file that can be executed from the command line:

```python
--8<-- "examples/launch_example.py"
```

## Complete Example Project

In addition to the basic examples, we've included a complete example project in the `example_project/` directory that demonstrates how to build a robot system with multiple interconnected nodes.

The complete example project includes:
- Configuration files using Hydra for flexible parameter management
- Publisher, subscriber, service, and client nodes
- A launch system to run all nodes together as a cohesive robot system

You can explore this complete example to understand how to structure a real-world application using the Python ROS Engine.

## Running Examples

To run any of these examples, make sure you have installed the Python ROS Engine:

```bash
pip install python-ros-engine
```

Then you can run the examples directly with Python:

```bash
python examples/publisher_example.py
python examples/subscriber_example.py
python examples/service_example.py
python examples/client_example.py
python examples/bridge_example.py
python examples/launch_system_example.py
```

To run the launch file example:

```bash
python -m pyros2.launch_cli examples/launch_example.py
```

To just check the system status without starting the nodes:

```bash
python -m pyros2.launch_cli examples/launch_example.py --status
```

For the complete example project:

```bash
python example_project/nodes/publisher_node.py
python example_project/nodes/subscriber_node.py
python example_project/nodes/service_node.py
python example_project/nodes/client_node.py
python example_project/launch/robot_system.py
```

Note: For the bridge example to work, you need to have a ROS master running on localhost:11311.