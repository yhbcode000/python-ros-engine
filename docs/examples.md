# Examples

The Python ROS Engine comes with several examples demonstrating its core functionality. These examples can be found in the `examples/` directory of the repository.

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