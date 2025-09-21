# 🚀 Launch System

The launch system in Python ROS Engine provides a way to manage multiple nodes and monitor the overall system status. It allows you to start, stop, and monitor collections of nodes as a single unit.

## ⚙️ LaunchSystem Class

The `LaunchSystem` class is the main interface for managing multiple nodes.

### Methods

#### `__init__()`
Initialize the launch system.

#### `add_node(node_name, node_class, *args, **kwargs)`
Add a node to the launch system.

- `node_name`: Name to register the node under
- `node_class`: The node class to instantiate
- `*args`: Positional arguments for node initialization
- `**kwargs`: Keyword arguments for node initialization

#### `add_node_from_file(node_name, file_path, class_name, *args, **kwargs)`
Add a node to the launch system from a Python file.

- `node_name`: Name to register the node under
- `file_path`: Path to the Python file containing the node class
- `class_name`: Name of the node class in the file
- `*args`: Positional arguments for node initialization
- `**kwargs`: Keyword arguments for node initialization

#### `remove_node(node_name)`
Remove a node from the launch system.

- `node_name`: Name of the node to remove

#### `start()`
Start all nodes in the launch system.

#### `shutdown()`
Shutdown all nodes in the launch system.

#### `get_system_status()`
Get the status of the entire system as a dictionary.

#### `print_system_status()`
Print the status of the entire system in a human-readable format.

## 📄 LaunchDescription Class

The `LaunchDescription` class provides a declarative way to describe a launch configuration.

### Methods

#### `__init__(launch_system=None)`
Initialize a launch description.

- `launch_system`: Optional LaunchSystem to use (creates a new one if not provided)

#### `add_node(node_class, *args, **kwargs)`
Add a node action to the launch description.

- `node_class`: The node class to instantiate
- `*args`: Positional arguments for node initialization
- `**kwargs`: Keyword arguments for node initialization

#### `add_node_from_file(file_path, class_name, *args, **kwargs)`
Add a node action from a Python file to the launch description.

- `file_path`: Path to the Python file containing the node class
- `class_name`: Name of the node class in the file
- `*args`: Positional arguments for node initialization
- `**kwargs`: Keyword arguments for node initialization

#### `execute()`
Execute all actions in the launch description and return the launch system.

## 🧪 Usage Examples

### 💻 Programmatic Usage

```python
from pyros2 import LaunchSystem
from pyros2.message import String

class MyPublisherNode:
    def __init__(self):
        from pyros2 import Node
        self.node = Node("my_publisher")
        self.publisher = self.node.create_publisher(String, "/my_topic")
        
    def publish_message(self):
        msg = String()
        msg.data = "Hello World!"
        self.publisher.publish(msg)

# Create launch system
launch_system = LaunchSystem()

# Add nodes
launch_system.add_node("publisher", MyPublisherNode)

# Print system status
launch_system.print_system_status()

# Start the system
launch_system.start()
```

### 💻 Declarative Usage

```python
from pyros2 import LaunchDescription
from pyros2.message import String

class MyPublisherNode:
    def __init__(self):
        from pyros2 import Node
        self.node = Node("my_publisher")
        self.publisher = self.node.create_publisher(String, "/my_topic")

class MySubscriberNode:
    def __init__(self):
        from pyros2 import Node
        self.node = Node("my_subscriber")
        self.subscriber = self.node.create_subscription(
            String, "/my_topic", self.callback
        )
        
    def callback(self, msg):
        print(f"Received: {msg.data}")

# Create launch description
launch_description = LaunchDescription()

# Add nodes
launch_description.add_node(MyPublisherNode)
launch_description.add_node(MySubscriberNode)

# Execute the launch description
launch_system = launch_description.execute()

# Print system status
launch_system.print_system_status()

# Start the system
launch_system.start()
```

### 💻 Launch File Usage

You can also create launch files that can be executed from the command line:

```python
# launch_my_system.py
from pyros2 import LaunchDescription
from my_nodes import MyPublisherNode, MySubscriberNode

def generate_launch_description():
    launch_description = LaunchDescription()
    
    # Add nodes
    launch_description.add_node(MyPublisherNode)
    launch_description.add_node(MySubscriberNode)
    
    return launch_description
```

Then run it from the command line:
```bash
python -m pyros2.launch_cli launch_my_system.py
```

To just check the system status without starting the nodes:
```bash
python -m pyros2.launch_cli launch_my_system.py --status
```