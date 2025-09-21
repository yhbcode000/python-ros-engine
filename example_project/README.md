# 🤖 Python ROS Engine Example Project

This is a complete example project demonstrating the usage of the Python ROS Engine package.

## 📁 Project Structure

```
example_project/
├── config/
│   ├── __init__.py
│   ├── config.yaml
│   └── robot_config.yaml
├── nodes/
│   ├── __init__.py
│   ├── publisher_node.py
│   ├── subscriber_node.py
│   ├── service_node.py
│   └── client_node.py
├── launch/
│   ├── __init__.py
│   └── robot_system.py
└── README.md
```

## ⚙️ Installation

First, install the Python ROS Engine package:

```bash
pip install python-ros-engine
```

## ▶️ Running the Example

1. Start the publisher node:
   ```bash
   python nodes/publisher_node.py
   ```

2. In another terminal, start the subscriber node:
   ```bash
   python nodes/subscriber_node.py
   ```

3. In another terminal, start the service node:
   ```bash
   python nodes/service_node.py
   ```

4. In another terminal, start the client node:
   ```bash
   python nodes/client_node.py
   ```

5. To launch the complete robot system:
   ```bash
   python launch/robot_system.py
   ```

## 🛠️ Configuration

The example uses Hydra for configuration management. You can modify the configuration files in the `config/` directory to change the behavior of the nodes.