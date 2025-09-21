# 📁 Python ROS Engine Project Structure

## 📂 Directory Layout
```
python_ros_engine/
├── src/
│   └── pyros2/
│       ├── __init__.py
│       ├── node.py              # Node base class and lifecycle management
│       ├── publisher.py         # Publisher implementation
│       ├── subscriber.py        # Subscriber implementation
│       ├── service.py           # Service server implementation
│       ├── client.py            # Service client implementation
│       ├── parameter.py         # Parameter server and management
│       ├── discovery.py         # Topic/node discovery mechanisms
│       ├── qos.py              # Quality of Service profiles
│       ├── message.py          # Base message class
│       ├── bridge.py           # ROS1/ROS2 bridging capabilities
│       └── exceptions.py      # Custom exceptions
├── tests/
│   ├── __init__.py
│   ├── test_node.py
│   ├── test_publisher_subscriber.py
│   ├── test_service_client.py
│   ├── test_parameter.py
│   ├── test_discovery.py
│   ├── test_qos.py
│   └── test_bridge.py
├── config/
│   ├── __init__.py
│   └── hydra_config.py        # Hydra configuration implementation
├── examples/
│   ├── __init__.py
│   ├── publisher_example.py
│   ├── subscriber_example.py
│   ├── service_example.py
│   ├── client_example.py
│   └── bridge_example.py
├── docs/
│   ├── api.md
│   ├── usage.md
│   ├── bridging.md
│   ├── installation.md
│   ├── examples.md
│   ├── index.md
│   ├── contributing.md
│   ├── license.md
│   ├── css/
│   ├── javascripts/
│   └── templates/
├── .github/
│   └── workflows/
│       └── deploy.yml         # GitHub Actions deployment configuration
├── README.md
├── LICENSE
├── setup.py
├── pyproject.toml
├── requirements.txt
├── build_docs.py             # Documentation build and serve script
└── mkdocs.yml                # MkDocs configuration
```

## 📦 Module Descriptions

### ⚙️ Core Modules (src/pyros2/)
1. **node.py** - Implements the Node base class with lifecycle management
2. **publisher.py** - Implements publisher functionality with topic registration
3. **subscriber.py** - Implements subscriber functionality with callback handling
4. **service.py** - Implements service server functionality
5. **client.py** - Implements service client functionality
6. **parameter.py** - Implements parameter server and management
7. **discovery.py** - Implements discovery mechanisms for topics and nodes
8. **qos.py** - Implements Quality of Service profiles
9. **message.py** - Base message class for all ROS messages
10. **bridge.py** - Implements bridging capabilities to native ROS
11. **exceptions.py** - Custom exceptions for the ROS engine

### 🛠️ Configuration (config/)
1. **hydra_config.py** - Implements configuration using Hydra best practices

### 🧪 Testing (tests/)
1. **test_node.py** - Tests for node creation and lifecycle
2. **test_publisher_subscriber.py** - Tests for publisher/subscriber functionality
3. **test_service_client.py** - Tests for service/client functionality
4. **test_parameter.py** - Tests for parameter handling
5. **test_discovery.py** - Tests for discovery mechanisms
6. **test_qos.py** - Tests for QoS profiles
7. **test_bridge.py** - Tests for bridging capabilities

### 📚 Examples (examples/)
1. **publisher_example.py** - Example publisher node
2. **subscriber_example.py** - Example subscriber node
3. **service_example.py** - Example service server
4. **client_example.py** - Example service client
5. **bridge_example.py** - Example showing bridging capabilities

### 📖 Documentation (docs/)
All documentation is written in Markdown format and uses Mermaid diagrams for visual explanations:
1. **api.md** - API documentation for all modules
2. **usage.md** - Usage examples and tutorials
3. **bridging.md** - Documentation on bridging with native ROS
4. **installation.md** - Installation guide
5. **examples.md** - Detailed examples
6. **index.md** - Main documentation page
7. **contributing.md** - Contribution guidelines
8. **license.md** - License information
9. **css/** - Custom CSS styling
10. **javascripts/** - Custom JavaScript enhancements
11. **templates/** - Custom HTML templates

### 🛠️ Documentation Tools
1. **build_docs.py** - Script to build and serve documentation locally with `mkdocs serve`
2. **mkdocs.yml** - MkDocs configuration file defining site structure and plugins

### ☁️ Deployment
1. **setup.py** - Package setup configuration
2. **pyproject.toml** - Modern Python package configuration
3. **requirements.txt** - Dependencies
4. **.github/workflows/deploy.yml** - GitHub Actions deployment workflow