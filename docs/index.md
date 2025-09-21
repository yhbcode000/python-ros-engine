# Python ROS Engine

A pure Python implementation of ROS2 core functionality with bridging capabilities to interact with native ROS nodes.

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

## Project Structure

<pre class="mermaid">
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
</pre>

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

## Getting Started

To get started with the Python ROS Engine, check out our [Installation Guide](installation.md) and [Usage Guide](usage.md).

## Documentation

- [API Documentation](api.md) - Detailed information about all classes and methods
- [Bridging with Native ROS](bridging.md) - Information about interacting with native ROS systems
- [Usage Guide](usage.md) - Comprehensive guide on how to use the engine
- [Diagrams Guide](diagrams.md) - Guide to using various diagram types for ROS documentation

## Examples

See our [Examples](examples.md) page for complete working examples demonstrating various features of the Python ROS Engine.

We've also included a [Complete Example Project](examples.md#complete-example-project) that shows how to build a robot system with multiple interconnected nodes using best practices.

## Contributing

We welcome contributions! Please see our [Contributing Guide](contributing.md) for details on how to contribute to this project.

## License

This project is licensed under the Apache License 2.0 - see the [License](license.md) file for details.