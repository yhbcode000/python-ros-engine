# Diagrams Guide

This guide demonstrates various types of diagrams that can be used to document ROS systems effectively.

## Sequence Diagrams

Sequence diagrams are excellent for showing the interaction between different components over time.

### Publisher-Subscriber Interaction

<pre class="mermaid">
sequenceDiagram
    participant PublisherNode
    participant Topic
    participant SubscriberNode1
    participant SubscriberNode2
    PublisherNode->>Topic: Publish message
    Topic->>SubscriberNode1: Deliver message
    Topic->>SubscriberNode2: Deliver message
</pre>

### Service-Client Interaction

<pre class="mermaid">
sequenceDiagram
    participant ClientNode
    participant ServiceNode
    ClientNode->>ServiceNode: Service request
    ServiceNode->>ServiceNode: Process request
    ServiceNode->>ClientNode: Return response
</pre>

## Flowcharts

Flowcharts are useful for showing processes and decision points.

### Node Lifecycle

<pre class="mermaid">
flowchart TD
    A[Node Creation] --> B[Configuration]
    B --> C[Activation]
    C --> D[Running]
    D --> E{Shutdown?}
    E -->|Yes| F[Cleanup]
    E -->|No| D
    F --> G[Node Destruction]
</pre>

### QoS Policy Selection

<pre class="mermaid">
flowchart LR
    A[Select QoS Policy] --> B{Reliability?}
    B -->|Critical| C[Reliable]
    B -->|Non-critical| D[Best Effort]
    C --> E{Durability?}
    D --> E
    E -->|Need history| F[Transient Local]
    E -->|No history| G[Volatile]
</pre>

## Class Diagrams

Class diagrams help visualize the structure of the system and relationships between classes.

### Core Node Components

<pre class="mermaid">
classDiagram
    class Node {
        +String name
        +String namespace
        +create_publisher()
        +create_subscription()
        +create_service()
        +create_client()
        +declare_parameter()
        +get_parameter()
        +set_parameter()
        +spin()
    }
    
    class Publisher {
        +publish()
    }
    
    class Subscriber {
        +callback
    }
    
    class Service {
        +handle_request()
    }
    
    class Client {
        +call()
    }
    
    class QoSProfile {
        +ReliabilityPolicy reliability
        +DurabilityPolicy durability
        +int depth
    }
    
    class Message {
        +serialize()
        +deserialize()
    }
    
    Node "1" --> "0..*" Publisher : creates
    Node "1" --> "0..*" Subscriber : creates
    Node "1" --> "0..*" Service : creates
    Node "1" --> "0..*" Client : creates
    Publisher --> Message : publishes
    Subscriber --> Message : subscribes
    Service --> Message : request/response
    Client --> Message : request/response
    Publisher --> QoSProfile : uses
    Subscriber --> QoSProfile : uses
    Service --> QoSProfile : uses
    Client --> QoSProfile : uses
</pre>

## State Diagrams

State diagrams are perfect for showing the different states a node can be in and transitions between them.

### Node States

<pre class="mermaid">
stateDiagram-v2
    [*] --> Unconfigured
    Unconfigured --> Inactive: configure
    Inactive --> Active: activate
    Active --> Inactive: deactivate
    Inactive --> Unconfigured: cleanup
    Unconfigured --> [*]: shutdown
    Inactive --> [*]: shutdown
    Active --> [*]: shutdown
</pre>

## Gantt Charts

Gantt charts can be useful for project planning and development timelines.

### Development Phases

<pre class="mermaid">
gantt
    title Python ROS Engine Development
    dateFormat  YYYY-MM-DD
    section Core Implementation
    Node System             :2023-01-01, 2023-02-15
    Publisher/Subscriber    :2023-02-16, 2023-03-15
    Service/Client          :2023-03-16, 2023-04-15
    Parameters              :2023-04-16, 2023-05-01
    section Bridge Features
    ROS1 Bridge             :2023-05-02, 2023-06-15
    Message Translation     :2023-06-16, 2023-07-15
    section Documentation
    API Docs               :2023-07-16, 2023-08-15
    Examples               :2023-08-16, 2023-09-15
</pre>