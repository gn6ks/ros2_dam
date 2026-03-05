# 2. Fundamental Concepts

This chapter establishes the theoretical foundation necessary for understanding ROS2 architecture and its core abstractions. Mastery of these concepts is essential for effective robot development, simulation, and deployment. Each concept is presented with emphasis on its practical application within the ROS2 ecosystem.

## 2.1 The ROS2 Architecture

ROS2 is built upon a distributed system architecture that enables modular, decoupled software components to communicate efficiently. Unlike monolithic robotic frameworks, ROS2 employs a peer-to-peer communication model where each component operates independently while maintaining the ability to discover and interact with other components dynamically.

The architecture is fundamentally based on the **Data Distribution Service (DDS)** standard, which provides the middleware layer responsible for data serialization, transport, and quality of service guarantees. This design decision enables ROS2 to support real-time requirements, fault tolerance, and platform heterogeneity.

## 2.2 The ROS Graph

The **ROS Graph** is a network representation of all active nodes and their communication relationships within a ROS2 system. It provides a conceptual model for understanding how data flows through the system.

### 2.2.1 Graph Components

The ROS Graph consists of:

- **Nodes**: Represented as vertices in the graph.
- **Topics**: Directed edges representing publish-subscribe communication channels.
- **Services**: Bidirectional edges representing request-response interactions.
- **Actions**: Specialized edges representing long-running, preemptable tasks.

### 2.2.2 Graph Demostration

<figure>
  <img src="./assets/gifs/Nodes-TopicandService.gif"
       alt="ROS Graph Communication Flow"
       width="750">
  <figcaption>Figure 2.1: Real-time visualization of ROS2 node discovery, topic publication, and service request-response patterns. The diagram illustrates how nodes dynamically establish communication channels through the DDS middleware.</figcaption>
</figure>

### 2.2.3 Graph Introspection

ROS2 provides command-line tools for examining the ROS Graph:

- `ros2 node list`: Displays all active nodes.
- `ros2 topic list`: Enumerates all active topics.
- `ros2 service list`: Lists all available services.
- `ros2 graph`: Visualizes the complete node and topic topology.

Understanding the ROS Graph is critical for debugging and system optimization, as it reveals communication patterns, potential bottlenecks, and dependencies between components.

## 2.3 Nodes

A **node** is the fundamental computational unit in ROS2, representing a single process that performs a specific function. Nodes embody the principle of modularity, allowing complex robotic systems to be decomposed into manageable, reusable components.

### 2.3.1 Node Characteristics

Each node in ROS2 exhibits the following properties:

- **Single Responsibility**: A node should perform one well-defined task, such as reading sensor data, controlling actuators, or executing path planning algorithms.
- **Independence**: Nodes can be started, stopped, and restarted independently without requiring system-wide coordination.
- **Language Agnosticism**: Nodes can be implemented in any programming language that supports ROS2 interfaces, with C++ and Python being the most commonly used.
- **Discoverability**: Nodes can dynamically discover other nodes in the system through the ROS2 discovery mechanism.

### 2.3.2 Node Lifecycle

Nodes follow a defined lifecycle from instantiation to termination:

1. **Creation**: The node is instantiated with a unique name within the ROS2 domain. (set with the ROS_DOMAIN_ID environment variable)
2. **Initialization**: Resources are allocated, and communication interfaces are established.
3. **Execution**: The node performs its designated function, processing inputs and generating outputs.
4. **Termination**: Resources are released, and the node gracefully exits.

In the context of robot simulation, typical nodes include sensor drivers, controller interfaces, state estimation algorithms, and visualization components. Each node contributes to the overall system behavior while maintaining loose coupling with other components.

## 2.4 Discovery Mechanism

ROS2 employs a **dynamic discovery mechanism** that allows nodes to automatically detect and communicate with each other without explicit configuration. This capability is inherited from the underlying DDS middleware.

### 2.4.1 Discovery Process

When a node joins the ROS2 domain, the following sequence occurs:

1. **Announcement**: The node broadcasts its presence on the network using multicast DNS or DDS discovery protocols.
2. **Detection**: Existing nodes detect the new node's announcement.
3. **Matching**: Nodes compare their communication requirements (topics, services, QoS policies).
4. **Connection Establishment**: Compatible nodes establish direct communication channels.

### 2.4.2 Discovery Configuration

Discovery behavior can be configured through:

- **Domain ID**: Nodes only discover other nodes within the same domain (default: 0).
- **Discovery Server**: For wide-area networks, a centralized discovery server can coordinate node detection.
- **Static Discovery**: In environments where multicast is unavailable, static peer lists can be configured.

The discovery mechanism is particularly important in simulation environments where multiple nodes must coordinate in real-time, and in multi-robot systems where dynamic team composition is required.

## 2.5 Communication Interfaces

ROS2 defines several communication paradigms, each suited to different interaction patterns. Selecting the appropriate interface is crucial for system performance and correctness.

### 2.5.1 Topics (Publish-Subscribe)

**Topics** implement the publish-subscribe pattern, enabling asynchronous, one-to-many (1-N) communication. This is the most common communication mechanism in ROS2.

#### Characteristics

- **Asynchronous**: Publishers and subscribers operate independently without blocking.
- **Anonymous**: Publishers do not know which subscribers exist, and vice versa.
- **Streaming**: Topics are optimized for continuous data streams such as sensor readings or control commands.
- **Best-Effort or Reliable**: Quality of Service (QoS) policies determine delivery guarantees.

#### Data Recording Capabilities

The publish-subscribe architecture enables transparent data recording without modifying existing nodes. External tools can leverage this capability by dynamically creating subscribers to specified topics at runtime. A primary example of this functionality in ROS2 is the `ros2 bag record` utility.

**Key Characteristics:**

- **Non-Intrusive Operation**: Recording tools such as `ros2 bag record` create subscribers that operate independently, without interrupting the flow of data to other parts of the system.
- **Transparent Integration**: Existing publishers and subscribers continue functioning normally; the recording tool is simply another anonymous subscriber within the ROS graph.
- **Selective Data Capture**: Users can choose specific topics for recording or capture the entire ROS graph depending on analysis requirements.
- **Timestamp Preservation**: Messages are stored with their original timestamps, enabling accurate playback, post-processing, and analysis.

This design exemplifies the power of the publish-subscribe pattern: new functionality, such as data logging via `ros2 bag record`, can be added to the system without modifying existing components or disrupting ongoing operations. The decoupled nature of topic-based communication ensures that system extensibility does not compromise stability or performance.

#### Use Cases in Simulation

- Sensor data publication (camera images, LiDAR scans, IMU readings)
- Command streaming (velocity commands, joint trajectories)
- State broadcasting (odometry, transform information)

## 2.6 Services

**Services** implement a synchronous *request-response* communication model within the ROS2 ecosystem. This mechanism enables nodes to execute discrete operations and await a result before continuing their execution flow.

### 2.6.1 System Architecture

The interaction is based on a binary relationship between two primary entities:

- **Server:** Advertises a service, listens for incoming requests, executes computation logic, and returns a response.
- **Client:** Locates services on the network, sends a request, and blocks execution until receiving the server's return.

> **Technical Note:** Unlike *Topics*, *Services* are not continuous data streams, but rather unique, finalized transactions that require reception confirmation.

### 2.6.2 Interface Definition (.srv)

Service interfaces are structured in `.srv` files, where request and response fields are separated by a triple dash delimiter:

```yaml
# .srv file structure (example: AddTwoInts.srv)
uint32 a
uint32 b
---
uint32 sum
```

### 2.6.3 Execution Flow and Console

The operational process follows a logical sequence managed by the DDS middleware, from service discovery to client thread resumption. For manual validation from the terminal, the following syntax is used:

```bash
# Example: Service call from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
```

### 2.6.4 Application Analysis and Constraints

| Category | Description |
| --- | --- |
| **Use Cases** | Simulation control (spawn/reset), configuration changes, punctual status queries (sensors, maps). |
| **Limitations** | Client execution thread blocking, lack of intermediate feedback, inability to cancel. |
| **Recommendation** | For long-duration tasks or those requiring preemption, **Actions** (Section 2.7) must be used. |

---

### 2.6.5 Introspection Tools

The ROS2 CLI (*Command Line Interface*) provides specific tools for service inspection at runtime:

| Command | Function |
| --- | --- |
| `ros2 service list` | Enumerates all active services in the ROS graph. |
| `ros2 service type /name` | Identifies the interface type associated with a service. |
| `ros2 interface show pkg/Srv` | Breaks down the internal structure (fields) of the `.srv` definition. |

## 2.7 Actions

**Actions** implement long-running remote procedure calls in ROS2, providing feedback capabilities and support for cancellation or preemption. They are designed for tasks that extend over time and require monitoring or intervention.

### 2.7.1 Architecture

Actions involve two complementary entities:

- **Action Server:** Accepts goal requests, executes the procedure, sends periodic feedback, handles cancellation/preemption requests, and returns the final result. Only one server should exist per action name.
- **Action Client:** Sends goal requests to the server, receives feedback during execution, and can cancel or preempt the goal if needed. Multiple clients may target the same action name.

> **Technical Note:** Actions are intended for long-running procedures due to connection setup overhead. For short operations, use **Services** instead.

### 2.7.2 Action Definition (.action)

Action interfaces are defined in `.action` files with three sections separated by `---`:

```yaml
# Goal (request)
int32 order
---
# Result (response)
int32[] sequence
---
# Feedback (intermediate updates)
int32[] sequence
```

### 2.7.3 Communication Flow

```yaml
Client → Goal → Server
Client ← Feedback ← Server (periodic)
Client ← Result ← Server (on completion)
Client → Cancel Request → Server (optional)
```

### 2.7.4 Use Cases

| Application | Example |
| --- | --- |
| **Navigation** | Travel to waypoint with progress updates |
| **Manipulation** | Execute pick-and-place trajectory |
| **Exploration** | Autonomous mapping with cancellation option |
| **State Machines** | High-level task orchestration |

### 2.7.5 Comparison: Services vs Actions

| Feature | Service | Action |
| --- | --- | --- |
| Duration | Short operations | Long-running tasks |
| Feedback | None | Periodic updates |
| Cancellation | Not supported | Supported |
| Overhead | Low | Higher |

### 2.7.6 Introspection

```bash
ros2 action list                     # List available actions
ros2 action type /name               # Show action type
ros2 action info /name               # Show action status
ros2 action send_goal /name pkg/Action "{order: 5}"  # Send goal
```

### 2.7.7 Summary

Actions provide the appropriate communication paradigm for long-running, preemptable tasks requiring intermediate feedback. The three-part message structure (goal, result, feedback) enables robust task monitoring and control, distinguishing Actions from the simpler request-response pattern of Services.

## 2.8 Parameters

**Parameters** provide a runtime configuration mechanism for ROS2 nodes, allowing values to be modified without recompilation. They function as a dynamic configuration system accessible through introspection tools and are typically used for tunable values such as gains, thresholds, and hardware settings.

### 2.8.1 Parameter Characteristics

| Characteristic | Description |
| --- | --- |
| **Typed Values** | Support strings, integers, floats, booleans, and arrays |
| **Dynamic Reconfiguration** | Can be modified while the node is running |
| **Persistence** | Can be saved to YAML files for reproducibility |
| **Namespacing** | Organized hierarchically using node namespaces |

### 2.8.2 Parameter Commands

The ROS2 CLI provides the following commands for parameter management:

| Command | Function | Example |
| --- | --- | --- |
| `ros2 param list` | List all parameters for a node | `ros2 param list /turtlebot3_node` |
| `ros2 param get` | Retrieve a parameter value | `ros2 param get /node param_name` |
| `ros2 param set` | Modify a parameter value | `ros2 param set /node param_name value` |
| `ros2 param delete` | Remove a parameter | `ros2 param delete /node param_name` |
| `ros2 param describe` | Show parameter metadata (type, constraints) | `ros2 param describe /node param_name` |
| `ros2 param dump` | Export all parameters to YAML file | `ros2 param dump /node > config.yaml` |
| `ros2 param load` | Import parameters from YAML file | `ros2 param load /node config.yaml` |

### 2.8.3 Practical Examples

```bash
# List all parameters for a specific node
ros2 param list /turtlebot3_node

# Get a specific parameter value
ros2 param get /turtlebot3_node odom_frame_id

# Set a parameter value (runtime)
ros2 param set /turtlebot3_node odom_frame_id "odom"

# Delete a parameter
ros2 param delete /turtlebot3_node temp_param

# Describe parameter metadata
ros2 param describe /turtlebot3_node odom_frame_id

# Dump all parameters to YAML file
ros2 param dump /turtlebot3_node > turtlebot_params.yaml

# Load parameters from YAML file
ros2 param load /turtlebot3_node turtlebot_params.yaml
```

### 2.8.4 Use Cases

- **Sensor Configuration:** Resolution, frame rate, calibration values
- **Controller Tuning:** PID gains, maximum velocities, acceleration limits
- **Algorithm Parameters:** Costmap resolution, planner timeouts, threshold values
- **Hardware Settings:** Port names, baud rates, device IDs
