---
id: lesson-03-ros2-fundamentals
title: "Lesson 3: ROS 2 Architecture Fundamentals"
---

## 1.3: The Robotic Nervous System: ROS 2

### 1.3.1: ROS 2 Architecture Fundamentals

**ROS 2 (Robot Operating System 2)** is middleware that enables decoupled, modular control of complex robot systems. Instead of writing one monolithic control program, ROS 2 lets you build robots as networks of independent **nodes** that communicate through well-defined message types.

**Core Concepts:**

**Nodes**: Independent processes that perform specific tasks (perception, planning, control, etc.)

**Topics**: Named communication channels where nodes publish and subscribe to messages

**Services**: Request-reply communication pattern for one-off tasks

**Actions**: Long-running task pattern for behaviors like "move to position" or "pick up object"

**Parameters**: Configuration values that can be changed without recompiling code

**Launch Files**: Scripts that start multiple nodes with configured parameters

This architecture enables:
- **Modularity**: Different teams can develop perception, planning, and control independently
- **Reusability**: Standard ROS packages provide common functionality (navigation, manipulation, perception)
- **Testing**: Nodes can be tested independently or in simulation before running on hardware
- **Debugging**: Nodes can be launched, stopped, and restarted without affecting others

### 1.3.2: Nodes, Topics, and the Pub/Sub Pattern

A **node** is an independent ROS 2 process. A **topic** is a named channel for communication. The **publish-subscribe (pub/sub) pattern** allows nodes to communicate loosely coupled:

- **Publisher nodes** write messages to a topic
- **Subscriber nodes** read messages from a topic
- Publishers and subscribers do not need to know about each other

**Example: A Simple Publisher**

This Python node publishes "Hello, world!" messages every second:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic_hello_world', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self) -> None:
        msg = String()
        msg.data = f'Hello, world! Message #{self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None) -> None:
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Example: A Simple Subscriber**

This Python node listens to messages and prints them:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic_hello_world',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg: String) -> None:
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None) -> None:
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Running Both Together:**

```bash
# Terminal 1
ros2 run learning_ros2 publisher_node

# Terminal 2
ros2 run learning_ros2 subscriber_node
```

The publisher and subscriber communicate through the topic without any direct connection.

### 1.3.3: Services and Actions

While topics implement publish-subscribe (one-way communication), ROS 2 provides two additional communication patterns:

| Pattern | Use Case | Timing | Example |
|---------|----------|--------|---------|
| **Topic** | Continuous data streams | Asynchronous, one-way | Camera frames, sensor readings |
| **Service** | One-off requests | Synchronous, request-reply | "What is the current joint angle?" |
| **Action** | Long-running tasks | Asynchronous, with feedback | "Move arm to position" (reports progress) |

**Services**: A node requests something and waits for a response. The client blocks until the server replies.

**Actions**: A node requests a goal and continues running. The action server provides periodic feedback and a final result.
