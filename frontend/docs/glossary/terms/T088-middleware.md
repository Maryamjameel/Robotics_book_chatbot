---
id: T088
term: Middleware
acronym:
categories:
  - Software Concepts
  - ROS 2 Architecture
related_terms:
  - ROS 2
  - API
  - Distributed System
  - Communication Protocol
chapter_introduced: 3
section_reference: 3.7
usage_example: ROS 2 acts as a crucial middleware, facilitating communication between different robot software components.
---
Middleware in robotics refers to software that bridges the gap between a robot's operating system or hardware and its application software. It provides a common set of services and abstractions that simplify the development of complex robotic systems, especially those with distributed components. Middleware typically handles inter-process communication, data serialization, message passing, and service discovery, allowing different modules (e.g., perception, planning, control) to communicate seamlessly without needing to know the low-level details of their underlying platforms. ROS (Robot Operating System) is a prominent example of robotics middleware.

### Key Concepts
- **Communication Bridge**: Connects diverse software components.
- **Abstraction Layer**: Hides low-level hardware and OS details.
- **Inter-Process Communication (IPC)**: Enables data exchange between separate processes.
- **Distributed Systems**: Facilitates coordination across multiple computational units.

### Related Terms
- ROS 2
- API
- Distributed System
- Communication Protocol

### See Also
- [Middleware on Wikipedia](https://en.wikipedia.org/wiki/Middleware)
- [Robot Operating System on Wikipedia](https://en.wikipedia.org/wiki/Robot_Operating_System)
- Robotics textbook chapter on ROS 2 Architecture
