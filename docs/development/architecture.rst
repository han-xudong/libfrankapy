Architecture
============

This document describes the architecture and design principles of LibFrankaPy.

Overview
--------

LibFrankaPy is designed as a hybrid Python-C++ library that provides high-level Python interfaces while maintaining real-time performance through C++ implementation. The architecture ensures safety, performance, and ease of use.

.. code-block:: text

   ┌─────────────────────────────────────────────────────────────┐
   │                    Python Layer                            │
   │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
   │  │   Robot     │  │   Control   │  │    State    │        │
   │  │   Interface │  │  Algorithms │  │ Management  │        │
   │  └─────────────┘  └─────────────┘  └─────────────┘        │
   └─────────────────────────────────────────────────────────────┘
                                │
                                │ pybind11
                                ▼
   ┌─────────────────────────────────────────────────────────────┐
   │                     C++ Layer                              │
   │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
   │  │  Real-time  │  │   Shared    │  │   Safety    │        │
   │  │ Controller  │  │   Memory    │  │  Monitor    │        │
   │  └─────────────┘  └─────────────┘  └─────────────┘        │
   └─────────────────────────────────────────────────────────────┘
                                │
                                │ libfranka
                                ▼
   ┌─────────────────────────────────────────────────────────────┐
   │                  Franka Robot                               │
   └─────────────────────────────────────────────────────────────┘

Design Principles
-----------------

Real-time Performance
^^^^^^^^^^^^^^^^^^^^^

- **C++ Control Loop**: Critical control operations run in C++ at 1kHz
- **Non-blocking Python**: Python never blocks the real-time loop
- **Shared Memory**: Efficient data exchange between Python and C++
- **Atomic Operations**: Thread-safe communication without locks

Safety First
^^^^^^^^^^^^

- **Multiple Safety Layers**: Hardware, software, and application-level safety
- **Graceful Degradation**: System continues operating safely when components fail
- **Error Recovery**: Automatic recovery from common error conditions
- **Emergency Stop**: Immediate stop capability at all levels

Modular Design
^^^^^^^^^^^^^^

- **Separation of Concerns**: Clear boundaries between components
- **Pluggable Architecture**: Easy to extend and customize
- **Interface Abstraction**: Implementation details hidden behind clean APIs
- **Testable Components**: Each module can be tested independently

Python-Friendly
^^^^^^^^^^^^^^^

- **Intuitive APIs**: Pythonic interfaces that feel natural
- **Type Safety**: Comprehensive type hints and runtime checking
- **Error Handling**: Python exceptions for all error conditions
- **Documentation**: Complete API documentation with examples

Core Components
---------------

Python Layer
^^^^^^^^^^^^

**FrankaRobot Class**

The main interface for robot control:

.. code-block:: python

   class FrankaRobot:
       """Main robot interface."""
       
       def __init__(self, ip_address: str) -> None:
           self._controller = RealTimeController(ip_address)
           self._state_manager = StateManager()
           self._safety_monitor = SafetyMonitor()
       
       def connect(self) -> None:
           """Connect to robot."""
           
       def move_to_joint(self, positions: List[float]) -> bool:
           """Move to joint positions."""

**Control Module**

Provides various control algorithms:

.. code-block:: python

   # Joint space control
   class JointController:
       def move_to_position(self, target: List[float]) -> bool
       def set_velocity(self, velocity: List[float]) -> None
   
   # Cartesian space control
   class CartesianController:
       def move_to_pose(self, pose: np.ndarray) -> bool
       def set_velocity(self, velocity: List[float]) -> None
   
   # Force control
   class ForceController:
       def apply_force(self, force: List[float]) -> None
       def set_impedance(self, stiffness: List[float]) -> None

**State Management**

Handles robot state information:

.. code-block:: python

   class RobotState:
       joint_state: JointState
       cartesian_state: CartesianState
       force_state: ForceState
       system_state: SystemState
   
   class StateManager:
       def get_current_state(self) -> RobotState
       def subscribe_to_updates(self, callback: Callable) -> None

C++ Layer
^^^^^^^^^

**Real-time Controller**

Core real-time control implementation:

.. code-block:: cpp

   class RealTimeController {
   public:
       bool initialize(const std::string& robot_ip);
       void start_control_loop();
       void stop_control_loop();
       
       // Command interface
       void set_joint_command(const JointCommand& cmd);
       void set_cartesian_command(const CartesianCommand& cmd);
       
   private:
       void control_loop();  // 1kHz control loop
       franka::Robot robot_;
       std::atomic<bool> running_;
   };

**Shared Memory System**

Efficient data exchange:

.. code-block:: cpp

   template<typename T>
   class SharedBuffer {
   public:
       void write(const T& data);
       T read() const;
       bool try_read(T& data) const;
       
   private:
       std::atomic<T> data_;
       std::atomic<uint64_t> sequence_;
   };
   
   class SharedMemoryManager {
   public:
       SharedBuffer<RobotState> robot_state;
       SharedBuffer<JointCommand> joint_command;
       SharedBuffer<CartesianCommand> cartesian_command;
   };

**Safety Monitor**

Continuous safety monitoring:

.. code-block:: cpp

   class SafetyMonitor {
   public:
       void check_joint_limits(const JointState& state);
       void check_velocity_limits(const JointState& state);
       void check_force_limits(const ForceState& state);
       void check_workspace_limits(const CartesianState& state);
       
   private:
       SafetyConfig config_;
       std::vector<SafetyViolation> violations_;
   };

Data Flow
---------

Command Flow
^^^^^^^^^^^^

.. code-block:: text

   Python Application
           │
           │ robot.move_to_joint(target)
           ▼
   FrankaRobot.move_to_joint()
           │
           │ Validate command
           ▼
   JointController.move_to_position()
           │
           │ Generate trajectory
           ▼
   SharedMemoryManager.joint_command
           │
           │ Atomic write
           ▼
   RealTimeController.control_loop()
           │
           │ Read command
           ▼
   franka::Robot.control()
           │
           │ Send to robot
           ▼
   Franka Robot Hardware

State Flow
^^^^^^^^^^

.. code-block:: text

   Franka Robot Hardware
           │
           │ 1kHz sensor data
           ▼
   franka::Robot.readOnce()
           │
           │ Process state
           ▼
   RealTimeController.control_loop()
           │
           │ Update shared memory
           ▼
   SharedMemoryManager.robot_state
           │
           │ Atomic read
           ▼
   StateManager.get_current_state()
           │
           │ Convert to Python objects
           ▼
   Python Application

Thread Architecture
-------------------

Thread Model
^^^^^^^^^^^^

.. code-block:: text

   ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
   │  Python Main    │    │   C++ Control   │    │  Safety Monitor │
   │     Thread      │    │     Thread      │    │     Thread      │
   │                 │    │                 │    │                 │
   │ • User commands │    │ • 1kHz control  │    │ • Safety checks │
   │ • State queries │    │ • Robot comm    │    │ • Error handling│
   │ • Error handling│    │ • Shared memory │    │ • Recovery      │
   └─────────────────┘    └─────────────────┘    └─────────────────┘
           │                        │                        │
           └────────────────────────┼────────────────────────┘
                                    │
                            Shared Memory
                         (Lock-free queues)

Real-time Thread
^^^^^^^^^^^^^^^^

.. code-block:: cpp

   void RealTimeController::control_loop() {
       // Set real-time priority
       set_realtime_priority(80);
       
       // Pin to specific CPU core
       set_cpu_affinity(2);
       
       while (running_) {
           auto start_time = std::chrono::high_resolution_clock::now();
           
           // Read robot state (non-blocking)
           auto robot_state = robot_.readOnce();
           
           // Update shared state (atomic)
           shared_memory_.robot_state.write(robot_state);
           
           // Read commands (non-blocking)
           JointCommand joint_cmd;
           if (shared_memory_.joint_command.try_read(joint_cmd)) {
               // Execute joint command
               execute_joint_command(joint_cmd);
           }
           
           // Safety checks
           safety_monitor_.check_all(robot_state);
           
           // Maintain 1kHz frequency
           auto end_time = std::chrono::high_resolution_clock::now();
           auto duration = end_time - start_time;
           std::this_thread::sleep_until(start_time + std::chrono::milliseconds(1));
       }
   }

Memory Management
-----------------

Shared Memory Design
^^^^^^^^^^^^^^^^^^^^

.. code-block:: cpp

   // Lock-free circular buffer for high-frequency data
   template<typename T, size_t Size>
   class CircularBuffer {
   private:
       alignas(64) std::array<T, Size> buffer_;
       alignas(64) std::atomic<size_t> head_{0};
       alignas(64) std::atomic<size_t> tail_{0};
   };
   
   // Single-producer, single-consumer queue
   template<typename T>
   class SPSCQueue {
   public:
       bool try_push(const T& item);
       bool try_pop(T& item);
   };

Memory Layout
^^^^^^^^^^^^^

.. code-block:: text

   Shared Memory Region:
   ┌─────────────────────────────────────────────────────────┐
   │  Control Block (64 bytes)                              │
   │  • Version info                                         │
   │  • Process IDs                                          │
   │  • Status flags                                         │
   ├─────────────────────────────────────────────────────────┤
   │  Robot State Buffer (cache-line aligned)               │
   │  • Current state                                        │
   │  • Previous state                                       │
   │  • Sequence numbers                                     │
   ├─────────────────────────────────────────────────────────┤
   │  Command Queues (lock-free)                            │
   │  • Joint commands                                       │
   │  • Cartesian commands                                   │
   │  • Force commands                                       │
   ├─────────────────────────────────────────────────────────┤
   │  Error/Event Log (circular buffer)                     │
   │  • Error messages                                       │
   │  • Performance metrics                                  │
   │  • Debug information                                    │
   └─────────────────────────────────────────────────────────┘

Error Handling
--------------

Error Propagation
^^^^^^^^^^^^^^^^^

.. code-block:: text

   Hardware Error
           │
           ▼
   libfranka Exception
           │
           ▼
   C++ Error Handler
           │
           │ Log error
           │ Attempt recovery
           ▼
   Shared Memory Error Flag
           │
           ▼
   Python Error Checker
           │
           │ Convert to Python exception
           ▼
   Python Application

Recovery Strategies
^^^^^^^^^^^^^^^^^^^

.. code-block:: cpp

   class ErrorRecovery {
   public:
       enum class RecoveryAction {
           NONE,
           RETRY,
           RESET_CONTROLLER,
           EMERGENCY_STOP
       };
       
       RecoveryAction determine_action(const FrankaError& error) {
           switch (error.type()) {
               case ErrorType::COMMUNICATION_TIMEOUT:
                   return RecoveryAction::RETRY;
               case ErrorType::JOINT_LIMIT_VIOLATION:
                   return RecoveryAction::RESET_CONTROLLER;
               case ErrorType::COLLISION_DETECTED:
                   return RecoveryAction::EMERGENCY_STOP;
               default:
                   return RecoveryAction::NONE;
           }
       }
   };

Performance Considerations
--------------------------

Latency Optimization
^^^^^^^^^^^^^^^^^^^^

- **CPU Affinity**: Bind real-time thread to dedicated CPU core
- **Memory Locking**: Prevent memory pages from being swapped
- **Cache Optimization**: Align data structures to cache lines
- **Minimal Allocations**: Avoid dynamic memory allocation in real-time path

Throughput Optimization
^^^^^^^^^^^^^^^^^^^^^^^

- **Batch Operations**: Group multiple commands when possible
- **Vectorization**: Use SIMD instructions for mathematical operations
- **Memory Prefetching**: Hint processor about future memory accesses
- **Lock-free Algorithms**: Avoid blocking operations

Benchmarking
^^^^^^^^^^^^

.. code-block:: cpp

   class PerformanceMonitor {
   public:
       void start_measurement(const std::string& name);
       void end_measurement(const std::string& name);
       void report_statistics();
       
   private:
       struct Measurement {
           std::chrono::high_resolution_clock::time_point start;
           std::chrono::nanoseconds total_time{0};
           size_t count{0};
       };
       
       std::unordered_map<std::string, Measurement> measurements_;
   };

Extensibility
-------------

Plugin Architecture
^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   class ControllerPlugin:
       """Base class for controller plugins."""
       
       def initialize(self, robot: FrankaRobot) -> bool:
           """Initialize the plugin."""
           pass
       
       def update(self, state: RobotState) -> Optional[Command]:
           """Update controller and return command."""
           pass
       
       def cleanup(self) -> None:
           """Cleanup resources."""
           pass
   
   # Example custom controller
   class MyCustomController(ControllerPlugin):
       def update(self, state: RobotState) -> Optional[Command]:
           # Custom control logic
           return JointCommand(positions=[...])

Configuration System
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   class ConfigurationManager:
       """Manages system configuration."""
       
       def load_config(self, config_file: str) -> Dict[str, Any]:
           """Load configuration from file."""
           
       def validate_config(self, config: Dict[str, Any]) -> bool:
           """Validate configuration parameters."""
           
       def apply_config(self, config: Dict[str, Any]) -> None:
           """Apply configuration to system."""

Testing Architecture
--------------------

Test Hierarchy
^^^^^^^^^^^^^^

.. code-block:: text

   Tests
   ├── Unit Tests
   │   ├── Python unit tests (pytest)
   │   └── C++ unit tests (Google Test)
   ├── Integration Tests
   │   ├── Python-C++ integration
   │   └── Robot hardware tests
   ├── Performance Tests
   │   ├── Latency benchmarks
   │   └── Throughput tests
   └── System Tests
       ├── End-to-end scenarios
       └── Stress tests

Mocking Framework
^^^^^^^^^^^^^^^^^

.. code-block:: python

   class MockRobot:
       """Mock robot for testing without hardware."""
       
       def __init__(self):
           self.state = RobotState()
           self.commands = []
       
       def move_to_joint(self, positions: List[float]) -> bool:
           self.commands.append(('move_to_joint', positions))
           return True
       
       def get_robot_state(self) -> RobotState:
           return self.state

Future Enhancements
-------------------

Planned Features
^^^^^^^^^^^^^^^

- **Multi-robot Support**: Control multiple robots simultaneously
- **Advanced Planning**: Integration with motion planning libraries
- **Machine Learning**: Support for learning-based control
- **Cloud Integration**: Remote monitoring and control capabilities
- **Simulation**: Integration with physics simulators

Architecture Evolution
^^^^^^^^^^^^^^^^^^^^^^

- **Microservices**: Split into smaller, independent services
- **Event-driven**: Move to event-driven architecture
- **Containerization**: Docker support for easy deployment
- **Distributed**: Support for distributed robot systems

Conclusion
----------

The LibFrankaPy architecture balances performance, safety, and usability through:

- **Hybrid Design**: Python ease-of-use with C++ performance
- **Real-time Guarantees**: Deterministic control loop execution
- **Safety Integration**: Multiple layers of safety protection
- **Modular Structure**: Easy to extend and maintain
- **Comprehensive Testing**: Robust testing at all levels

This architecture enables researchers and developers to focus on their applications while providing the performance and safety required for professional robotics applications.