Testing
=======

This document describes the testing strategy and practices for libfrankapy.

Testing Philosophy
------------------

libfrankapy follows a comprehensive testing approach to ensure reliability, safety, and performance:

- **Safety First**: All safety-critical code must be thoroughly tested
- **Test-Driven Development**: Write tests before implementing features
- **Continuous Integration**: Automated testing on every commit
- **Multiple Test Levels**: Unit, integration, system, and performance tests
- **Real Hardware Testing**: Critical paths tested with actual robot hardware

Test Structure
--------------

Test Organization
^^^^^^^^^^^^^^^^^

.. code-block:: text

   tests/
   ├── unit/                     # Unit tests
   │   ├── python/
   │   │   ├── test_robot.py
   │   │   ├── test_control.py
   │   │   ├── test_state.py
   │   │   └── test_exceptions.py
   │   └── cpp/
   │       ├── test_controller.cpp
   │       ├── test_shared_memory.cpp
   │       └── test_safety.cpp
   ├── integration/              # Integration tests
   │   ├── test_python_cpp.py
   │   ├── test_robot_control.py
   │   └── test_force_control.py
   ├── system/                   # System tests
   │   ├── test_end_to_end.py
   │   ├── test_error_recovery.py
   │   └── test_safety_limits.py
   ├── performance/              # Performance tests
   │   ├── test_latency.py
   │   ├── test_throughput.py
   │   └── benchmarks/
   ├── fixtures/                 # Test data and fixtures
   │   ├── robot_states.json
   │   ├── trajectories.csv
   │   └── mock_data.py
   ├── hardware/                 # Hardware-specific tests
   │   ├── test_real_robot.py
   │   └── test_force_sensor.py
   └── conftest.py              # Pytest configuration

Test Categories
^^^^^^^^^^^^^^^

**Unit Tests**
- Test individual functions and classes
- Fast execution (< 1 second each)
- No external dependencies
- High code coverage (> 90%)

**Integration Tests**
- Test component interactions
- May use mock hardware
- Medium execution time (< 10 seconds each)
- Focus on interfaces

**System Tests**
- End-to-end functionality
- May require real hardware
- Longer execution time (< 60 seconds each)
- Test complete workflows

**Performance Tests**
- Measure latency and throughput
- Benchmark against requirements
- Detect performance regressions
- Generate performance reports

Unit Testing
------------

Python Unit Tests
^^^^^^^^^^^^^^^^^

**Test Structure**

.. code-block:: python

   import pytest
   from unittest.mock import Mock, patch, MagicMock
   import numpy as np
   import libfrankapy as fp
   
   class TestFrankaRobot:
       """Test suite for FrankaRobot class."""
       
       def setup_method(self):
           """Set up test fixtures."""
           self.robot_ip = "192.168.1.100"
           self.robot = fp.FrankaRobot(self.robot_ip)
       
       def teardown_method(self):
           """Clean up after tests."""
           if self.robot.is_connected:
               self.robot.disconnect()
       
       def test_initialization(self):
           """Test robot initialization."""
           assert self.robot.ip_address == self.robot_ip
           assert not self.robot.is_connected
           assert not self.robot.is_control_active
       
       @patch('libfrankapy.robot.RealTimeController')
       def test_connection(self, mock_controller):
           """Test robot connection."""
           # Setup mock
           mock_controller.return_value.connect.return_value = True
           
           # Test connection
           result = self.robot.connect()
           
           # Assertions
           assert result is True
           assert self.robot.is_connected
           mock_controller.return_value.connect.assert_called_once()
       
       def test_joint_validation(self):
           """Test joint position validation."""
           # Valid joint positions
           valid_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
           assert self.robot._validate_joint_positions(valid_joints)
           
           # Invalid joint positions (wrong length)
           invalid_joints = [0.0, -0.785, 0.0]
           with pytest.raises(fp.ConfigurationError):
               self.robot._validate_joint_positions(invalid_joints)
           
           # Invalid joint positions (out of range)
           out_of_range = [5.0, 0, 0, 0, 0, 0, 0]  # Joint 1 limit exceeded
           with pytest.raises(fp.JointLimitError):
               self.robot._validate_joint_positions(out_of_range)

**Parametrized Tests**

.. code-block:: python

   @pytest.mark.parametrize("joint_positions,expected_valid", [
       ([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], True),
       ([2.8, -1.7, 2.8, -0.1, 2.8, 3.7, 2.8], True),  # At limits
       ([3.0, 0, 0, 0, 0, 0, 0], False),  # Exceeds limit
       ([0, 0, 0, 0, 0, 0], False),  # Wrong length
   ])
   def test_joint_validation_parametrized(self, joint_positions, expected_valid):
       """Test joint validation with multiple inputs."""
       if expected_valid:
           assert self.robot._validate_joint_positions(joint_positions)
       else:
           with pytest.raises((fp.JointLimitError, fp.ConfigurationError)):
               self.robot._validate_joint_positions(joint_positions)

**Mock Usage**

.. code-block:: python

   class TestRobotState:
       """Test robot state management."""
       
       @patch('libfrankapy.state.SharedMemoryManager')
       def test_state_retrieval(self, mock_shared_memory):
           """Test robot state retrieval."""
           # Setup mock data
           mock_state = fp.RobotState(
               joint_state=fp.JointState(
                   positions=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
                   velocities=[0.0] * 7,
                   torques=[0.0] * 7
               ),
               cartesian_state=fp.CartesianState(
                   pose=np.eye(4),
                   forces=[0.0] * 6
               )
           )
           
           mock_shared_memory.return_value.robot_state.read.return_value = mock_state
           
           # Test state retrieval
           robot = fp.FrankaRobot("192.168.1.100")
           state = robot.get_robot_state()
           
           # Assertions
           assert isinstance(state, fp.RobotState)
           assert len(state.joint_state.positions) == 7
           assert state.joint_state.positions[0] == 0.1

C++ Unit Tests
^^^^^^^^^^^^^^

**Google Test Framework**

.. code-block:: cpp

   #include <gtest/gtest.h>
   #include <gmock/gmock.h>
   #include "realtime_controller.hpp"
   #include "shared_memory.hpp"
   
   class RealTimeControllerTest : public ::testing::Test {
   protected:
       void SetUp() override {
           controller_ = std::make_unique<RealTimeController>("192.168.1.100");
       }
       
       void TearDown() override {
           if (controller_->is_running()) {
               controller_->stop();
           }
       }
       
       std::unique_ptr<RealTimeController> controller_;
   };
   
   TEST_F(RealTimeControllerTest, Initialization) {
       EXPECT_FALSE(controller_->is_running());
       EXPECT_FALSE(controller_->is_connected());
   }
   
   TEST_F(RealTimeControllerTest, JointCommandValidation) {
       JointCommand valid_cmd;
       valid_cmd.positions = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
       valid_cmd.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
       
       EXPECT_TRUE(controller_->validate_joint_command(valid_cmd));
       
       // Test invalid command
       JointCommand invalid_cmd;
       invalid_cmd.positions = {5.0, 0, 0, 0, 0, 0, 0};  // Exceeds limits
       
       EXPECT_FALSE(controller_->validate_joint_command(invalid_cmd));
   }

**Mock Objects**

.. code-block:: cpp

   class MockFrankaRobot {
   public:
       MOCK_METHOD(franka::RobotState, readOnce, (), (const));
       MOCK_METHOD(void, control, (std::function<franka::Torques(const franka::RobotState&, franka::Duration)>), ());
   };
   
   class SafetyMonitorTest : public ::testing::Test {
   protected:
       void SetUp() override {
           mock_robot_ = std::make_shared<MockFrankaRobot>();
           safety_monitor_ = std::make_unique<SafetyMonitor>(mock_robot_);
       }
       
       std::shared_ptr<MockFrankaRobot> mock_robot_;
       std::unique_ptr<SafetyMonitor> safety_monitor_;
   };
   
   TEST_F(SafetyMonitorTest, ForceLimit) {
       franka::RobotState state{};
       state.K_F_ext_hat_K = {25.0, 0, 0, 0, 0, 0};  // Exceeds 20N limit
       
       EXPECT_CALL(*mock_robot_, readOnce())
           .WillOnce(::testing::Return(state));
       
       EXPECT_THROW(safety_monitor_->check_force_limits(state), ForceLimitError);
   }

Integration Testing
-------------------

Python-C++ Integration
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   class TestPythonCppIntegration:
       """Test Python-C++ interface integration."""
       
       def test_shared_memory_communication(self):
           """Test shared memory communication between Python and C++."""
           # Create shared memory manager
           shared_mem = fp.SharedMemoryManager()
           
           # Write from Python
           joint_cmd = fp.JointCommand(
               positions=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
               velocities=[0.0] * 7
           )
           shared_mem.write_joint_command(joint_cmd)
           
           # Read from C++ side (simulated)
           read_cmd = shared_mem.read_joint_command()
           
           # Verify data integrity
           assert np.allclose(read_cmd.positions, joint_cmd.positions)
           assert np.allclose(read_cmd.velocities, joint_cmd.velocities)
       
       def test_real_time_performance(self):
           """Test real-time performance requirements."""
           controller = fp.RealTimeController("192.168.1.100")
           
           # Measure control loop timing
           timings = []
           for _ in range(1000):
               start = time.perf_counter()
               controller.update_once()
               end = time.perf_counter()
               timings.append(end - start)
           
           # Verify timing requirements
           max_time = max(timings)
           avg_time = sum(timings) / len(timings)
           
           assert max_time < 0.001  # Must complete within 1ms
           assert avg_time < 0.0005  # Average should be < 0.5ms

Robot Control Integration
^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   @pytest.mark.integration
   @pytest.mark.requires_robot
   class TestRobotControlIntegration:
       """Integration tests requiring robot hardware."""
       
       @pytest.fixture(scope="class")
       def robot(self):
           """Robot fixture for integration tests."""
           robot = fp.FrankaRobot("192.168.1.100")
           robot.connect()
           robot.start_control()
           yield robot
           robot.stop_control()
           robot.disconnect()
       
       def test_joint_movement_accuracy(self, robot):
           """Test joint movement accuracy."""
           target_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
           
           # Move to target
           success = robot.move_to_joint(target_positions, speed_factor=0.1)
           assert success
           
           # Check final position
           final_state = robot.get_robot_state()
           final_positions = final_state.joint_state.positions
           
           # Verify accuracy (within 0.01 rad)
           for target, actual in zip(target_positions, final_positions):
               assert abs(target - actual) < 0.01
       
       def test_cartesian_movement_accuracy(self, robot):
           """Test Cartesian movement accuracy."""
           # Get current pose
           current_state = robot.get_cartesian_state()
           start_pose = current_state.pose.copy()
           
           # Move 10cm in Z direction
           target_pose = start_pose.copy()
           target_pose[2, 3] += 0.1
           
           # Execute movement
           success = robot.move_to_pose(target_pose, speed_factor=0.1)
           assert success
           
           # Check final pose
           final_state = robot.get_cartesian_state()
           final_pose = final_state.pose
           
           # Verify position accuracy (within 1mm)
           position_error = np.linalg.norm(final_pose[:3, 3] - target_pose[:3, 3])
           assert position_error < 0.001

System Testing
--------------

End-to-End Tests
^^^^^^^^^^^^^^^^

.. code-block:: python

   @pytest.mark.system
   class TestEndToEndScenarios:
       """End-to-end system tests."""
       
       def test_complete_pick_and_place(self):
           """Test complete pick and place operation."""
           robot = fp.FrankaRobot("192.168.1.100")
           
           try:
               # Connect and initialize
               robot.connect()
               robot.start_control()
               
               # Move to approach position
               approach_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
               robot.move_to_joint(approach_joints)
               
               # Move to pick position
               pick_pose = robot.get_cartesian_state().pose.copy()
               pick_pose[2, 3] -= 0.1  # Move down 10cm
               robot.move_to_pose(pick_pose, speed_factor=0.05)
               
               # Simulate gripper close
               time.sleep(1.0)
               
               # Move to place position
               place_pose = pick_pose.copy()
               place_pose[0, 3] += 0.2  # Move 20cm in X
               place_pose[2, 3] += 0.1  # Move back up
               robot.move_to_pose(place_pose, speed_factor=0.1)
               
               # Move down to place
               place_pose[2, 3] -= 0.1
               robot.move_to_pose(place_pose, speed_factor=0.05)
               
               # Simulate gripper open
               time.sleep(1.0)
               
               # Return to home
               robot.move_to_joint(approach_joints)
               
               # Verify successful completion
               final_state = robot.get_robot_state()
               assert final_state.system_state.robot_mode == fp.RobotMode.IDLE
               
           finally:
               robot.stop_control()
               robot.disconnect()

Error Recovery Tests
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   class TestErrorRecovery:
       """Test error recovery mechanisms."""
       
       def test_joint_limit_recovery(self):
           """Test recovery from joint limit violation."""
           robot = fp.FrankaRobot("192.168.1.100")
           
           try:
               robot.connect()
               robot.start_control()
               
               # Attempt to exceed joint limits
               invalid_joints = [5.0, 0, 0, 0, 0, 0, 0]  # Exceeds joint 1 limit
               
               with pytest.raises(fp.JointLimitError):
                   robot.move_to_joint(invalid_joints)
               
               # Verify robot is in error state
               assert robot.is_in_error_state()
               
               # Attempt recovery
               recovery_success = robot.recover_from_errors()
               assert recovery_success
               
               # Verify robot is operational
               assert not robot.is_in_error_state()
               
               # Test normal operation after recovery
               safe_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
               success = robot.move_to_joint(safe_joints)
               assert success
               
           finally:
               robot.stop_control()
               robot.disconnect()
       
       def test_collision_recovery(self):
           """Test recovery from collision detection."""
           robot = fp.FrankaRobot("192.168.1.100")
           
           try:
               robot.connect()
               robot.start_control()
               
               # Simulate collision by applying high force
               robot.start_force_control()
               high_force = [0, 0, -50.0, 0, 0, 0]  # 50N downward
               
               with pytest.raises(fp.CollisionError):
                   robot.set_cartesian_force(high_force)
               
               # Robot should automatically stop
               assert robot.get_robot_state().system_state.safety_state == fp.SafetyState.PROTECTIVE_STOP
               
               # Recover from collision
               robot.recover_from_errors()
               
               # Verify normal operation
               state = robot.get_robot_state()
               assert state.system_state.safety_state == fp.SafetyState.NORMAL
               
           finally:
               robot.stop_force_control()
               robot.stop_control()
               robot.disconnect()

Performance Testing
-------------------

Latency Benchmarks
^^^^^^^^^^^^^^^^^^

.. code-block:: python

   import time
   import statistics
   import pytest
   
   class TestPerformance:
       """Performance benchmarks and tests."""
       
       def test_state_query_latency(self):
           """Benchmark robot state query latency."""
           robot = fp.FrankaRobot("192.168.1.100")
           robot.connect()
           robot.start_control()
           
           try:
               latencies = []
               
               # Warm up
               for _ in range(100):
                   robot.get_robot_state()
               
               # Measure latencies
               for _ in range(1000):
                   start = time.perf_counter()
                   robot.get_robot_state()
                   end = time.perf_counter()
                   latencies.append((end - start) * 1000)  # Convert to ms
               
               # Analyze results
               avg_latency = statistics.mean(latencies)
               max_latency = max(latencies)
               p95_latency = statistics.quantiles(latencies, n=20)[18]  # 95th percentile
               
               # Performance requirements
               assert avg_latency < 0.1  # Average < 0.1ms
               assert max_latency < 1.0  # Max < 1ms
               assert p95_latency < 0.5  # 95th percentile < 0.5ms
               
               print(f"State query performance:")
               print(f"  Average: {avg_latency:.3f}ms")
               print(f"  Maximum: {max_latency:.3f}ms")
               print(f"  95th percentile: {p95_latency:.3f}ms")
               
           finally:
               robot.stop_control()
               robot.disconnect()
       
       def test_command_throughput(self):
           """Benchmark command throughput."""
           robot = fp.FrankaRobot("192.168.1.100")
           robot.connect()
           robot.start_control()
           
           try:
               # Generate test commands
               commands = []
               for i in range(1000):
                   angle = i * 0.001  # Small incremental changes
                   cmd = [angle, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
                   commands.append(cmd)
               
               # Measure throughput
               start_time = time.perf_counter()
               
               for cmd in commands:
                   robot.set_joint_position(cmd)  # Non-blocking command
               
               end_time = time.perf_counter()
               
               # Calculate throughput
               duration = end_time - start_time
               throughput = len(commands) / duration
               
               # Performance requirement: > 500 commands/second
               assert throughput > 500
               
               print(f"Command throughput: {throughput:.1f} commands/second")
               
           finally:
               robot.stop_control()
               robot.disconnect()

Memory and Resource Tests
^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   import psutil
   import gc
   
   class TestResourceUsage:
       """Test memory and resource usage."""
       
       def test_memory_leaks(self):
           """Test for memory leaks during extended operation."""
           robot = fp.FrankaRobot("192.168.1.100")
           
           # Measure initial memory
           process = psutil.Process()
           initial_memory = process.memory_info().rss
           
           try:
               robot.connect()
               robot.start_control()
               
               # Perform many operations
               for i in range(1000):
                   # Get state
                   state = robot.get_robot_state()
                   
                   # Send command
                   angle = i * 0.001
                   cmd = [angle, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
                   robot.set_joint_position(cmd)
                   
                   # Force garbage collection every 100 iterations
                   if i % 100 == 0:
                       gc.collect()
               
               # Measure final memory
               gc.collect()  # Force final garbage collection
               final_memory = process.memory_info().rss
               
               # Check for memory leaks (allow 10MB growth)
               memory_growth = final_memory - initial_memory
               assert memory_growth < 10 * 1024 * 1024  # 10MB
               
               print(f"Memory growth: {memory_growth / 1024 / 1024:.1f}MB")
               
           finally:
               robot.stop_control()
               robot.disconnect()
       
       def test_cpu_usage(self):
           """Test CPU usage during normal operation."""
           robot = fp.FrankaRobot("192.168.1.100")
           robot.connect()
           robot.start_control()
           
           try:
               # Monitor CPU usage
               cpu_samples = []
               
               for _ in range(100):
                   start_cpu = psutil.cpu_percent()
                   
                   # Perform operations
                   for _ in range(10):
                       robot.get_robot_state()
                       robot.set_joint_position([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
                   
                   end_cpu = psutil.cpu_percent()
                   cpu_samples.append(end_cpu)
                   
                   time.sleep(0.01)  # 100Hz
               
               # Analyze CPU usage
               avg_cpu = statistics.mean(cpu_samples)
               max_cpu = max(cpu_samples)
               
               # CPU usage should be reasonable (< 50% average, < 80% peak)
               assert avg_cpu < 50.0
               assert max_cpu < 80.0
               
               print(f"CPU usage - Average: {avg_cpu:.1f}%, Peak: {max_cpu:.1f}%")
               
           finally:
               robot.stop_control()
               robot.disconnect()

Test Configuration
------------------

Pytest Configuration
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # conftest.py
   import pytest
   import os
   
   def pytest_configure(config):
       """Configure pytest markers and options."""
       config.addinivalue_line(
           "markers", "unit: mark test as unit test"
       )
       config.addinivalue_line(
           "markers", "integration: mark test as integration test"
       )
       config.addinivalue_line(
           "markers", "system: mark test as system test"
       )
       config.addinivalue_line(
           "markers", "performance: mark test as performance test"
       )
       config.addinivalue_line(
           "markers", "requires_robot: mark test as requiring real robot hardware"
       )
       config.addinivalue_line(
           "markers", "slow: mark test as slow running"
       )
   
   def pytest_collection_modifyitems(config, items):
       """Modify test collection based on command line options."""
       if config.getoption("--no-robot"):
           skip_robot = pytest.mark.skip(reason="--no-robot option given")
           for item in items:
               if "requires_robot" in item.keywords:
                   item.add_marker(skip_robot)
   
   def pytest_addoption(parser):
       """Add custom command line options."""
       parser.addoption(
           "--no-robot",
           action="store_true",
           default=False,
           help="skip tests that require robot hardware"
       )
       parser.addoption(
           "--robot-ip",
           action="store",
           default="192.168.1.100",
           help="IP address of test robot"
       )
   
   @pytest.fixture(scope="session")
   def robot_ip(request):
       """Provide robot IP address for tests."""
       return request.config.getoption("--robot-ip")
   
   @pytest.fixture
   def mock_robot():
       """Provide mock robot for testing."""
       from tests.fixtures.mock_data import MockRobot
       return MockRobot()

Test Data and Fixtures
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # tests/fixtures/mock_data.py
   import numpy as np
   import libfrankapy as fp
   
   class MockRobot:
       """Mock robot for testing without hardware."""
       
       def __init__(self):
           self.is_connected = False
           self.is_control_active = False
           self.current_state = self._create_default_state()
           self.command_history = []
       
       def _create_default_state(self):
           """Create default robot state."""
           return fp.RobotState(
               joint_state=fp.JointState(
                   positions=[0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
                   velocities=[0.0] * 7,
                   torques=[0.0] * 7
               ),
               cartesian_state=fp.CartesianState(
                   pose=np.eye(4),
                   forces=[0.0] * 6
               ),
               system_state=fp.SystemState(
                   control_mode=fp.ControlMode.IDLE,
                   robot_mode=fp.RobotMode.IDLE,
                   safety_state=fp.SafetyState.NORMAL
               )
           )
       
       def connect(self):
           self.is_connected = True
           return True
       
       def disconnect(self):
           self.is_connected = False
       
       def start_control(self):
           if not self.is_connected:
               raise fp.ConnectionError("Robot not connected")
           self.is_control_active = True
       
       def stop_control(self):
           self.is_control_active = False
       
       def move_to_joint(self, positions, speed_factor=0.1):
           if not self.is_control_active:
               raise fp.ControlError("Control not active")
           
           self.command_history.append(('move_to_joint', positions, speed_factor))
           
           # Update mock state
           self.current_state.joint_state.positions = positions
           return True
       
       def get_robot_state(self):
           return self.current_state

Continuous Integration
----------------------

GitHub Actions Workflow
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: yaml

   # .github/workflows/test.yml
   name: Tests
   
   on:
     push:
       branches: [ main, develop ]
     pull_request:
       branches: [ main ]
   
   jobs:
     unit-tests:
       runs-on: ubuntu-22.04
       strategy:
         matrix:
           python-version: [3.8, 3.9, "3.10", "3.11"]
       
       steps:
       - uses: actions/checkout@v4
       
       - name: Set up Python ${{ matrix.python-version }}
         uses: actions/setup-python@v4
         with:
           python-version: ${{ matrix.python-version }}
       
       - name: Install system dependencies
         run: |
           sudo apt-get update
           sudo apt-get install -y build-essential cmake libpoco-dev libeigen3-dev
       
       - name: Install Python dependencies
         run: |
           python -m pip install --upgrade pip
           pip install -e ".[dev]"
       
       - name: Run unit tests
         run: |
           pytest tests/unit/ -v --cov=libfrankapy --cov-report=xml
       
       - name: Upload coverage
         uses: codecov/codecov-action@v3
         with:
           file: ./coverage.xml
   
     integration-tests:
       runs-on: ubuntu-22.04
       needs: unit-tests
       
       steps:
       - uses: actions/checkout@v4
       
       - name: Set up Python
         uses: actions/setup-python@v4
         with:
           python-version: "3.10"
       
       - name: Install dependencies
         run: |
           sudo apt-get update
           sudo apt-get install -y build-essential cmake libpoco-dev libeigen3-dev
           python -m pip install --upgrade pip
           pip install -e ".[dev]"
       
       - name: Run integration tests
         run: |
           pytest tests/integration/ -v --no-robot
   
     cpp-tests:
       runs-on: ubuntu-22.04
       
       steps:
       - uses: actions/checkout@v4
       
       - name: Install dependencies
         run: |
           sudo apt-get update
           sudo apt-get install -y build-essential cmake libgtest-dev libgmock-dev
       
       - name: Build and run C++ tests
         run: |
           mkdir build && cd build
           cmake -DBUILD_TESTS=ON ..
           make -j$(nproc)
           ctest --output-on-failure

Test Reporting
^^^^^^^^^^^^^^

.. code-block:: python

   # Generate test reports
   pytest tests/ \
     --cov=libfrankapy \
     --cov-report=html \
     --cov-report=xml \
     --junitxml=test-results.xml \
     --html=test-report.html \
     --self-contained-html

Best Practices
--------------

Test Writing Guidelines
^^^^^^^^^^^^^^^^^^^^^^^

1. **Test Names**: Use descriptive names that explain what is being tested
2. **Test Structure**: Follow Arrange-Act-Assert pattern
3. **Test Independence**: Each test should be independent and isolated
4. **Test Data**: Use fixtures and factories for test data
5. **Assertions**: Use specific assertions with clear error messages

Safety Testing
^^^^^^^^^^^^^^

1. **Safety Limits**: Always test safety limit enforcement
2. **Error Conditions**: Test all error conditions and recovery paths
3. **Emergency Stop**: Verify emergency stop functionality
4. **Graceful Degradation**: Test system behavior under failure conditions

Performance Testing
^^^^^^^^^^^^^^^^^^^

1. **Baseline Measurements**: Establish performance baselines
2. **Regression Detection**: Monitor for performance regressions
3. **Real-world Scenarios**: Test with realistic workloads
4. **Resource Monitoring**: Monitor CPU, memory, and network usage

Conclusion
----------

Comprehensive testing is essential for libfrankapy's reliability and safety. The testing strategy covers:

- **Multiple Test Levels**: Unit, integration, system, and performance tests
- **Safety Focus**: Extensive testing of safety-critical functionality
- **Real Hardware**: Integration with actual robot hardware
- **Continuous Integration**: Automated testing on every change
- **Performance Monitoring**: Ensuring real-time performance requirements

This testing approach ensures that libfrankapy meets the high standards required for professional robotics applications.