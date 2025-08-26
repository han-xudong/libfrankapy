Configuration
=============

This section covers various configuration options for libfrankapy.

Robot Configuration
-------------------

Connection Settings
^^^^^^^^^^^^^^^^^^^

Basic connection configuration:

.. code-block:: python

   import libfrankapy as fp
   
   # Basic configuration
   robot = fp.FrankaRobot(
       ip_address="192.168.1.100",
       port=1000,  # Default Franka port
       timeout=5.0  # Connection timeout in seconds
   )

Advanced Connection Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Advanced configuration
   config = fp.RobotConfig(
       ip_address="192.168.1.100",
       port=1000,
       timeout=5.0,
       retry_attempts=3,
       retry_delay=1.0,
       use_realtime=True,
       realtime_priority=80
   )
   
   robot = fp.FrankaRobot(config)

Control Parameters
------------------

Joint Control Settings
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Joint control configuration
   joint_config = fp.JointControlConfig(
       max_velocity=[2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5],  # rad/s
       max_acceleration=[15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0],  # rad/s²
       max_jerk=[7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0],  # rad/s³
       position_tolerance=0.01,  # rad
       velocity_tolerance=0.1   # rad/s
   )
   
   robot.set_joint_control_config(joint_config)

Cartesian Control Settings
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Cartesian control configuration
   cartesian_config = fp.CartesianControlConfig(
       max_translation_velocity=0.2,  # m/s
       max_rotation_velocity=0.5,     # rad/s
       max_translation_acceleration=1.0,  # m/s²
       max_rotation_acceleration=2.0,     # rad/s²
       max_translation_jerk=5.0,      # m/s³
       max_rotation_jerk=10.0,        # rad/s³
       position_tolerance=0.001,      # m
       orientation_tolerance=0.01     # rad
   )
   
   robot.set_cartesian_control_config(cartesian_config)

Safety Configuration
--------------------

Force and Torque Limits
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Safety limits configuration
   safety_config = fp.SafetyConfig(
       # Force limits (N)
       max_force_x=20.0,
       max_force_y=20.0,
       max_force_z=20.0,
       
       # Torque limits (Nm)
       max_torque_x=10.0,
       max_torque_y=10.0,
       max_torque_z=10.0,
       
       # Joint torque limits (Nm)
       max_joint_torques=[87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0],
       
       # Collision detection
       collision_detection_enabled=True,
       collision_force_threshold=10.0,
       collision_torque_threshold=5.0
   )
   
   robot.set_safety_config(safety_config)

Workspace Limits
^^^^^^^^^^^^^^^^

.. code-block:: python

   # Workspace limits
   workspace_config = fp.WorkspaceConfig(
       # Cartesian limits (m)
       x_min=-0.8, x_max=0.8,
       y_min=-0.8, y_max=0.8,
       z_min=0.1, z_max=1.2,
       
       # Joint limits (rad) - usually from robot specifications
       joint_limits_lower=[-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
       joint_limits_upper=[2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
   )
   
   robot.set_workspace_config(workspace_config)

Real-time Configuration
-----------------------

Real-time Settings
^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Real-time configuration
   rt_config = fp.RealTimeConfig(
       control_frequency=1000,  # Hz
       communication_frequency=1000,  # Hz
       use_realtime_kernel=True,
       realtime_priority=80,
       cpu_affinity=[2, 3],  # Bind to specific CPU cores
       memory_lock=True,     # Lock memory to prevent swapping
       stack_size=8192       # Stack size for real-time thread
   )
   
   robot.set_realtime_config(rt_config)

Logging Configuration
---------------------

Logging Settings
^^^^^^^^^^^^^^^^

.. code-block:: python

   import logging
   
   # Configure logging
   logging_config = fp.LoggingConfig(
       level=logging.INFO,
       log_to_file=True,
       log_file_path="/tmp/libfrankapy.log",
       max_file_size=10*1024*1024,  # 10MB
       backup_count=5,
       log_robot_state=True,
       state_log_frequency=100,  # Hz
       log_commands=True,
       log_errors=True
   )
   
   robot.set_logging_config(logging_config)

Data Recording Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Data recording configuration
   recording_config = fp.RecordingConfig(
       enabled=True,
       output_directory="/tmp/robot_data",
       file_format="csv",  # or "hdf5", "json"
       recording_frequency=1000,  # Hz
       buffer_size=10000,  # Number of samples to buffer
       
       # What to record
       record_joint_states=True,
       record_cartesian_states=True,
       record_forces=True,
       record_commands=True,
       record_errors=True
   )
   
   robot.set_recording_config(recording_config)

Environment Variables
---------------------

libfrankapy supports several environment variables for configuration:

.. code-block:: bash

   # Robot connection
   export FRANKA_ROBOT_IP="192.168.1.100"
   export FRANKA_ROBOT_PORT="1000"
   
   # Real-time settings
   export FRANKA_USE_REALTIME="true"
   export FRANKA_RT_PRIORITY="80"
   export FRANKA_CPU_AFFINITY="2,3"
   
   # Logging
   export FRANKA_LOG_LEVEL="INFO"
   export FRANKA_LOG_FILE="/tmp/libfrankapy.log"
   
   # Safety
   export FRANKA_ENABLE_COLLISION_DETECTION="true"
   export FRANKA_MAX_FORCE="20.0"
   
   # Paths
   export CMAKE_PREFIX_PATH="/opt/libfranka:$CMAKE_PREFIX_PATH"
   export LD_LIBRARY_PATH="/opt/libfranka/lib:$LD_LIBRARY_PATH"

Configuration Files
-------------------

YAML Configuration
^^^^^^^^^^^^^^^^^^

You can also use YAML files for configuration:

.. code-block:: yaml

   # robot_config.yaml
   robot:
     ip_address: "192.168.1.100"
     port: 1000
     timeout: 5.0
     use_realtime: true
   
   control:
     joint:
       max_velocity: [2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5]
       max_acceleration: [15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0]
       position_tolerance: 0.01
     
     cartesian:
       max_translation_velocity: 0.2
       max_rotation_velocity: 0.5
       position_tolerance: 0.001
   
   safety:
     max_force_x: 20.0
     max_force_y: 20.0
     max_force_z: 20.0
     collision_detection_enabled: true
   
   logging:
     level: "INFO"
     log_to_file: true
     log_file_path: "/tmp/libfrankapy.log"

Loading YAML Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   import libfrankapy as fp
   
   # Load configuration from YAML file
   robot = fp.FrankaRobot.from_config_file("robot_config.yaml")
   
   # Or load specific sections
   config = fp.load_config("robot_config.yaml")
   robot = fp.FrankaRobot(config.robot)
   robot.set_joint_control_config(config.control.joint)
   robot.set_cartesian_control_config(config.control.cartesian)
   robot.set_safety_config(config.safety)

Validation and Defaults
-----------------------

Configuration Validation
^^^^^^^^^^^^^^^^^^^^^^^^^

libfrankapy automatically validates configuration parameters:

.. code-block:: python

   try:
       # This will raise an exception if invalid
       config = fp.JointControlConfig(
           max_velocity=[10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]  # Too high!
       )
   except fp.ConfigurationError as e:
       print(f"Configuration error: {e}")

Default Configurations
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Get default configurations
   default_joint_config = fp.JointControlConfig.default()
   default_safety_config = fp.SafetyConfig.default()
   default_rt_config = fp.RealTimeConfig.default()
   
   # Modify only what you need
   custom_config = default_joint_config.copy()
   custom_config.max_velocity[0] = 1.5  # Reduce velocity for joint 1

Best Practices
--------------

1. **Start with defaults**: Use default configurations and modify only what's necessary
2. **Validate early**: Test configurations in a safe environment first
3. **Use environment variables**: For deployment-specific settings
4. **Version control**: Keep configuration files in version control
5. **Document changes**: Comment why specific values were chosen
6. **Test thoroughly**: Validate configurations with actual robot hardware

.. warning::
   Always test configuration changes in a safe environment before deploying to production systems.