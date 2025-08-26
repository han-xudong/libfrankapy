State API
=========

This module provides state management and monitoring for libfrankapy.

State Classes
-------------

RobotState
~~~~~~~~~~

The ``RobotState`` class represents the complete state of the robot.

Key attributes:

* ``joint_positions`` - Current joint angles
* ``joint_velocities`` - Current joint velocities
* ``cartesian_pose`` - End-effector pose
* ``external_force`` - External forces/torques
* ``timestamp`` - State timestamp

JointState
~~~~~~~~~~

The ``JointState`` class represents joint-space state information.

Key attributes:

* ``positions`` - Joint positions [rad]
* ``velocities`` - Joint velocities [rad/s]
* ``accelerations`` - Joint accelerations [rad/s²]
* ``torques`` - Joint torques [Nm]

CartesianState
~~~~~~~~~~~~~~

The ``CartesianState`` class represents Cartesian space state information.

Key attributes:

* ``position`` - End-effector position [m]
* ``orientation`` - End-effector orientation
* ``linear_velocity`` - Linear velocity [m/s]
* ``angular_velocity`` - Angular velocity [rad/s]

ForceState
~~~~~~~~~~

The ``ForceState`` class represents force/torque state information.

Key attributes:

* ``force`` - Applied forces [N]
* ``torque`` - Applied torques [Nm]
* ``contact_force`` - Contact forces
* ``external_wrench`` - External wrench

Utility Classes
---------------

Pose
~~~~

The ``Pose`` class represents a 6D pose (position and orientation) in 3D space.

Key attributes:

* ``position`` - 3D position vector [x, y, z]
* ``orientation`` - Quaternion [w, x, y, z] or rotation matrix
* ``matrix`` - 4x4 transformation matrix representation

Transform
~~~~~~~~~

The ``Transform`` class represents spatial transformations between coordinate frames.

Key methods:

* ``from_translation()`` - Create transform from translation vector
* ``from_rotation()`` - Create transform from rotation
* ``apply()`` - Apply transform to pose or point
* ``inverse()`` - Get inverse transformation

State Functions
---------------

The state module provides various utility functions for working with robot states:

* ``get_current_state()`` - Get the current robot state
* ``monitor_state()`` - Monitor state changes with callbacks
* ``log_state()`` - Log state data for analysis
* ``compare_states()`` - Compare two robot states
* ``interpolate_poses()`` - Interpolate between poses

Examples
--------

State monitoring::

    from libfrankapy.state import RobotState, monitor_state
    
    # Get current robot state
    state = RobotState.get_current()
    print(f"Joint positions: {state.joint_positions}")
    print(f"End-effector pose: {state.cartesian_pose}")
    
    # Monitor state changes
    def state_callback(state):
        print(f"Force: {state.external_force}")
    
    monitor_state(state_callback, frequency=100)

Pose manipulation::

    from libfrankapy.state import Pose, Transform
    import numpy as np
    
    # Create pose
    pose = Pose(position=[0.5, 0.0, 0.5], orientation=[1.0, 0.0, 0.0, 0.0])
    
    # Apply transformation
    transform = Transform.from_translation([0.1, 0.0, 0.0])
    new_pose = transform.apply(pose)