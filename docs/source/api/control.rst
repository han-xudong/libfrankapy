Control API
===========

This module provides control interfaces for libfrankapy.

Controller Classes
------------------

JointController
~~~~~~~~~~~~~~~

The ``JointController`` class provides joint-space control functionality.

Key methods:

* ``move_to_joint()`` - Move to target joint positions
* ``set_joint_velocities()`` - Set joint velocity commands
* ``get_joint_limits()`` - Get joint position/velocity limits
* ``validate_joint_command()`` - Validate joint commands

CartesianController
~~~~~~~~~~~~~~~~~~~

The ``CartesianController`` class provides Cartesian space control.

Key methods:

* ``move_to_pose()`` - Move to target Cartesian pose
* ``set_cartesian_velocity()`` - Set Cartesian velocity commands
* ``get_workspace_limits()`` - Get workspace boundaries
* ``validate_pose()`` - Validate Cartesian poses

ForceController
~~~~~~~~~~~~~~~

The ``ForceController`` class provides force/impedance control.

Key methods:

* ``set_force_target()`` - Set target force/torque
* ``set_impedance_parameters()`` - Configure impedance
* ``enable_force_control()`` - Enable force control mode
* ``get_contact_force()`` - Get measured contact forces

TrajectoryController
~~~~~~~~~~~~~~~~~~~~

The ``TrajectoryController`` class provides trajectory execution.

Key methods:

* ``execute_trajectory()`` - Execute pre-planned trajectory
* ``generate_trajectory()`` - Generate smooth trajectories
* ``set_trajectory_parameters()`` - Configure trajectory settings
* ``get_trajectory_progress()`` - Get execution progress

Control Functions
-----------------

The control module provides various utility functions:

* ``generate_joint_trajectory()`` - Generate smooth joint trajectories
* ``generate_cartesian_trajectory()`` - Generate Cartesian space trajectories
* ``interpolate_trajectory()`` - Interpolate between trajectory points
* ``validate_joint_limits()`` - Validate joint angle limits
* ``validate_cartesian_limits()`` - Validate Cartesian workspace limits

Examples
--------

Joint space control::

    from libfrankapy.control import JointController
    
    controller = JointController()
    target_joints = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
    controller.move_to_joint(target_joints, duration=5.0)

Cartesian space control::

    from libfrankapy.control import CartesianController
    import numpy as np
    
    controller = CartesianController()
    target_pose = np.array([0.5, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0])  # [x,y,z,qw,qx,qy,qz]
    controller.move_to_pose(target_pose, duration=3.0)