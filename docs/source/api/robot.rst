Robot API
=========

This module provides the main robot interface for libfrankapy.

FrankaRobot Class
-----------------

The ``FrankaRobot`` class is the main interface for controlling Franka Emika robots.

Key features:

* Real-time robot control at 1kHz
* Joint and Cartesian space control
* Force/impedance control capabilities
* Comprehensive safety monitoring
* State monitoring and logging
* Error handling and recovery

Robot Control Methods
---------------------

The FrankaRobot class provides the following key methods:

* ``connect()`` - Connect to the robot
* ``disconnect()`` - Disconnect from the robot
* ``get_state()`` - Get current robot state
* ``move_to_joint()`` - Move to joint position
* ``move_to_pose()`` - Move to Cartesian pose
* ``apply_force()`` - Apply force control
* ``stop()`` - Stop robot motion
* ``reset_errors()`` - Reset robot errors

Examples
--------

Basic robot control::

    from libfrankapy import FrankaRobot
    
    robot = FrankaRobot()
    robot.connect()
    
    # Get current state
    state = robot.get_state()
    print(f"Current position: {state.position}")
    
    # Move to joint position
    joint_angles = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
    robot.move_to_joint(joint_angles)
    
    robot.disconnect()