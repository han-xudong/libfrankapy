Exceptions API
==============

This module provides exception handling for LibFrankaPy.

Exception Hierarchy
-------------------

Base Exception
~~~~~~~~~~~~~~

``FrankaException``

Base exception class for all LibFrankaPy exceptions.

Connection Exceptions
~~~~~~~~~~~~~~~~~~~~~

``ConnectionError``

Raised when robot connection fails or is lost.

``CommunicationError``

Raised when communication with the robot fails.

Control Exceptions
~~~~~~~~~~~~~~~~~~

``ControlError``

Raised when control commands fail or are invalid.

``SafetyError``

Raised when safety limits are violated.

Configuration Exceptions
~~~~~~~~~~~~~~~~~~~~~~~~~

``ConfigurationError``

Raised when configuration parameters are invalid.

Robot Exceptions
~~~~~~~~~~~~~~~~

``RobotError``

Raised when robot hardware errors occur.

Exception Handling Functions
----------------------------

The exceptions module provides utility functions for error handling:

* ``handle_robot_error()`` - Handle robot-specific errors
* ``log_exception()`` - Log exceptions with context
* ``recover_from_error()`` - Attempt automatic error recovery
* ``is_recoverable_error()`` - Check if an error is recoverable

Examples
--------

Basic exception handling::

    from libfrankapy import FrankaRobot
    from libfrankapy.exceptions import ConnectionError, ControlError, SafetyError
    
    robot = FrankaRobot()
    
    try:
        robot.connect()
        robot.move_to_joint([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
    except ConnectionError as e:
        print(f"Failed to connect to robot: {e}")
    except ControlError as e:
        print(f"Control error occurred: {e}")
        robot.reset_errors()
    except SafetyError as e:
        print(f"Safety violation: {e}")
        robot.stop()
    finally:
        robot.disconnect()

Advanced error recovery::

    from libfrankapy.exceptions import handle_robot_error, is_recoverable_error
    
    try:
        # Robot operation
        robot.move_to_pose(target_pose)
    except Exception as e:
        if is_recoverable_error(e):
            handle_robot_error(robot, e)
            # Retry operation
            robot.move_to_pose(target_pose)
        else:
            raise