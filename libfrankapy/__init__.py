"""LibFrankaPy - Python bindings for libfranka with real-time control.

This package provides high-level Python interfaces for controlling Franka robots
while maintaining real-time performance through a hybrid C++/Python architecture.
"""

from .control import RealtimeConfig, SafetyLimits, Trajectory
from .exceptions import (
    ConnectionError,
    ControlError,
    FrankaException,
    SafetyError,
    TimeoutError,
)
from .robot import FrankaRobot
from .state import CartesianPose, JointState, RobotState

__version__ = "0.1.0"
__author__ = "LibFrankaPy Team"
__email__ = "support@libfrankapy.org"
__description__ = "Python bindings for libfranka with real-time control"

__all__ = [
    # Core classes
    "FrankaRobot",
    # State classes
    "JointState",
    "CartesianPose",
    "RobotState",
    # Control classes
    "Trajectory",
    "RealtimeConfig",
    "SafetyLimits",
    # Exceptions
    "FrankaException",
    "ConnectionError",
    "ControlError",
    "SafetyError",
    "TimeoutError",
    # Package info
    "__version__",
    "__author__",
    "__email__",
    "__description__",
]
