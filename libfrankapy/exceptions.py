"""Exception classes for LibFrankaPy.

This module defines custom exception classes used throughout the LibFrankaPy package
to provide clear error handling and debugging information.
"""

from typing import Optional


class FrankaException(Exception):
    """Base exception class for all LibFrankaPy errors.
    
    All other exceptions in this package inherit from this base class.
    """
    
    def __init__(self, message: str, error_code: Optional[int] = None):
        super().__init__(message)
        self.message = message
        self.error_code = error_code
    
    def __str__(self) -> str:
        if self.error_code is not None:
            return f"[Error {self.error_code}] {self.message}"
        return self.message


class ConnectionError(FrankaException):
    """Raised when robot connection fails or is lost.
    
    This exception is raised when:
    - Initial connection to robot fails
    - Connection is lost during operation
    - Network communication errors occur
    """
    pass


class ControlError(FrankaException):
    """Raised when robot control operations fail.
    
    This exception is raised when:
    - Motion commands fail to execute
    - Control loop errors occur
    - Invalid control parameters are provided
    """
    pass


class SafetyError(FrankaException):
    """Raised when safety limits are violated.
    
    This exception is raised when:
    - Joint limits are exceeded
    - Force/torque limits are violated
    - Emergency stop is triggered
    - Safety monitoring detects violations
    """
    pass


class TimeoutError(FrankaException):
    """Raised when operations timeout.
    
    This exception is raised when:
    - Motion commands timeout
    - Communication timeouts occur
    - Real-time deadlines are missed
    """
    pass


class ConfigurationError(FrankaException):
    """Raised when configuration is invalid.
    
    This exception is raised when:
    - Invalid robot IP address
    - Incorrect real-time configuration
    - Missing or invalid parameters
    """
    pass


class StateError(FrankaException):
    """Raised when robot state is invalid for requested operation.
    
    This exception is raised when:
    - Robot is not connected
    - Robot is in error state
    - Operation not allowed in current state
    """
    pass