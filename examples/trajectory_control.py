#!/usr/bin/env python3
"""Trajectory control example for LibFrankaPy.

This example demonstrates how to create and execute trajectories
for both joint space and Cartesian space motion.
"""

import time
import numpy as np
import libfrankapy as fp
from libfrankapy.control import Trajectory, TrajectoryPoint, ControlMode, MotionType


def create_joint_trajectory():
    """Create a simple joint space trajectory."""
    
    # Define waypoints in joint space
    waypoints = [
        # Home position
        [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
        # Intermediate position 1
        [0.5, -0.5, 0.0, -2.0, 0.0, 1.5, 0.785],
        # Intermediate position 2
        [-0.5, -0.5, 0.0, -2.0, 0.0, 1.5, 0.785],
        # Back to home
        [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
    ]
    
    # Create trajectory points
    points = []
    for i, positions in enumerate(waypoints):
        point = TrajectoryPoint(
            positions=positions,
            time_from_start=float(i * 3.0)  # 3 seconds between waypoints
        )
        points.append(point)
    
    # Create trajectory
    trajectory = Trajectory(
        points=points,
        control_mode=ControlMode.JOINT_POSITION,
        speed_factor=0.2,
        acceleration_factor=0.2
    )
    
    return trajectory


def create_cartesian_trajectory(start_pose):
    """Create a simple Cartesian space trajectory.
    
    Args:
        start_pose: Starting pose [x, y, z, qx, qy, qz, qw]
    """
    
    # Extract position and orientation
    start_pos = start_pose[:3]
    orientation = start_pose[3:]  # Keep orientation constant
    
    # Define a square trajectory in the XY plane
    square_size = 0.1  # 10cm square
    
    waypoints = [
        # Start position
        start_pos + orientation,
        # Move right
        [start_pos[0] + square_size, start_pos[1], start_pos[2]] + orientation,
        # Move forward
        [start_pos[0] + square_size, start_pos[1] + square_size, start_pos[2]] + orientation,
        # Move left
        [start_pos[0], start_pos[1] + square_size, start_pos[2]] + orientation,
        # Return to start
        start_pos + orientation,
    ]
    
    # Create trajectory points
    points = []
    for i, pose in enumerate(waypoints):
        point = TrajectoryPoint(
            positions=pose,
            time_from_start=float(i * 2.0)  # 2 seconds between waypoints
        )
        points.append(point)
    
    # Create trajectory
    trajectory = Trajectory(
        points=points,
        control_mode=ControlMode.CARTESIAN_POSITION,
        motion_type=MotionType.LINEAR,
        speed_factor=0.1
    )
    
    return trajectory


def trajectory_callback(point_index, trajectory_point):
    """Callback function called during trajectory execution.
    
    Args:
        point_index: Index of current trajectory point
        trajectory_point: Current trajectory point being executed
    """
    print(f"Executing trajectory point {point_index + 1}")
    print(f"  Target positions: {np.array(trajectory_point.positions):.3f}")
    print(f"  Time from start: {trajectory_point.time_from_start:.1f}s")


def main():
    """Main example function."""
    
    # Robot IP address (change this to your robot's IP)
    robot_ip = "192.168.1.100"
    
    print(f"LibFrankaPy Trajectory Control Example")
    print(f"Connecting to robot at {robot_ip}...")
    
    try:
        # Create robot instance with custom configuration
        config = fp.RealtimeConfig(
            control_frequency=1000,  # 1kHz control loop
            filter_cutoff=100.0,     # 100Hz filter
        )
        
        robot = fp.FrankaRobot(robot_ip, realtime_config=config)
        
        # Connect and start control
        if not robot.connect():
            print("Failed to connect to robot")
            return
        
        if not robot.start_control():
            print("Failed to start control loop")
            return
        
        print("Robot connected and control started")
        
        # Move to home position first
        print("\n=== Moving to Home Position ===")
        robot.move_to_home(speed_factor=0.1)
        print("At home position")
        
        time.sleep(1.0)
        
        # Execute joint space trajectory
        print("\n=== Joint Space Trajectory ===")
        joint_trajectory = create_joint_trajectory()
        
        print(f"Executing joint trajectory with {len(joint_trajectory.points)} waypoints")
        print(f"Total duration: {joint_trajectory.duration:.1f} seconds")
        
        start_time = time.time()
        robot.execute_trajectory(joint_trajectory, callback=trajectory_callback)
        execution_time = time.time() - start_time
        
        print(f"Joint trajectory completed in {execution_time:.1f} seconds")
        
        time.sleep(2.0)
        
        # Execute Cartesian space trajectory
        print("\n=== Cartesian Space Trajectory ===")
        
        # Get current pose as starting point
        current_pose = robot.get_cartesian_pose()
        start_pose = current_pose.position + current_pose.orientation
        
        cartesian_trajectory = create_cartesian_trajectory(start_pose)
        
        print(f"Executing Cartesian trajectory with {len(cartesian_trajectory.points)} waypoints")
        print(f"Total duration: {cartesian_trajectory.duration:.1f} seconds")
        print(f"Starting position: {np.array(current_pose.position):.3f}")
        
        start_time = time.time()
        robot.execute_trajectory(cartesian_trajectory, callback=trajectory_callback)
        execution_time = time.time() - start_time
        
        print(f"Cartesian trajectory completed in {execution_time:.1f} seconds")
        
        # Return to home
        print("\n=== Returning to Home ===")
        robot.move_to_home(speed_factor=0.1)
        print("Returned to home position")
        
        print("\n=== Trajectory Example Completed Successfully ===")
        
    except fp.ConnectionError as e:
        print(f"Connection error: {e}")
    except fp.ControlError as e:
        print(f"Control error: {e}")
    except fp.SafetyError as e:
        print(f"Safety error: {e}")
    except fp.TimeoutError as e:
        print(f"Timeout error: {e}")
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        if 'robot' in locals():
            robot.emergency_stop()
    except Exception as e:
        print(f"Unexpected error: {e}")
        if 'robot' in locals():
            robot.emergency_stop()
    
    finally:
        # Clean up
        if 'robot' in locals():
            print("Disconnecting from robot...")
            robot.disconnect()
            print("Disconnected")


if __name__ == "__main__":
    main()