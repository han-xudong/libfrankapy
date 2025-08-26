#!/usr/bin/env python3
"""Basic control example for LibFrankaPy.

This example demonstrates basic robot connection, state reading,
and simple motion commands.
"""

import time
import numpy as np
import libfrankapy as fp


def main():
    """Main example function."""
    
    # Robot IP address (change this to your robot's IP)
    robot_ip = "192.168.1.100"
    
    print(f"LibFrankaPy Basic Control Example")
    print(f"Connecting to robot at {robot_ip}...")
    
    try:
        # Create robot instance
        robot = fp.FrankaRobot(robot_ip)
        
        # Connect to robot
        if not robot.connect():
            print("Failed to connect to robot")
            return
        
        print("Successfully connected to robot")
        
        # Start control loop
        if not robot.start_control():
            print("Failed to start control loop")
            return
        
        print("Control loop started")
        
        # Read initial robot state
        print("\n=== Initial Robot State ===")
        state = robot.get_robot_state()
        print(f"Robot mode: {state.robot_mode}")
        print(f"Control frequency: {state.control_frequency:.1f} Hz")
        print(f"Joint positions: {np.array(state.joint_state.positions):.3f}")
        print(f"End-effector position: {np.array(state.cartesian_pose.position):.3f}")
        
        # Check if robot is ready
        if robot.has_error():
            print(f"Robot has error (code: {robot.get_error_code()})")
            return
        
        print("\n=== Moving to Home Position ===")
        # Move to home position
        robot.move_to_home(speed_factor=0.1)
        print("Moved to home position")
        
        # Wait a moment
        time.sleep(1.0)
        
        print("\n=== Joint Space Motion ===")
        # Define target joint positions (in radians)
        target_joints = [0.0, -0.3, 0.0, -2.2, 0.0, 1.9, 0.785]
        
        print(f"Moving to joint configuration: {np.array(target_joints):.3f}")
        robot.move_to_joint(target_joints, speed_factor=0.1)
        print("Joint motion completed")
        
        # Read current state
        current_state = robot.get_joint_state()
        print(f"Current joint positions: {np.array(current_state.positions):.3f}")
        
        # Wait a moment
        time.sleep(1.0)
        
        print("\n=== Cartesian Space Motion ===")
        # Get current pose
        current_pose = robot.get_cartesian_pose()
        print(f"Current position: {np.array(current_pose.position):.3f}")
        print(f"Current orientation: {np.array(current_pose.orientation):.3f}")
        
        # Move 10cm up in Z direction
        target_position = current_pose.position.copy()
        target_position[2] += 0.1  # Move up 10cm
        target_pose = target_position + current_pose.orientation
        
        print(f"Moving to position: {np.array(target_position):.3f}")
        robot.move_to_pose(target_pose, speed_factor=0.1)
        print("Cartesian motion completed")
        
        # Read final state
        final_pose = robot.get_cartesian_pose()
        print(f"Final position: {np.array(final_pose.position):.3f}")
        
        # Wait a moment
        time.sleep(1.0)
        
        print("\n=== Returning to Home ===")
        # Return to home position
        robot.move_to_home(speed_factor=0.1)
        print("Returned to home position")
        
        print("\n=== Example Completed Successfully ===")
        
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