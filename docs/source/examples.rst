Examples
========

This section provides comprehensive examples demonstrating various features of libfrankapy.

Basic Control Examples
----------------------

Simple Joint Movement
^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   import libfrankapy as fp
   import time
   
   def simple_joint_movement():
       robot = fp.FrankaRobot("192.168.1.100")
       
       try:
           robot.connect()
           robot.start_control()
           
           # Define some joint positions
           positions = [
               [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],  # Home
               [0.5, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],  # Move joint 1
               [0.0, -0.285, 0.0, -2.356, 0.0, 1.571, 0.785],  # Move joint 2
               [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],  # Back to home
           ]
           
           for i, pos in enumerate(positions):
               print(f"Moving to position {i+1}...")
               robot.move_to_joint(pos, speed_factor=0.2)
               time.sleep(1.0)
               
       finally:
           robot.stop_control()
           robot.disconnect()
   
   if __name__ == "__main__":
       simple_joint_movement()

Cartesian Space Control
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   import libfrankapy as fp
   import numpy as np
   
   def cartesian_movement():
       robot = fp.FrankaRobot("192.168.1.100")
       
       try:
           robot.connect()
           robot.start_control()
           
           # Get current pose
           current_pose = robot.get_robot_state().cartesian_state.pose
           print(f"Current pose:\n{current_pose}")
           
           # Create a square trajectory in XY plane
           square_size = 0.1  # 10cm square
           
           waypoints = [
               current_pose,  # Start position
           ]
           
           # Generate square waypoints
           for dx, dy in [(square_size, 0), (0, square_size), (-square_size, 0), (0, -square_size)]:
               new_pose = waypoints[-1].copy()
               new_pose[0, 3] += dx  # X translation
               new_pose[1, 3] += dy  # Y translation
               waypoints.append(new_pose)
           
           # Execute square trajectory
           for i, pose in enumerate(waypoints[1:]):
               print(f"Moving to waypoint {i+1}...")
               robot.move_to_pose(pose, speed_factor=0.1)
               
       finally:
           robot.stop_control()
           robot.disconnect()
   
   if __name__ == "__main__":
       cartesian_movement()

Advanced Examples
-----------------

Force Control
^^^^^^^^^^^^^

.. code-block:: python

   import libfrankapy as fp
   import numpy as np
   import time
   
   def force_control_example():
       robot = fp.FrankaRobot("192.168.1.100")
       
       try:
           robot.connect()
           robot.start_control()
           
           # Set up force control parameters
           force_threshold = 10.0  # Newtons
           contact_force = np.array([0, 0, -5.0, 0, 0, 0])  # 5N downward force
           
           # Start force control mode
           robot.start_force_control()
           
           # Monitor forces and react
           start_time = time.time()
           while time.time() - start_time < 10.0:  # Run for 10 seconds
               state = robot.get_robot_state()
               current_forces = state.cartesian_state.forces
               
               # Check if contact is detected
               if np.linalg.norm(current_forces[:3]) > force_threshold:
                   print(f"Contact detected! Forces: {current_forces[:3]}")
                   # Apply controlled contact force
                   robot.set_cartesian_force(contact_force)
               else:
                   # No contact, maintain position
                   robot.set_cartesian_force(np.zeros(6))
               
               time.sleep(0.01)  # 100Hz control loop
           
           # Stop force control
           robot.stop_force_control()
           
       finally:
           robot.stop_control()
           robot.disconnect()
   
   if __name__ == "__main__":
       force_control_example()

Trajectory Following
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   import libfrankapy as fp
   import numpy as np
   import matplotlib.pyplot as plt
   
   def trajectory_following():
       robot = fp.FrankaRobot("192.168.1.100")
       
       try:
           robot.connect()
           robot.start_control()
           
           # Generate a smooth trajectory
           duration = 5.0  # seconds
           frequency = 0.5  # Hz
           amplitude = 0.1  # meters
           
           # Time points
           dt = 0.01
           t = np.arange(0, duration, dt)
           
           # Get starting position
           start_pose = robot.get_robot_state().cartesian_state.pose
           
           # Generate sinusoidal trajectory in Y direction
           y_trajectory = amplitude * np.sin(2 * np.pi * frequency * t)
           
           # Execute trajectory
           positions = []
           for i, y_offset in enumerate(y_trajectory):
               target_pose = start_pose.copy()
               target_pose[1, 3] += y_offset
               
               robot.set_cartesian_pose(target_pose)
               
               # Record actual position
               actual_pose = robot.get_robot_state().cartesian_state.pose
               positions.append(actual_pose[1, 3] - start_pose[1, 3])
               
               time.sleep(dt)
           
           # Plot results
           plt.figure(figsize=(10, 6))
           plt.plot(t, y_trajectory, label='Desired', linewidth=2)
           plt.plot(t, positions, label='Actual', linewidth=2)
           plt.xlabel('Time (s)')
           plt.ylabel('Y Position (m)')
           plt.title('Trajectory Following Performance')
           plt.legend()
           plt.grid(True)
           plt.show()
           
       finally:
           robot.stop_control()
           robot.disconnect()
   
   if __name__ == "__main__":
       trajectory_following()

State Monitoring
^^^^^^^^^^^^^^^^

.. code-block:: python

   import libfrankapy as fp
   import time
   import csv
   
   def state_monitoring():
       robot = fp.FrankaRobot("192.168.1.100")
       
       try:
           robot.connect()
           robot.start_control()
           
           # Data collection
           data = []
           start_time = time.time()
           
           print("Collecting robot state data for 10 seconds...")
           
           while time.time() - start_time < 10.0:
               state = robot.get_robot_state()
               
               # Collect relevant data
               timestamp = time.time() - start_time
               joint_pos = state.joint_state.positions
               joint_vel = state.joint_state.velocities
               cartesian_pos = state.cartesian_state.pose[:3, 3]
               forces = state.cartesian_state.forces[:3]
               
               data.append({
                   'time': timestamp,
                   'joint_pos': joint_pos,
                   'joint_vel': joint_vel,
                   'cartesian_pos': cartesian_pos,
                   'forces': forces
               })
               
               # Print current state
               if len(data) % 100 == 0:  # Print every second
                   print(f"Time: {timestamp:.1f}s, Position: {cartesian_pos}")
               
               time.sleep(0.01)  # 100Hz sampling
           
           # Save data to CSV
           with open('robot_state_log.csv', 'w', newline='') as csvfile:
               fieldnames = ['time', 'joint_pos', 'joint_vel', 'cartesian_pos', 'forces']
               writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
               writer.writeheader()
               writer.writerows(data)
           
           print(f"Data saved to robot_state_log.csv ({len(data)} samples)")
           
       finally:
           robot.stop_control()
           robot.disconnect()
   
   if __name__ == "__main__":
       state_monitoring()

Error Handling and Recovery
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   import libfrankapy as fp
   import time
   
   def error_handling_example():
       robot = fp.FrankaRobot("192.168.1.100")
       
       try:
           robot.connect()
           robot.start_control()
           
           # Simulate various scenarios that might cause errors
           scenarios = [
               "normal_operation",
               "joint_limits",
               "high_velocity",
               "force_limit"
           ]
           
           for scenario in scenarios:
               print(f"\nTesting scenario: {scenario}")
               
               try:
                   if scenario == "normal_operation":
                       # Normal movement
                       target = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
                       robot.move_to_joint(target, speed_factor=0.1)
                       
                   elif scenario == "joint_limits":
                       # Try to move beyond joint limits (will be caught by safety)
                       target = [3.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]  # Joint 1 limit exceeded
                       robot.move_to_joint(target, speed_factor=0.1)
                       
                   elif scenario == "high_velocity":
                       # Try high velocity movement
                       target = [1.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
                       robot.move_to_joint(target, speed_factor=1.0)  # Very fast
                       
                   elif scenario == "force_limit":
                       # Simulate force limit scenario
                       robot.start_force_control()
                       high_force = [0, 0, -100.0, 0, 0, 0]  # Very high force
                       robot.set_cartesian_force(high_force)
                       time.sleep(0.1)
                       robot.stop_force_control()
                   
                   print(f"Scenario {scenario} completed successfully")
                   
               except fp.FrankaException as e:
                   print(f"Robot error in {scenario}: {e}")
                   print("Attempting recovery...")
                   
                   # Attempt to recover from error
                   robot.recover_from_errors()
                   
                   # Wait a moment before continuing
                   time.sleep(1.0)
                   
                   print("Recovery completed")
               
               except Exception as e:
                   print(f"Unexpected error in {scenario}: {e}")
                   break
           
       finally:
           robot.stop_control()
           robot.disconnect()
   
   if __name__ == "__main__":
       error_handling_example()

Integration Examples
--------------------

ROS Integration
^^^^^^^^^^^^^^^

.. code-block:: python

   # Example of integrating libfrankapy with ROS
   import rospy
   import libfrankapy as fp
   from geometry_msgs.msg import PoseStamped
   from sensor_msgs.msg import JointState
   
   class FrankaROSBridge:
       def __init__(self, robot_ip):
           self.robot = fp.FrankaRobot(robot_ip)
           
           # ROS setup
           rospy.init_node('franka_ros_bridge')
           
           # Publishers
           self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
           self.pose_pub = rospy.Publisher('/ee_pose', PoseStamped, queue_size=1)
           
           # Subscribers
           self.pose_sub = rospy.Subscriber('/target_pose', PoseStamped, self.pose_callback)
           
       def pose_callback(self, msg):
           # Convert ROS pose to numpy array and command robot
           target_pose = self.ros_pose_to_matrix(msg.pose)
           self.robot.move_to_pose(target_pose, speed_factor=0.1)
       
       def ros_pose_to_matrix(self, pose):
           # Convert ROS Pose to 4x4 transformation matrix
           # Implementation details...
           pass
       
       def run(self):
           self.robot.connect()
           self.robot.start_control()
           
           rate = rospy.Rate(100)  # 100Hz
           
           while not rospy.is_shutdown():
               # Publish current state
               state = self.robot.get_robot_state()
               
               # Publish joint states
               joint_msg = JointState()
               joint_msg.header.stamp = rospy.Time.now()
               joint_msg.position = state.joint_state.positions
               joint_msg.velocity = state.joint_state.velocities
               self.joint_pub.publish(joint_msg)
               
               # Publish end-effector pose
               pose_msg = PoseStamped()
               pose_msg.header.stamp = rospy.Time.now()
               pose_msg.header.frame_id = "base_link"
               # Convert matrix to ROS pose...
               self.pose_pub.publish(pose_msg)
               
               rate.sleep()
   
   if __name__ == "__main__":
       bridge = FrankaROSBridge("192.168.1.100")
       bridge.run()

Running the Examples
--------------------

To run these examples:

1. Make sure your Franka robot is properly set up and accessible
2. Update the robot IP address in the examples
3. Ensure you have the necessary permissions for real-time control
4. Run the examples in a safe environment with proper safety measures

.. warning::
   Always ensure the robot workspace is clear and emergency stop is accessible when running these examples.