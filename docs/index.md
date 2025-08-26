---
layout: home

hero:
  name: "libfrankapy"
  text: "Python library for controlling Franka Emika robots"
  tagline: "High-performance Python bindings for libfranka with real-time control capabilities"
  actions:
    - theme: brand
      text: Get Started
      link: /installation
    - theme: alt
      text: View on GitHub
      link: https://github.com/han-xudong/libfrankapy

features:
  - icon: ⚡
    title: Real-time Performance
    details: C++ control loop maintains 1kHz real-time performance, Python does not participate in real-time loops
  - icon: 🐍
    title: Python Friendly
    details: Provides intuitive Python API with complete type hints
  - icon: 🚀
    title: Efficient Communication
    details: Uses shared memory and atomic operations for Python-C++ data exchange
  - icon: 🛡️
    title: Safety Control
    details: Complete safety limits, error handling, and emergency stop functionality
  - icon: 🎯
    title: Multiple Control Modes
    details: Supports joint space, Cartesian space, and trajectory control
  - icon: 📊
    title: Real-time Monitoring
    details: Complete robot state feedback and monitoring functionality
---

## Quick Start

### Installation

```bash
pip install libfrankapy
```

### Basic Usage

```python
import libfrankapy as fp

# Connect to robot
robot = fp.FrankaRobot("192.168.1.100")
robot.connect()
robot.start_control()

try:
    # Get current state
    state = robot.get_robot_state()
    print(f"Current joint positions: {state.joint_state.positions}")
    
    # Joint space motion
    target_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    robot.move_to_joint(target_joints, speed_factor=0.1)
    
finally:
    robot.disconnect()
```

## Badges

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Python Version](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![C++ Version](https://img.shields.io/badge/C++-17-blue.svg)](https://en.cppreference.com/w/cpp/17)

## Links

- [GitHub Repository](https://github.com/han-xudong/libfrankapy)
- [PyPI Page](https://pypi.org/project/libfrankapy/)
- [Issue Tracker](https://github.com/han-xudong/libfrankapy/issues)
- [Discussions](https://github.com/han-xudong/libfrankapy/discussions)

## License

This project is licensed under the [Apache License 2.0](https://opensource.org/licenses/Apache-2.0).
