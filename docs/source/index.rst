LibFrankaPy Documentation
========================

Welcome to LibFrankaPy documentation! LibFrankaPy is a Python binding project for the libfranka C++ library, designed to provide high-level Python interfaces for Franka robotic arms while maintaining the performance advantages of low-level C++ real-time control.

.. image:: https://img.shields.io/badge/License-Apache%202.0-blue.svg
   :target: https://opensource.org/licenses/Apache-2.0
   :alt: License

.. image:: https://img.shields.io/badge/python-3.8+-blue.svg
   :target: https://www.python.org/downloads/
   :alt: Python Version

.. image:: https://img.shields.io/badge/C++-17-blue.svg
   :target: https://en.cppreference.com/w/cpp/17
   :alt: C++ Version

Features
--------

🚀 **Real-time Performance Guarantee**: C++ control loop maintains 1kHz real-time performance, Python does not participate in real-time loops

🐍 **Python Friendly**: Provides intuitive Python API with complete type hints

⚡ **Efficient Communication**: Uses shared memory and atomic operations for Python-C++ data exchange

🛡️ **Safety Control**: Complete safety limits, error handling, and emergency stop functionality

🎯 **Multiple Control Modes**: Supports joint space, Cartesian space, and trajectory control

📊 **Real-time Monitoring**: Complete robot state feedback and monitoring functionality

🏗️ **Modular Design**: Clear code structure and interface separation

Quick Start
-----------

Installation
^^^^^^^^^^^^

.. code-block:: bash

   pip install libfrankapy

Basic Usage
^^^^^^^^^^^

.. code-block:: python

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

Table of Contents
-----------------

.. toctree::
   :maxdepth: 2
   :caption: User Guide

   installation
   quickstart
   examples
   configuration

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   api/robot
   api/control
   api/state
   api/exceptions

.. toctree::
   :maxdepth: 2
   :caption: Developer Guide

   development/contributing
   development/architecture
   development/testing

.. toctree::
   :maxdepth: 1
   :caption: Other

   changelog
   license

Indices and Tables
------------------

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Links
-----

* `GitHub Repository <https://github.com/yourusername/libfrankapy>`_
* `PyPI Page <https://pypi.org/project/libfrankapy/>`_
* `Issue Tracker <https://github.com/yourusername/libfrankapy/issues>`_
* `Discussions <https://github.com/yourusername/libfrankapy/discussions>`_

License
-------

This project is licensed under the `Apache License 2.0 <https://opensource.org/licenses/Apache-2.0>`_.