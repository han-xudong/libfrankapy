Changelog
=========

All notable changes to libfrankapy will be documented in this file.

The format is based on `Keep a Changelog <https://keepachangelog.com/en/1.0.0/>`_,
and this project adheres to `Semantic Versioning <https://semver.org/spec/v2.0.0.html>`_.

[Unreleased]
------------

Added
^^^^^
- New impedance control interface with configurable stiffness and damping
- Support for custom control plugins
- Advanced trajectory planning with obstacle avoidance
- Real-time performance monitoring and metrics
- Multi-robot coordination capabilities
- Enhanced error recovery mechanisms

Changed
^^^^^^^
- Improved shared memory performance with lock-free algorithms
- Updated documentation with more comprehensive examples
- Enhanced type hints for better IDE support
- Optimized C++ control loop for lower latency

Fixed
^^^^^
- Memory leak in continuous state monitoring
- Race condition in real-time controller initialization
- Incorrect force sensor calibration in certain configurations

[0.1.0] - 2024-01-15
---------------------

Added
^^^^^
- Initial release of libfrankapy
- Core robot interface with connection management
- Joint space control with position, velocity, and torque modes
- Cartesian space control with pose and velocity commands
- Force control with configurable force/torque limits
- Real-time state monitoring at 1kHz
- Comprehensive safety monitoring and error handling
- Python bindings for C++ real-time controller
- Shared memory communication between Python and C++
- Complete API documentation and examples
- Unit and integration test suite
- CI/CD pipeline with automated testing

Supported Features
^^^^^^^^^^^^^^^^^^
- **Robot Control**:
  - Joint position control
  - Joint velocity control
  - Joint torque control
  - Cartesian pose control
  - Cartesian velocity control
  - Force/torque control
  - Impedance control

- **Safety Features**:
  - Joint limit monitoring
  - Velocity limit enforcement
  - Force/torque limit checking
  - Collision detection
  - Emergency stop functionality
  - Protective stop handling
  - Automatic error recovery

- **State Monitoring**:
  - Real-time joint states (position, velocity, torque)
  - Cartesian pose and velocity
  - Force/torque measurements
  - System status and error states
  - Performance metrics

- **Programming Interface**:
  - Pythonic API with type hints
  - Comprehensive error handling
  - Configurable control parameters
  - Event-driven state updates
  - Plugin architecture for extensions

Technical Specifications
^^^^^^^^^^^^^^^^^^^^^^^^
- **Real-time Performance**: 1kHz control loop
- **Communication Latency**: < 1ms typical
- **Python Compatibility**: 3.8, 3.9, 3.10, 3.11
- **Operating System**: Linux (Ubuntu 20.04+)
- **Dependencies**: libfranka 0.15.0, Eigen3, Poco
- **Build System**: CMake 3.16+, pybind11

Known Limitations
^^^^^^^^^^^^^^^^^
- Linux-only support (PREEMPT_RT kernel recommended)
- Single robot control per process
- Limited to Franka Emika Panda robots
- Requires real-time kernel for optimal performance

Breaking Changes
^^^^^^^^^^^^^^^^
None (initial release)

Migration Guide
^^^^^^^^^^^^^^^
Not applicable (initial release)

Security Updates
^^^^^^^^^^^^^^^^
None (initial release)

Deprecations
^^^^^^^^^^^^
None (initial release)

Contributors
^^^^^^^^^^^^
- libfrankapy Team
- Community contributors

---

## Version History

### Development Milestones

**v0.1.0-alpha** (2023-12-01)
- Initial alpha release for testing
- Basic robot control functionality
- Core safety features
- Limited documentation

**v0.1.0-beta** (2023-12-15)
- Beta release with expanded features
- Comprehensive testing suite
- Improved documentation
- Performance optimizations

**v0.1.0-rc1** (2024-01-05)
- Release candidate with final API
- Complete documentation
- Full test coverage
- Production-ready stability

**v0.1.0** (2024-01-15)
- First stable release
- All planned features implemented
- Comprehensive documentation
- Production deployment ready

### Upcoming Releases

**v0.2.0** (Planned Q2 2024)
- Multi-robot support
- Advanced planning algorithms
- Enhanced visualization tools
- Performance improvements

**v0.3.0** (Planned Q3 2024)
- Machine learning integration
- Cloud connectivity features
- Advanced safety mechanisms
- Extended hardware support

**v1.0.0** (Planned Q4 2024)
- First major stable release
- Long-term API stability
- Enterprise features
- Comprehensive ecosystem

---

## Detailed Change Log

### Core Features

#### Robot Interface
- Implemented `FrankaRobot` class as main interface
- Added connection management with automatic reconnection
- Integrated real-time control loop at 1kHz
- Provided comprehensive state monitoring

#### Control Algorithms
- Joint space control with smooth trajectory generation
- Cartesian space control with orientation handling
- Force control with impedance parameters
- Safety-aware motion planning

#### Safety Systems
- Multi-layer safety architecture
- Real-time limit monitoring
- Collision detection and response
- Emergency stop integration
- Graceful error recovery

### Technical Implementation

#### Architecture
- Hybrid Python-C++ design for performance
- Lock-free shared memory communication
- Real-time thread with PREEMPT_RT support
- Modular plugin architecture

#### Performance
- Sub-millisecond control loop latency
- Efficient memory management
- Optimized data structures
- Minimal CPU overhead

#### Quality Assurance
- Comprehensive test suite (>95% coverage)
- Continuous integration pipeline
- Static analysis and linting
- Performance benchmarking

### Documentation

#### User Documentation
- Complete API reference
- Tutorial and examples
- Installation guide
- Configuration manual

#### Developer Documentation
- Architecture overview
- Contributing guidelines
- Testing procedures
- Performance optimization

### Community

#### Open Source
- Apache 2.0 license
- GitHub repository
- Issue tracking
- Community discussions

#### Support
- Documentation website
- Example applications
- Community forum
- Professional support options

---

## Release Notes Format

Each release follows this format:

### [Version] - Date

**Added**
- New features and capabilities

**Changed**
- Modifications to existing features
- Performance improvements
- API changes (non-breaking)

**Deprecated**
- Features marked for removal
- Migration recommendations

**Removed**
- Deleted features and APIs
- Breaking changes

**Fixed**
- Bug fixes and corrections
- Security patches

**Security**
- Security-related changes
- Vulnerability fixes

---

## Versioning Policy

libfrankapy follows Semantic Versioning (SemVer):

- **MAJOR** version for incompatible API changes
- **MINOR** version for backwards-compatible functionality additions
- **PATCH** version for backwards-compatible bug fixes

### Version Components

- **Major.Minor.Patch** (e.g., 1.2.3)
- **Pre-release identifiers**: alpha, beta, rc (e.g., 1.0.0-alpha.1)
- **Build metadata**: +build.1 (e.g., 1.0.0+build.1)

### API Stability

- **Stable API**: Major version 1.0.0 and above
- **Beta API**: Minor versions 0.x.x
- **Alpha API**: Pre-release versions

### Deprecation Policy

- Features are deprecated for at least one minor version
- Deprecation warnings are provided in documentation and code
- Migration guides are provided for breaking changes
- Legacy support is maintained when possible

---

## Contributing to Changelog

When contributing to libfrankapy:

1. **Add entries** to the "Unreleased" section
2. **Follow the format** described above
3. **Include issue/PR references** when applicable
4. **Describe user impact** rather than implementation details
5. **Group related changes** under appropriate categories

### Example Entry

::

    Added
    ^^^^^
    - New force control interface with impedance parameters (#123)
    - Support for custom trajectory generators (#145)

    Fixed
    ^^^^^
    - Memory leak in state monitoring thread (#156)
    - Incorrect joint limit validation for joint 7 (#162)

---

## Links and References

- `GitHub Repository <https://github.com/libfrankapy/libfrankapy>`_
- `Issue Tracker <https://github.com/libfrankapy/libfrankapy/issues>`_
- `Documentation <https://libfrankapy.readthedocs.io>`_
- `PyPI Package <https://pypi.org/project/libfrankapy/>`_
- `Release Downloads <https://github.com/libfrankapy/libfrankapy/releases>`_