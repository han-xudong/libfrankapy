Contributing
============

We welcome contributions to LibFrankaPy! This guide will help you get started with contributing to the project.

Getting Started
---------------

Development Environment Setup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. **Fork and Clone the Repository**

   .. code-block:: bash

      git clone https://github.com/yourusername/libfrankapy.git
      cd libfrankapy

2. **Set Up Development Environment**

   .. code-block:: bash

      # Create virtual environment
      python -m venv venv
      source venv/bin/activate  # On Windows: venv\Scripts\activate
      
      # Install development dependencies
      pip install -e ".[dev]"

3. **Install System Dependencies**

   Follow the installation guide to install libfranka and other system dependencies.

4. **Set Up Pre-commit Hooks**

   .. code-block:: bash

      pre-commit install

Development Workflow
^^^^^^^^^^^^^^^^^^^^

1. **Create a Feature Branch**

   .. code-block:: bash

      git checkout -b feature/your-feature-name

2. **Make Your Changes**

   - Write code following our coding standards
   - Add tests for new functionality
   - Update documentation as needed

3. **Run Tests and Checks**

   .. code-block:: bash

      # Run tests
      pytest
      
      # Run linting
      flake8 libfrankapy/
      
      # Run type checking
      mypy libfrankapy/
      
      # Check code formatting
      black --check libfrankapy/
      isort --check-only libfrankapy/

4. **Commit Your Changes**

   .. code-block:: bash

      git add .
      git commit -m "feat: add new feature description"

5. **Push and Create Pull Request**

   .. code-block:: bash

      git push origin feature/your-feature-name

Coding Standards
----------------

Python Code Style
^^^^^^^^^^^^^^^^^

- Follow **PEP 8** style guidelines
- Use **Black** for code formatting
- Use **isort** for import sorting
- Maximum line length: **88 characters**
- Use **type hints** for all public functions

.. code-block:: python

   # Good example
   def move_to_joint(
       self,
       target_positions: List[float],
       speed_factor: float = 0.1,
       timeout: Optional[float] = None
   ) -> bool:
       """Move robot to target joint positions.
       
       Args:
           target_positions: List of 7 joint angles in radians
           speed_factor: Speed factor between 0.0 and 1.0
           timeout: Maximum time to wait for completion
           
       Returns:
           True if movement completed successfully
           
       Raises:
           JointLimitError: If target positions exceed joint limits
           VelocityLimitError: If speed_factor is too high
       """
       # Implementation here
       pass

C++ Code Style
^^^^^^^^^^^^^^

- Follow **Google C++ Style Guide**
- Use **clang-format** for formatting
- Use **snake_case** for variables and functions
- Use **PascalCase** for classes
- Include comprehensive documentation

.. code-block:: cpp

   // Good example
   class RealTimeController {
   public:
       /**
        * @brief Initialize the real-time controller
        * @param robot_ip IP address of the robot
        * @param control_frequency Control loop frequency in Hz
        * @return true if initialization successful
        */
       bool initialize(const std::string& robot_ip, double control_frequency);
       
   private:
       std::string robot_ip_;
       double control_frequency_;
   };

Documentation Standards
^^^^^^^^^^^^^^^^^^^^^^^

- Use **Google-style docstrings** for Python
- Use **Doxygen comments** for C++
- Include **examples** in docstrings
- Document **all public APIs**
- Update **README** and **documentation** for new features

Testing Guidelines
------------------

Test Structure
^^^^^^^^^^^^^^

.. code-block:: text

   tests/
   ├── unit/                 # Unit tests
   │   ├── test_robot.py
   │   ├── test_control.py
   │   └── test_state.py
   ├── integration/          # Integration tests
   │   ├── test_robot_control.py
   │   └── test_force_control.py
   ├── fixtures/             # Test fixtures and data
   └── conftest.py          # Pytest configuration

Writing Tests
^^^^^^^^^^^^^

1. **Unit Tests**

   .. code-block:: python

      import pytest
      from unittest.mock import Mock, patch
      import libfrankapy as fp
      
      class TestFrankaRobot:
          def test_robot_initialization(self):
              robot = fp.FrankaRobot("192.168.1.100")
              assert robot.ip_address == "192.168.1.100"
              assert not robot.is_connected
          
          @patch('libfrankapy.robot.libfranka')
          def test_connection(self, mock_libfranka):
              robot = fp.FrankaRobot("192.168.1.100")
              robot.connect()
              mock_libfranka.Robot.assert_called_once()

2. **Integration Tests**

   .. code-block:: python

      import pytest
      import libfrankapy as fp
      
      @pytest.mark.integration
      @pytest.mark.requires_robot
      class TestRobotIntegration:
          @pytest.fixture
          def robot(self):
              robot = fp.FrankaRobot("192.168.1.100")
              robot.connect()
              robot.start_control()
              yield robot
              robot.stop_control()
              robot.disconnect()
          
          def test_joint_movement(self, robot):
              target = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
              result = robot.move_to_joint(target, speed_factor=0.1)
              assert result is True

3. **Test Configuration**

   .. code-block:: python

      # conftest.py
      import pytest
      
      def pytest_configure(config):
          config.addinivalue_line(
              "markers", "integration: mark test as integration test"
          )
          config.addinivalue_line(
              "markers", "requires_robot: mark test as requiring real robot"
          )
      
      @pytest.fixture(scope="session")
      def robot_ip():
          return "192.168.1.100"

Running Tests
^^^^^^^^^^^^^

.. code-block:: bash

   # Run all tests
   pytest
   
   # Run only unit tests
   pytest tests/unit/
   
   # Run with coverage
   pytest --cov=libfrankapy --cov-report=html
   
   # Run integration tests (requires robot)
   pytest -m integration
   
   # Skip robot-dependent tests
   pytest -m "not requires_robot"

Contribution Types
------------------

Bug Fixes
^^^^^^^^^

1. **Report the Bug**
   - Create an issue with detailed description
   - Include steps to reproduce
   - Provide system information

2. **Fix the Bug**
   - Write a test that reproduces the bug
   - Implement the fix
   - Ensure the test passes

3. **Submit Pull Request**
   - Reference the issue number
   - Describe the fix clearly

New Features
^^^^^^^^^^^^

1. **Discuss the Feature**
   - Create an issue or discussion
   - Get feedback from maintainers
   - Agree on the design approach

2. **Implement the Feature**
   - Follow the agreed design
   - Add comprehensive tests
   - Update documentation

3. **Submit Pull Request**
   - Include feature description
   - Provide usage examples

Documentation
^^^^^^^^^^^^^

- Fix typos and improve clarity
- Add examples and tutorials
- Update API documentation
- Improve installation guides

Performance Improvements
^^^^^^^^^^^^^^^^^^^^^^^^

- Profile the code to identify bottlenecks
- Implement optimizations
- Add benchmarks to verify improvements
- Ensure no functionality is broken

Pull Request Guidelines
-----------------------

PR Title and Description
^^^^^^^^^^^^^^^^^^^^^^^^

Use conventional commit format:

- **feat**: New feature
- **fix**: Bug fix
- **docs**: Documentation changes
- **style**: Code style changes
- **refactor**: Code refactoring
- **test**: Adding or updating tests
- **chore**: Maintenance tasks

Example:

.. code-block:: text

   feat: add force control with impedance parameters
   
   - Implement ImpedanceController class
   - Add configurable stiffness and damping
   - Include safety limits for force control
   - Add comprehensive tests and examples
   
   Closes #123

PR Checklist
^^^^^^^^^^^^

Before submitting a PR, ensure:

- [ ] Code follows style guidelines
- [ ] Tests are added for new functionality
- [ ] All tests pass
- [ ] Documentation is updated
- [ ] Commit messages are clear
- [ ] PR description explains the changes
- [ ] Breaking changes are documented

Code Review Process
^^^^^^^^^^^^^^^^^^^

1. **Automated Checks**
   - CI/CD pipeline runs tests
   - Code quality checks pass
   - Documentation builds successfully

2. **Manual Review**
   - Maintainers review the code
   - Feedback is provided
   - Changes are requested if needed

3. **Approval and Merge**
   - PR is approved by maintainers
   - Code is merged to main branch
   - Release notes are updated

Development Tools
-----------------

Recommended IDE Setup
^^^^^^^^^^^^^^^^^^^^^

**VS Code Extensions:**

- Python
- C/C++
- Pylance
- Black Formatter
- isort
- GitLens

**PyCharm Configuration:**

- Enable type checking
- Configure Black as formatter
- Set up pytest as test runner
- Install Python requirements

Debugging
^^^^^^^^^

1. **Python Debugging**

   .. code-block:: python

      import pdb; pdb.set_trace()  # Breakpoint
      
      # Or use logging
      import logging
      logging.basicConfig(level=logging.DEBUG)
      logger = logging.getLogger(__name__)
      logger.debug("Debug message")

2. **C++ Debugging**

   .. code-block:: bash

      # Build with debug symbols
      cmake -DCMAKE_BUILD_TYPE=Debug ..
      make
      
      # Use GDB
      gdb python
      (gdb) run your_script.py

Profiling
^^^^^^^^^

1. **Python Profiling**

   .. code-block:: python

      import cProfile
      import pstats
      
      # Profile your code
      cProfile.run('your_function()', 'profile_output')
      
      # Analyze results
      stats = pstats.Stats('profile_output')
      stats.sort_stats('cumulative').print_stats(10)

2. **Memory Profiling**

   .. code-block:: bash

      pip install memory-profiler
      python -m memory_profiler your_script.py

Community Guidelines
--------------------

Code of Conduct
^^^^^^^^^^^^^^^

- Be respectful and inclusive
- Provide constructive feedback
- Help newcomers get started
- Follow project guidelines

Communication Channels
^^^^^^^^^^^^^^^^^^^^^^

- **GitHub Issues**: Bug reports and feature requests
- **GitHub Discussions**: General questions and ideas
- **Pull Requests**: Code contributions
- **Documentation**: API and usage questions

Getting Help
^^^^^^^^^^^^

- Check existing issues and documentation
- Ask questions in GitHub Discussions
- Provide detailed information when asking for help
- Be patient and respectful

Release Process
---------------

Versioning
^^^^^^^^^^

We follow **Semantic Versioning** (SemVer):

- **MAJOR**: Breaking changes
- **MINOR**: New features (backward compatible)
- **PATCH**: Bug fixes (backward compatible)

Release Checklist
^^^^^^^^^^^^^^^^^

1. Update version numbers
2. Update CHANGELOG.md
3. Run full test suite
4. Build and test packages
5. Create release tag
6. Publish to PyPI
7. Update documentation

Thank You!
----------

Thank you for contributing to LibFrankaPy! Your contributions help make robotics more accessible and improve the experience for all users.

For questions about contributing, please:

- Check the documentation
- Search existing issues
- Create a new issue or discussion
- Contact the maintainers

We appreciate your time and effort in making LibFrankaPy better!