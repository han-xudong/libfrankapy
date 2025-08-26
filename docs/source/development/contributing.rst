Contributing to LibFrankaPy
============================

We welcome contributions to LibFrankaPy! This guide will help you get started.

Development Setup
-----------------

Prerequisites
~~~~~~~~~~~~~

* Python 3.8 or higher
* CMake 3.16 or higher
* C++17 compatible compiler
* libfranka installed
* Git

Setting Up Development Environment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Fork the repository on GitHub
2. Clone your fork::

    git clone https://github.com/yourusername/libfrankapy.git
    cd libfrankapy

3. Create a virtual environment::

    python -m venv venv
    source venv/bin/activate  # On Windows: venv\Scripts\activate

4. Install development dependencies::

    pip install -e ".[dev]"

5. Install pre-commit hooks::

    pre-commit install

Development Workflow
--------------------

1. Create a new branch for your feature::

    git checkout -b feature/your-feature-name

2. Make your changes
3. Run tests::

    pytest

4. Run linting::

    flake8 libfrankapy/
    black libfrankapy/
    isort libfrankapy/

5. Commit your changes::

    git add .
    git commit -m "Add your descriptive commit message"

6. Push to your fork::

    git push origin feature/your-feature-name

7. Create a Pull Request on GitHub

Coding Standards
----------------

Python Code
~~~~~~~~~~~

* Follow PEP 8 style guide
* Use type hints for all function signatures
* Write docstrings for all public functions and classes
* Use meaningful variable and function names
* Keep functions focused and small (< 50 lines when possible)

C++ Code
~~~~~~~~

* Follow Google C++ Style Guide
* Use modern C++17 features
* Use RAII for resource management
* Write clear, self-documenting code
* Use const correctness

Testing
-------

All contributions must include appropriate tests:

* Unit tests for individual functions/classes
* Integration tests for component interactions
* System tests for end-to-end functionality
* Performance tests for critical paths

Running Tests
~~~~~~~~~~~~~

::

    # Run all tests
    pytest
    
    # Run specific test file
    pytest tests/test_robot.py
    
    # Run with coverage
    pytest --cov=libfrankapy
    
    # Run performance tests
    pytest tests/performance/

Documentation
-------------

All code changes should include documentation updates:

* Update docstrings for modified functions
* Add examples for new features
* Update README if necessary
* Add changelog entries

Building Documentation
~~~~~~~~~~~~~~~~~~~~~~

::

    cd docs
    make html
    # Open _build/html/index.html in browser

Pull Request Guidelines
-----------------------

Before submitting a pull request:

1. Ensure all tests pass
2. Update documentation
3. Add changelog entry
4. Rebase on latest main branch
5. Write clear commit messages
6. Include description of changes

Pull Request Template
~~~~~~~~~~~~~~~~~~~~~

::

    ## Description
    Brief description of changes
    
    ## Type of Change
    - [ ] Bug fix
    - [ ] New feature
    - [ ] Documentation update
    - [ ] Performance improvement
    
    ## Testing
    - [ ] Unit tests added/updated
    - [ ] Integration tests added/updated
    - [ ] Manual testing performed
    
    ## Checklist
    - [ ] Code follows style guidelines
    - [ ] Self-review completed
    - [ ] Documentation updated
    - [ ] Tests pass locally

Code Review Process
-------------------

1. All PRs require at least one review
2. Maintainers will review within 48 hours
3. Address feedback promptly
4. Squash commits before merge

Getting Help
------------

* Open an issue for bugs or feature requests
* Join our Discord server for discussions
* Check existing documentation and examples
* Ask questions in pull request comments

Release Process
---------------

1. Update version numbers
2. Update changelog
3. Create release tag
4. Build and upload packages
5. Update documentation