#!/usr/bin/env python3
"""Setup script for LibFrankaPy.

This script builds the C++ extension module and installs the Python package.
"""

import os
import sys
import subprocess
import platform
from pathlib import Path

from pybind11.setup_helpers import Pybind11Extension, build_ext
import pybind11
from setuptools import setup, find_packages

# Package information
PACKAGE_NAME = "libfrankapy"
VERSION = "0.1.0"
AUTHOR = "LibFrankaPy Team"
AUTHOR_EMAIL = "support@libfrankapy.org"
DESCRIPTION = "Python bindings for libfranka with real-time control"
URL = "https://github.com/libfrankapy/libfrankapy"

# Read long description from README
this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text(encoding="utf-8")

# Check if we're on a supported platform
if platform.system() != "Linux":
    raise RuntimeError("LibFrankaPy only supports Linux with PREEMPT_RT kernel")


# Check for required system libraries
def check_library(lib_name, pkg_config_name=None):
    """Check if a system library is available."""
    if pkg_config_name:
        try:
            subprocess.check_call(
                ["pkg-config", "--exists", pkg_config_name],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            return True
        except (subprocess.CalledProcessError, FileNotFoundError):
            pass

    # Get additional search paths from CMAKE_PREFIX_PATH
    search_paths = ["/usr/lib", "/usr/local/lib", "/opt/lib", "/opt/libfranka/lib"]

    # Add paths from CMAKE_PREFIX_PATH environment variable
    cmake_prefix_path = os.environ.get("CMAKE_PREFIX_PATH", "")
    if cmake_prefix_path:
        for path in cmake_prefix_path.split(":"):
            if path.strip():
                lib_path = os.path.join(path.strip(), "lib")
                if lib_path not in search_paths:
                    search_paths.append(lib_path)

    # Try to find the library in all search locations
    for lib_dir in search_paths:
        if os.path.exists(f"{lib_dir}/lib{lib_name}.so"):
            return True

    return False


# Check for required dependencies
required_libs = [
    ("franka", None),
    ("PocoFoundation", "poco-foundation"),
    ("eigen3", "eigen3"),
]

missing_libs = []
for lib_name, pkg_name in required_libs:
    if not check_library(lib_name, pkg_name):
        missing_libs.append(lib_name)

if missing_libs:
    print("Error: Missing required system libraries:")
    for lib in missing_libs:
        print(f"  - {lib}")
    print("\nPlease install the missing libraries and try again.")
    print("See README.md for installation instructions.")
    sys.exit(1)


# Get include directories
def get_include_dirs():
    """Get include directories for compilation."""
    include_dirs = [
        # pybind11 includes
        pybind11.get_include(),
        # Local source directory
        "src",
        # libfranka include directory
        "/opt/libfranka/include",
    ]

    # Add paths from CMAKE_PREFIX_PATH environment variable
    cmake_prefix_path = os.environ.get("CMAKE_PREFIX_PATH", "")
    if cmake_prefix_path:
        for path in cmake_prefix_path.split(":"):
            if path.strip():
                inc_path = os.path.join(path.strip(), "include")
                if os.path.exists(inc_path) and inc_path not in include_dirs:
                    include_dirs.append(inc_path)

    # Try to get Eigen3 include directory
    try:
        eigen_include = subprocess.check_output(
            ["pkg-config", "--cflags-only-I", "eigen3"],
            stderr=subprocess.DEVNULL
        ).decode().strip().replace("-I", "")
        if eigen_include:
            include_dirs.append(eigen_include)
    except (subprocess.CalledProcessError, FileNotFoundError):
        # Fallback to common locations
        for path in ["/usr/include/eigen3", "/usr/local/include/eigen3"]:
            if os.path.exists(path):
                include_dirs.append(path)
                break

    return include_dirs


# Get library directories and libraries
def get_libraries():
    """Get libraries to link against."""
    libraries = ["franka", "PocoFoundation", "pthread", "rt"]
    library_dirs = ["/usr/lib", "/usr/local/lib", "/opt/libfranka/lib"]

    # Add paths from CMAKE_PREFIX_PATH environment variable
    cmake_prefix_path = os.environ.get("CMAKE_PREFIX_PATH", "")
    if cmake_prefix_path:
        for path in cmake_prefix_path.split(":"):
            if path.strip():
                lib_path = os.path.join(path.strip(), "lib")
                if lib_path not in library_dirs:
                    library_dirs.append(lib_path)

    return libraries, library_dirs


# Define the extension module
def create_extension():
    """Create the pybind11 extension."""

    # Source files
    sources = [
        "src/python_bindings.cpp",
        "src/shared_memory.cpp",
        "src/realtime_controller.cpp",
        "src/motion_generators.cpp",
    ]

    # Get compilation parameters
    include_dirs = get_include_dirs()
    libraries, library_dirs = get_libraries()

    # Compiler flags
    extra_compile_args = [
        "-std=c++17",
        "-O3",
        "-Wall",
        "-Wextra",
        "-fPIC",
        "-fvisibility=hidden",
        "-DWITH_PYTHON_BINDINGS",
        "-DEIGEN_MPL2_ONLY",
        "-D_GNU_SOURCE",
    ]

    # Linker flags
    extra_link_args = [
        "-Wl,--as-needed",
        "-Wl,--no-undefined",
    ]

    # Create extension
    ext = Pybind11Extension(
        "libfrankapy._libfrankapy_core",
        sources=sources,
        include_dirs=include_dirs,
        libraries=libraries,
        library_dirs=library_dirs,
        extra_compile_args=extra_compile_args,
        extra_link_args=extra_link_args,
        cxx_std=17,
    )

    return ext


# Custom build command
class CustomBuildExt(build_ext):
    """Custom build extension command."""

    def build_extensions(self):
        """Build the extensions with custom settings."""

        # Check compiler
        compiler_type = self.compiler.compiler_type
        if compiler_type == "unix":
            # Add optimization flags for release builds
            for ext in self.extensions:
                ext.extra_compile_args.extend([
                    "-march=native",
                    "-mtune=native",
                ])

        # Call parent build
        super().build_extensions()

    def run(self):
        """Run the build process."""
        print("Building LibFrankaPy C++ extension...")
        super().run()
        print("Build completed successfully!")


# Setup configuration
setup(
    name=PACKAGE_NAME,
    version=VERSION,
    author=AUTHOR,
    author_email=AUTHOR_EMAIL,
    description=DESCRIPTION,
    long_description=long_description,
    long_description_content_type="text/markdown",
    url=URL,

    # Package configuration
    packages=find_packages(),
    package_data={
        "libfrankapy": ["py.typed"],
    },

    # Extension modules
    ext_modules=[create_extension()],
    cmdclass={"build_ext": CustomBuildExt},

    # Dependencies
    python_requires=">=3.8",
    install_requires=[
        "numpy>=1.19.0",
        "pybind11>=2.10.0",
    ],

    extras_require={
        "dev": [
            "pytest>=6.0",
            "pytest-cov",
            "black",
            "isort",
            "mypy",
            "sphinx",
            "sphinx-rtd-theme",
        ],
        "examples": [
            "matplotlib",
            "scipy",
        ],
    },

    # Metadata
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Science/Research",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: POSIX :: Linux",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: C++",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Scientific/Engineering :: Physics",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],

    keywords="robotics franka robot-control real-time panda",

    # Entry points
    entry_points={
        "console_scripts": [
            "libfrankapy-info=libfrankapy.utils:print_system_info",
        ],
    },

    # Include additional files
    include_package_data=True,
    zip_safe=False,

    # Project URLs
    project_urls={
        "Bug Reports": f"{URL}/issues",
        "Source": URL,
        "Documentation": f"{URL}/docs",
    },
)
