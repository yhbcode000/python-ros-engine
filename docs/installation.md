# Installation Guide

## Prerequisites

- Python 3.8 or higher
- pip (Python package installer)

## Installation Methods

### From PyPI (Recommended)

To install the Python ROS Engine from PyPI, simply run:

```bash
pip install python-ros-engine
```

### From Source

If you want to install from source, first clone the repository:

```bash
git clone https://github.com/yhbcode000/python-ros-engine.git
cd python-ros-engine
```

Then install the package:

```bash
pip install .
```

### Development Installation

For development purposes, you can install the package in editable mode with test dependencies:

```bash
pip install -e .[test]
```

This will install all dependencies needed for development and testing.

## Verifying Installation

After installation, you can verify that the package is correctly installed by running:

```bash
python -c "import pyros2; print('Python ROS Engine installed successfully')"
```

## Running Tests

To run the tests, you'll need to install the test dependencies:

```bash
pip install -e .[test]
```

Then run the tests with pytest:

```bash
python -m pytest tests/ -v
```