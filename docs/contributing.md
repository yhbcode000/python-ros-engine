# 🤝 Contributing to Python ROS Engine

Thank you for your interest in contributing to the Python ROS Engine! We welcome contributions from the community to help improve this pure Python implementation of ROS2 functionality.

## 📋 Table of Contents

1. [Code of Conduct](#code-of-conduct)
2. [🚀 Getting Started](#getting-started)
3. [💡 How to Contribute](#how-to-contribute)
4. [⚙️ Development Setup](#development-setup)
5. [💻 Coding Standards](#coding-standards)
6. [🧪 Testing](#testing)
7. [📖 Documentation](#documentation)
8. [🔄 Pull Request Process](#pull-request-process)
9. [🐛 Reporting Bugs](#reporting-bugs)
10. [✨ Requesting Features](#requesting-features)

## 🎯 Code of Conduct

This project adheres to a Code of Conduct adapted from the [Contributor Covenant](https://www.contributor-covenant.org/). By participating, you are expected to uphold this code. Please report unacceptable behavior to pyros@example.com.

## 🚀 Getting Started

1. Fork the repository on GitHub
2. Clone your fork locally
3. Set up the development environment (see [Development Setup](#development-setup))

## 💡 How to Contribute

There are several ways you can contribute to this project:

- Report bugs and issues
- Suggest new features
- Improve documentation
- Write code to implement new features or fix bugs
- Review pull requests from other contributors

## ⚙️ Development Setup

1. Ensure you have Python 3.8 or higher installed
2. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```
3. Install the package in development mode:
   ```bash
   pip install -e .
   ```
4. Install test dependencies:
   ```bash
   pip install -e .[test]
   ```

## 💻 Coding Standards

We follow the PEP 8 style guide for Python code. Additionally:

- Use type hints for all function parameters and return values
- Write docstrings for all public classes and methods using the Google Python Style Guide
- Keep functions and classes focused on a single responsibility
- Use dataclasses for message types when possible
- Ensure code is readable and well-commented where necessary

## 🧪 Testing

All contributions must include appropriate tests. We use pytest for testing.

To run all tests:
```bash
python -m pytest tests/ -v
```

To run tests with coverage:
```bash
pip install pytest-cov
python -m pytest tests/ --cov=src/ --cov-report=html
```

## 📖 Documentation

We maintain documentation in the `docs/` directory. When adding new features or modifying existing functionality, please update the relevant documentation files.

- API documentation: `docs/api.md`
- Usage guide: `docs/usage.md`
- Bridging documentation: `docs/bridging.md`

Documentation can be built and served locally using the `build_docs.py` script. See [project_structure.md](../project_structure.md) for details on how to use this script.

## 🔄 Pull Request Process

1. Ensure any install or build dependencies are removed before the end of the layer when doing a build
2. Update the README.md and documentation files with details of changes to the interface, including new environment variables, exposed ports, useful file locations, and container parameters
3. Increase the version numbers in any examples files and the README.md to the new version that this Pull Request would represent. The versioning scheme we use is [SemVer](https://semver.org/)
4. Your Pull Request will be reviewed by maintainers, who may request changes
5. Once approved, your Pull Request will be merged

## 🐛 Reporting Bugs

Please use the GitHub issue tracker to report bugs. When filing an issue, please include:

- A clear and descriptive title
- Steps to reproduce the bug
- Expected behavior
- Actual behavior
- Any relevant error messages or logs
- Information about your environment (Python version, OS, etc.)

## ✨ Requesting Features

We welcome feature requests! Please use the GitHub issue tracker and include:

- A clear and descriptive title
- A detailed description of the proposed feature
- The motivation for the feature
- How the feature would be used