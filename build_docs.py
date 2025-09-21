#!/usr/bin/env python3

"""
Script to build and serve the documentation site.
"""

import subprocess
import sys


def build_docs():
    """Build the documentation site."""
    print("Building documentation site...")
    try:
        subprocess.run(["mkdocs", "build"], check=True)
        print("Documentation site built successfully!")
    except subprocess.CalledProcessError as e:
        print(f"Error building documentation: {e}")
        sys.exit(1)


def serve_docs():
    """Serve the documentation site locally."""
    print("Starting local documentation server...")
    try:
        subprocess.run(["mkdocs", "serve"], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error serving documentation: {e}")
        sys.exit(1)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python build_docs.py [build|serve]")
        sys.exit(1)

    command = sys.argv[1]

    if command == "build":
        build_docs()
    elif command == "serve":
        serve_docs()
    else:
        print("Invalid command. Use 'build' or 'serve'")
        sys.exit(1)
