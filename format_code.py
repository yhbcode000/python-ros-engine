#!/usr/bin/env python3
"""Code formatting and linting script."""

import subprocess
import sys


def run_command(cmd, description):
    """Run a command and print its output."""
    print(f"Running {description}...")
    try:
        result = subprocess.run(
            cmd, shell=True, check=True, capture_output=True, text=True
        )
        if result.stdout:
            print(result.stdout)
        print(f"✅ {description} completed successfully.\n")
    except subprocess.CalledProcessError as e:
        print(f"❌ {description} failed with exit code {e.returncode}.")
        if e.stdout:
            print(e.stdout)
        if e.stderr:
            print(e.stderr)
        sys.exit(e.returncode)


def main():
    """Run all formatting and linting tools."""
    print("Starting code formatting and linting process...\n")

    # Run isort to sort imports
    run_command("isort .", "isort (import sorting)")

    # Run black to format code
    run_command("black .", "black (code formatting)")

    # Run flake8 to check for style issues
    run_command("flake8", "flake8 (style checking)")

    print("All formatting and linting tools completed successfully!")


if __name__ == "__main__":
    main()
