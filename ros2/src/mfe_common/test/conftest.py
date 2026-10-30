"""Make the package importable when pytest runs from the source tree (colcon installs it otherwise)."""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
