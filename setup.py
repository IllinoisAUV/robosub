## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
"""
catkin default setup.py
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

SETUP_ARGS = generate_distutils_setup(
    # Add new subdirectories here
    packages=['missions'],
    package_dir={'': 'src'},
)

setup(**SETUP_ARGS)
