#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['sot_ros_api', 'sot_ros_api.sot_robot', 'sot_ros_api.utilities'],
   package_dir={'': 'src'}
)

setup(**d)
