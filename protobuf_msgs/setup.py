#!/usr/bin/env python
import os
from pathlib import Path
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['protobuf_msgs'],
    package_dir={'': '..'}
)

setup(**d)
