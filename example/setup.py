#!/usr/bin/env python

import os
from pathlib import Path
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

p = Path(os.path.dirname(os.path.realpath(__file__))).parent.absolute()

d = generate_distutils_setup(
    packages=['example'],
    package_dir={'': str(p)}
)

setup(**d)
