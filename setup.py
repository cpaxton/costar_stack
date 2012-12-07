#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['xdot'],
    package_dir={'': 'src'},
    scripts=['scripts/dot_viewer.py']
)

setup(**d)
