## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['mpc_lib','sac_lib','hlt_planner','replay_buffer','test_env','utils'],
    package_dir={'': 'src'})

setup(**setup_args)
