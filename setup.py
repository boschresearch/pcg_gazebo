#!/usr/bin/env python
# Copyright (c) 2019 - The Procedural Generation for Gazebo authors
# For information on the respective copyright owner see the NOTICE file
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
from setuptools import setup

version_file = os.path.join(os.path.dirname(__file__), 'pcg_gazebo/version.py')

with open(version_file, 'r') as f:
    __version__ = eval(f.read().strip().split('=')[-1])

requirements_required = [
    'lxml',
    'numpy',
    'psutil',
    'yasha',
    'xmltodict',
    'Jinja2',
    'Shapely',
    'bokeh',
    'matplotlib',
    'descartes',
    'PyYAML',
    'trimesh[all]',
    'networkx',
    'pycollada==0.6',
    'rospkg',
]

requirements_ros = ['rospkg']

requirements_test = ['pytest']

setup(
    name='pcg_gazebo',
    version=__version__,
    description='A Python package for rapid-prototyping and scripting of simulations for Gazebo',
    author='Musa Morena Marcusso Manhaes',
    author_email='musa.marcusso@de.bosch.com',
    maintainer='Musa Morena Marcusso Manhaes',
    maintainer_email='musa.marcusso@de.bosch.com',
    license='Apache-2.0',
    url='https://github.com/boschresearch/pcg_gazebo',
    keywords='gazebo ros simulation robotics',
    packages=[
        'pcg_gazebo'
    ],
    package_data={
        '': ['*.sdf.jinja']
    },
    install_requires=requirements_required,
    extras_require=dict(
        test=requirements_test
    )
)