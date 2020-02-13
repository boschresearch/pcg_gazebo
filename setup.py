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

requirements_required = set([
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
    'trimesh[easy]',
    'networkx',
    'pycollada==0.6',
    'triangle',
    'python-fcl',
    'jsonschema',
    'scikit-image',
    'rospkg'
])

requirements_examples = requirements_required.union(['jupyterlab'])

requirements_test = requirements_required.union(set([
    'nbconvert', 
    'pytest', 
    'pytest-console-scripts']))

requirements_docs = requirements_required.union(set([
    'nbconvert', 
    'pydoc-markdown', 
    'pypandoc', 
    'mkdocs-material']))

requirements_all = requirements_required.union(set([
    'jupyterlab', 
    'nbconvert', 
    'pytest', 
    'pytest-console-scripts', 
    'pydoc-markdown', 
    'pypandoc', 
    'mkdocs-material']))

# `python setup.py --list-all > requirements.txt`
if '--list-all' in sys.argv:
    print('\n'.join(requirements_all))
    exit()
elif '--list-docs' in sys.argv:
    print('\n'.join(requirements_docs))
    exit()
elif '--list-examples' in sys.argv:
    print('\n'.join(requirements_examples))
    exit()
elif '--list-easy' in sys.argv:
    print('\n'.join(requirements_test))
    exit()

# Set the README.md page as long description
README = ''
try:
    import pypandoc
    README = pypandoc.convert('README.md', 'rst')    
except (IOError, ImportError):
    with open('README.md') as f:
        README = f.read()

setup(
    name='pcg_gazebo',
    version=__version__,
    description='A Python package for rapid-prototyping and scripting of simulations for Gazebo',
    long_description=README,
    long_description_content_type='text/x-rst',
    author='Musa Morena Marcusso Manhaes',
    author_email='musa.marcusso@de.bosch.com',
    maintainer='Musa Morena Marcusso Manhaes',
    maintainer_email='musa.marcusso@de.bosch.com',
    license='Apache-2.0',
    classifiers=[
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7'
    ],
    url='https://github.com/boschresearch/pcg_gazebo',
    keywords='gazebo ros simulation robotics sdf urdf robot',
    packages=[
        'pcg_gazebo',
        'pcg_gazebo.generators',
        'pcg_gazebo.generators.components',
        'pcg_gazebo.generators.constraints',
        'pcg_gazebo.generators.engines',
        'pcg_gazebo.parsers',
        'pcg_gazebo.parsers.gazebo',
        'pcg_gazebo.parsers.sdf',
        'pcg_gazebo.parsers.sdf_config',
        'pcg_gazebo.parsers.urdf',
        'pcg_gazebo.parsers.types',
        'pcg_gazebo.parsers.urdf',
        'pcg_gazebo.simulation',
        'pcg_gazebo.simulation.physics',
        'pcg_gazebo.simulation.properties',
        'pcg_gazebo.simulation.sensors',
        'pcg_gazebo.task_manager',        
    ],
    package_data={
        '': ['templates/*.jinja'],
        'pcg_gazebo': ['simulation/properties/resources/xkcd_rgb.txt']
    },
    scripts=[
        'scripts/pcg-generate-occupancy-map',
        'scripts/pcg-generate-world',
        'scripts/pcg-list-gazebo-models',
        'scripts/pcg-preview-sdf',
        'scripts/pcg-preview-urdf',
        'scripts/pcg-print-xml-element',
        'scripts/pcg-process-jinja-template',
        'scripts/pcg-run-model-factory',
        'scripts/pcg-sdf2urdf',
        'scripts/pcg-sdflint',
        'scripts/pcg-simulation-timer',
        'scripts/pcg-spawn-sdf-model',
        'scripts/pcg-start-gazebo-world',
        'scripts/pcg-urdf2sdf',
        'scripts/pcg-urdflint',
        'scripts/pcg-view-gazebo-model',
        'scripts/pcg-view-mesh'
    ],
    install_requires=list(requirements_required),
    extras_require=dict(
        all=list(requirements_all),
        examples=list(requirements_examples),
        test=list(requirements_test),
        docs=list(requirements_docs)
    )
)
