# `pcg_gazebo`: A Python package for rapid-prototyping and scripting of simulations for Gazebo

[![Build Status](https://travis-ci.org/boschresearch/pcg_gazebo.svg?branch=master)](https://travis-ci.org/boschresearch/pcg_gazebo)
[![GitHub issues](https://img.shields.io/github/issues/boschresearch/pcg_gazebo_pkgs.svg)](https://github.com/boschresearch/pcg_gazebo/issues)
[![License](https://img.shields.io/badge/license-Apache%202-blue.svg)](https://github.com/boschresearch/pcg_gazebo/blob/master/LICENSE)
[![PyPI](https://img.shields.io/pypi/v/pcg-gazebo)](https://pypi.org/project/pcg-gazebo/)
![PyPI - Python Version](https://img.shields.io/pypi/pyversions/pcg-gazebo)

The `pcg_gazebo` Python package is an Open Source
Project extending the simulation capabilities of the robotics simulator [Gazebo](http://gazebosim.org/)
for automation and scripting of Gazebo simulations.

Visit the [documentation page](https://boschresearch.github.io/pcg_gazebo/) for more information.

## Purpose of the project

This software is a research prototype.

The software is not ready for production use. It has neither been developed nor
tested for a specific use case. However, the license conditions of the
applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software
fulfills your requirements and adjust it according to any applicable safety
standards (e.g. ISO 26262).

## Installation

### Using `pip`

You can install the `pcg-gazebo` package using `pip` as follows

```bash
pip install pcg-gazebo
```

you still might need to install some extra dependencies that cannot
be handled by `pip` as

```
sudo apt install libspatialindex-dev pybind11-dev libgeos-dev
```

> The default installation does not include `rospy` dependencies that
> are not available as a `pip` package. They have to be installed 
> separately for certain submodules to work, such as `pcg_gazebo.task_manager`.

### From source

First install some non-Python dependencies

```
sudo apt install libspatialindex-dev pybind11-dev libgeos-dev
```

Then clone the repository and install it using `pip`

```
git clone https://github.com/boschresearch/pcg_gazebo.git
cd pcg_gazebo
pip install .
```

### Using the package with ROS and Gazebo

Certain functionalities as the Gazebo proxy, task manager and model spawning 
are only available if `rospy` is installed.
The lack of Gazebo and `rospy` only restricts functionalities related to 
interaction with the simulation in runtime and the creation of ROS-related 
tasks.

At the moment, ROS 2 is **not** supported for this purpose. 
To install ROS `melodic`, follow these [installation instructions](https://wiki.ros.org/melodic/Installation/Ubuntu) and install `ros-melodic-desktop-full`
or separately install Gazebo and its ROS bindings as

```
sudo apt install gazebo9 libgazebo9-dev ros-melodic-gazebo-*
```

## License

Procedural Generation for Gazebo is open-sourced under the Apache-2.0 license. See the [LICENSE](https://github.com/boschresearch/pcg_gazebo/blob/master/LICENSE) file for details.

For a list of other open source components included in Procedural Generation for Gazebo package, see the file [3rd-party-licenses](https://github.com/boschresearch/pcg_gazebo/blob/master/3rd-party-licenses.md).
