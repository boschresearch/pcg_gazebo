# `pcg_gazebo`: A Python package for rapid-prototyping and scripting of simulations for Gazebo

[![Build Status](https://travis-ci.org/boschresearch/pcg_gazebo.svg?branch=master)](https://travis-ci.org/boschresearch/pcg_gazebo)
[![GitHub issues](https://img.shields.io/github/issues/boschresearch/pcg_gazebo_pkgs.svg)](https://github.com/boschresearch/pcg_gazebo/issues)
[![License](https://img.shields.io/badge/license-Apache%202-blue.svg)](https://github.com/boschresearch/pcg_gazebo/blob/master/LICENSE)

The `pcg_gazebo` Python package is an Open Source
Project extending the simulation capabilities of the robotics simulator [Gazebo](http://gazebosim.org/)
for automation and scripting of Gazebo simulations.

Visit the [documentation page](https://boschresearch.github.io/pcg_gazebo_pkgs/) for more information.

## Purpose of the project

This software is a research prototype.

The software is not ready for production use. It has neither been developed nor
tested for a specific use case. However, the license conditions of the
applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software
fulfills your requirements and adjust it according to any applicable safety
standards (e.g. ISO 26262).

## Installation

### From source

First install some non-Python dependencies

```
sudo apt install libspatialindex-dev pybind11-dev libgeos-dev
```

Then clone the repository and install it using `pip`

```
git clone https://github.com/boschresearch/pcg_gazebo.git
pip install .
```

## License

Procedural Generation for Gazebo is open-sourced under the Apache-2.0 license. See the [LICENSE](https://github.com/boschresearch/pcg_gazebo/blob/master/LICENSE) file for details.

For a list of other open source components included in Procedural Generation for Gazebo package, see the file [3rd-party-licenses](https://github.com/boschresearch/pcg_gazebo/blob/master/3rd-party-licenses.md).
