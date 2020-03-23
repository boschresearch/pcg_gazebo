# Advanced installation

[![PyPI](https://img.shields.io/pypi/v/pcg-gazebo)](https://pypi.org/project/pcg-gazebo/)
![PyPI - Python Version](https://img.shields.io/pypi/pyversions/pcg-gazebo)

The list of Python dependencies can be retrieved by cloning this package and running the `setup.py` script as follows:

```bash
python setup.py --list-all
```

The package requires some non-Python libraries that are not resolved using the usual Python installation.
Running the following command installs the missing dependencies. 

```
sudo apt install libspatialindex-dev pybind11-dev libgeos-dev
```

## As a `pip` package

```bash
pip install pcg-gazebo
```

It is highly recommended to use Python 3.x. 
To install also dependencies necessary to run the `pcg-gazebo` package in a Jupyter notebook, run 

```bash
pip install pcg-gazebo[all]
```

## ROS

The `pcg-gazebo` package is available as a ROS dependency that can be resolved by `rosdep`. 
It is necessary to add the following tag in the `package.xml` file of your ROS package as 

```xml
<exec_depend>python3-pcg-gazebo-pip</exec_depend>
```
for Python 3.x, and 

```xml
<exec_depend>python-pcg-gazebo-pip</exec_depend>
```

for Python 2.x.