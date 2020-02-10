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
"""This module holds all entities related to task scheduling and stages.
"""
from .gazebo_proxy import GazeboProxy
from .process_manager import ProcessManager
from .ros_config import ROSConfig
from .server import Server
from .simulation_timer import SimulationTimer
from .stage import Stage
from .task_templates import TASK_GAZEBO_EMPTY_WORLD, TASK_ROS_CORE, \
    TASK_ROSBAG_RECORD_ALL, TASK_ROSBAG_RECORD_TOPICS, \
    TASK_RQT, TASK_RVIZ, TASK_SIMULATION_TF_MANAGER, TASK_SIMULATION_TIMER
from .task import Task


def is_roscore_running(ros_master_uri='http://localhost:11311'):
    """Return True if a `roscore` is running for the provided ROS URI

    > *Input parameters*

    * `ros_master_uri` (*type:* `str`, *default:*
     (`http://localhost:11311`)): The ROS URI of the target
      `roscore` node to be tested
    """
    import os
    import subprocess
    from ..log import PCG_ROOT_LOGGER
    env_variables = os.environ.copy()
    env_variables['ROS_MASTER_URI'] = ros_master_uri
    try:
        subprocess.check_output(
            ['rostopic', 'list'],
            env=env_variables
        )
        return True
    except subprocess.CalledProcessError as ex:
        PCG_ROOT_LOGGER.warning(
            'Error testing roscore, message={}'.format(ex))
        return False


def is_gazebo_running(ros_master_uri='http://localhost:11311'):
    """Return True if an instance of Gazebo is running
     and was initialized are a ROS node.

    > *Input parameters*

    * `ros_master_uri` (*type:* `str`, *default:*
     (`http://localhost:11311`)): The ROS URI of the target node to be tested
    """
    import os
    import subprocess
    from ..log import PCG_ROOT_LOGGER
    env_variables = os.environ.copy()
    env_variables['ROS_MASTER_URI'] = ros_master_uri
    try:
        output = subprocess.check_output(
            ['rostopic', 'list'],
            env=env_variables
        )
        output = output.decode('utf-8')
        return 'gazebo' in output
    except subprocess.CalledProcessError as ex:
        PCG_ROOT_LOGGER.warning(
            'Error testing Gazebo server, message={}'.format(ex))
        return False


def rosnode_exists(name, ros_master_uri='http://localhost:11311'):
    """Return True if a node with name defined by the input `name` is running

    > *Input parameters*

    * `name` (*type:* `str`): Name of the ROS node
    * `ros_master_uri` (*type:* `str`, *default:*
     (`http://localhost:11311`)): The ROS URI of the target node to be tested
    """
    import os
    import subprocess
    from ..log import PCG_ROOT_LOGGER
    env_variables = os.environ.copy()
    env_variables['ROS_MASTER_URI'] = ros_master_uri
    try:
        output = subprocess.check_output(
            ['rosnode', 'list'],
            env=env_variables
        )
        output = output.decode('utf-8')
        return name in output
    except subprocess.CalledProcessError as ex:
        PCG_ROOT_LOGGER.warning(
            'Error testing ROS master, message={}'.format(ex))
        return False


def get_rosparam_list(ros_master_uri='http://localhost:11311'):
    """Return the list of ROS parameter names in the parameter
    server running under the provided URI. If no `roscore` is
    running, return `None` instead.

    > *Input parameters*

    * `ros_master_uri` (*type:* `str`, *default:*
     (`http://localhost:11311`)): The ROS URI of the target node to be tested
    """
    import os
    import subprocess
    from ..log import PCG_ROOT_LOGGER
    env_variables = os.environ.copy()
    env_variables['ROS_MASTER_URI'] = ros_master_uri
    try:
        output = subprocess.check_output(
            ['rosparam', 'list'],
            env=env_variables
        )
        output = output.decode('utf-8').split('\n')
        return [elem for elem in output if elem != '']
    except subprocess.CalledProcessError as ex:
        PCG_ROOT_LOGGER.warning(
            'Error getting ROS parameter list, message={}'.format(ex))
        return None


def get_rostopic_list(ros_master_uri='http://localhost:11311'):
    """Return the list of ROS topic names under the provided URI.
    If no `roscore` is running, return `None` instead.

    > *Input parameters*

    * `ros_master_uri` (*type:* `str`, *default:*
     (`http://localhost:11311`)): The ROS URI of the target node to be tested
    """
    import os
    import subprocess
    from ..log import PCG_ROOT_LOGGER
    env_variables = os.environ.copy()
    env_variables['ROS_MASTER_URI'] = ros_master_uri
    try:
        output = subprocess.check_output(
            ['rostopic', 'list'],
            env=env_variables
        )
        output = output.decode('utf-8').split('\n')
        return [elem for elem in output if elem != '']
    except subprocess.CalledProcessError as ex:
        PCG_ROOT_LOGGER.warning(
            'Error getting ROS parameter list, message={}'.format(ex))
        return None


def get_rosservice_list(ros_master_uri='http://localhost:11311'):
    """Return the list of ROS service names under the provided URI.
    If no `roscore` is running, return `None` instead.

    > *Input parameters*

    * `ros_master_uri` (*type:* `str`, *default:*
     (`http://localhost:11311`)): The ROS URI of the target node to be tested
    """
    import os
    import subprocess
    from ..log import PCG_ROOT_LOGGER
    env_variables = os.environ.copy()
    env_variables['ROS_MASTER_URI'] = ros_master_uri
    try:
        output = subprocess.check_output(
            ['rosservice', 'list'],
            env=env_variables
        )
        output = output.decode('utf-8').split('\n')
        return [elem for elem in output if elem != '']
    except subprocess.CalledProcessError as ex:
        PCG_ROOT_LOGGER.warning(
            'Error getting ROS parameter list, message={}'.format(ex))
        return None


def set_rosparam(params, ros_master_uri='http://localhost:11311'):
    """Set the parameters from the input dictionary `params` in the ROS
    parameters server running under the provided ROS URI. If no `roscore`
    is running, return `None`.

    > *Input parameters*

    * `params` (*type:* `dict`): Table of parameters
    * `ros_master_uri` (*type:* `str`, *default:*
     (`http://localhost:11311`)): The ROS URI of the target node to be tested
    """
    import yaml
    import os
    import subprocess
    from ..log import PCG_ROOT_LOGGER
    assert isinstance(params, dict, 'Parameters must be provided as dict')
    env_variables = os.environ.copy()
    env_variables['ROS_MASTER_URI'] = ros_master_uri
    try:
        output = subprocess.check_output(
            ['rosparam', 'set', yaml.dump(params, default_flow_style=False)],
            env=env_variables
        )
        output = output.decode('utf-8').split('\n')
        return [elem for elem in output if elem != '']
    except subprocess.CalledProcessError as ex:
        PCG_ROOT_LOGGER.warning(
            'Error setting ROS parameters, message={}'.format(ex))
        return None


__all__ = [
    'GazeboProxy',
    'ProcessManager',
    'ROSConfig',
    'Server',
    'SimulationTimer',
    'Stage',
    'Task',
    'TASK_GAZEBO_EMPTY_WORLD',
    'TASK_ROS_CORE',
    'TASK_ROSBAG_RECORD_ALL',
    'TASK_ROSBAG_RECORD_TOPICS',
    'TASK_RQT',
    'TASK_RVIZ',
    'TASK_SIMULATION_TF_MANAGER',
    'TASK_SIMULATION_TIMER',
    'get_rosparam_list',
    'get_rosservice_list',
    'get_rostopic_list',
    'is_gazebo_running',
    'is_roscore_running'
]
