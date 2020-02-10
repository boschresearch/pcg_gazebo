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

TASK_ROS_CORE = dict(
    name='roscore',
    command='roscore -v --port {port}',
    stage='roscore',
    type=None,
    has_gazebo=False,
    params=dict(
        port=11311
    )
)

TASK_SIMULATION_TIMER = dict(
    name='simulation_timer',
    command='pcg-simulation-timer --timeout {timeout}',
    type=None,
    stage='gazebo',
    has_gazebo=False,
    required=True,
    params=dict(
        timeout=0
    )
)

TASK_GAZEBO_EMPTY_WORLD = dict(
    name='gazebo',
    command='roslaunch gazebo_ros empty_world.launch',
    type='roslaunch',
    stage='gazebo',
    has_gazebo=True,
    params=dict(
        paused=False,
        use_sim_time=True,
        extra_gazebo_args='',
        gui=True,
        recording=False,
        headless=False,
        debug=False,
        physics='ode',
        verbose=True,
        world_name='worlds/empty.world',
        respawn_gazebo=False
    )
)

TASK_RVIZ = dict(
    name='rviz',
    command='rosrun rviz rviz',
    type=None,
    has_gazebo=False,
    params=dict()
)

TASK_RQT = dict(
    name='rqt',
    command='rosrun rqt_gui rqt_gui',
    type=None,
    has_gazebo=False,
    params=dict()
)

TASK_ROSBAG_RECORD_TOPICS = dict(
    name='rosbag_record',
    command='rosbag record -O {output_name} {topics}',
    type=None,
    has_gazebo=False,
    params=dict(
        output_name='recording.bag',
        topics=''
    )
)

TASK_ROSBAG_RECORD_ALL = dict(
    name='rosbag_record',
    command='rosbag record -O {output_name} -a',
    type=None,
    has_gazebo=False,
    params=dict(
        output_name='recording.bag'
    )
)

TASK_SIMULATION_TF_MANAGER = dict(
    name='gazebo',
    command='roslaunch pcg_libraries start_pcg_simulation_tf_manager.launch',
    type='roslaunch',
    stage='gazebo',
    has_gazebo=False
)
