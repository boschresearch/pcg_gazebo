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

from time import time
from threading import Thread
from ..log import create_logger
try:
    import rospy
    from rosgraph_msgs.msg import Clock
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class SimulationTimer(Thread):
    def __init__(
            self,
            simulation_timeout=0,
            start_gazebo_timeout=60,
            ros_config=None,
            output_log_dir=None,
            callback=None):
        assert ROS_AVAILABLE, 'ROS components could not be loaded'
        Thread.__init__(self)
        assert ros_config is not None, 'ROS network configuration cannot' \
            ' be null'
        assert start_gazebo_timeout >= 0, 'Start Gazebo timeout must be ' \
            'equal or greater than zero'
        assert simulation_timeout >= 0, 'Simulation timeout must be equal' \
            ' or greater than zero'
        if callback is not None:
            assert callable(callback), 'Invalid callback function'
        self._logger = create_logger(name='simulation_timeout',
                                     output_dir=output_log_dir)

        self._gazebo_clock = 0
        self._simulation_timeout = simulation_timeout
        self._start_gazebo_timeout = start_gazebo_timeout
        self._ros_config = ros_config
        self._success = True
        self._callback = callback

        rospy.init_node('simulation_timer', anonymous=True)

        self._sim_time_sub = rospy.Subscriber(
            'clock', Clock, self._clock_callback)

        self._logger.info(
            'Simulation timeout configured, '
            'simulation_timeout={}, '
            'start_gazebo_timeout={}'.format(
                self._simulation_timeout,
                self._start_gazebo_timeout))

    def _clock_callback(self, msg):
        self._gazebo_clock = rospy.Time(
            msg.clock.secs, msg.clock.nsecs).to_sec()

    def run(self):
        self._logger.info(
            'Starting simulation timer - Timeout = {} s'.format(
                self._simulation_timeout))

        rate = rospy.Rate(100)

        start_process_timeout = time()

        while self._gazebo_clock < self._simulation_timeout:
            rate.sleep()
            if self._gazebo_clock == 0:
                if time() - start_process_timeout > self._start_gazebo_timeout:
                    self._logger.error(
                        'Clock was not initialized for {} seconds'.format(
                            self._start_gazebo_timeout))
                    self._success = False
                    break
            if rospy.is_shutdown():
                self._logger.error('ROS master was killed!')
                self._success = False
                break

        if self._success:
            self._logger.info('Simulation timeout was triggered!')
        else:
            self._logger.error('Simulation timeout encountered an error')

        if self._callback is not None:
            self._logger.info('Calling simulation timeout callback')
            self._callback()
