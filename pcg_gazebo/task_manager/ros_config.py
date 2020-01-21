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

from __future__ import print_function

import os
import socket
import time
import random
from ..log import create_logger

PORT_LOCK_FILE = 'pcg_lock'


class ROSConfig(object):
    def __init__(self, ros_host='localhost', ros_port=11311,
                 gazebo_host='localhost', gazebo_port=11345):

        self._logger = create_logger('pcg_ros_config')
        assert isinstance(ros_host, str)
        assert isinstance(gazebo_host, str)

        if ros_port is None:
            self._ros_port = self._get_random_open_port(15000, 20000)
        else:
            assert isinstance(ros_port, int)
            assert ros_port > 0
            self._ros_port = ros_port

        if gazebo_port is None:
            self._gazebo_port = self._get_random_open_port(25000, 30000)
        else:
            assert isinstance(gazebo_port, int)
            assert gazebo_port > 0
            self._gazebo_port = gazebo_port

        self._gazebo_host = gazebo_host
        self._ros_host = ros_host

    def __str__(self):
        msg = 'ROS_MASTER_URI=http://{}:{}, '.format(
            self._ros_host, self._ros_port)
        msg += 'GAZEBO_MASTER_URI=http://{}:{}'.format(
            self._gazebo_host, self._gazebo_port)
        return msg

    @property
    def ros_host(self):
        return self._ros_host

    @property
    def ros_port(self):
        return self._ros_port

    @ros_port.setter
    def ros_port(self, value):
        assert isinstance(value, int)
        self._ros_port = value

    @property
    def gazebo_host(self):
        return self._gazebo_host

    @property
    def gazebo_port(self):
        return self._gazebo_port

    @gazebo_port.setter
    def gazebo_port(self, value):
        assert isinstance(value, int)
        self._gazebo_port = value

    @property
    def ros_master_uri(self):
        return 'http://{}:{}\n'.format(self._ros_host, self._ros_port)

    @property
    def gazebo_master_uri(self):
        return 'http://{}:{}\n'.format(self._gazebo_host, self._gazebo_port)

    def _get_random_open_port(
            self,
            start=1000,
            end=3000,
            timeout=1,
            lock=False):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            port = random.randrange(start, end, 1)
            self._logger.info('Testing port %d' % port)
            if not self._is_port_open(port) and not self._is_port_locked(port):
                self._logger.info('Locking port %d' % port)
                return self._lock_port(port)
            self._logger.info('Port %d is locked' % port)
        raise RuntimeError(
            "Could not find any open port from %d to %d for %ds." %
            (start, end, timeout))

    def _is_port_open(self, port):
        return_code = 1
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            return_code = sock.connect_ex(('', port))
            sock.close()
        except Exception as exp:
            print(exp)
        return return_code == 0

    def _is_port_locked(self, port):
        return os.path.exists(self._get_port_lock_file(port))

    def _get_port_lock_file(self, port):
        return os.path.join('/tmp', '%s-%d.lock' % (PORT_LOCK_FILE, port))

    def _lock_port(self, port):
        with open(self._get_port_lock_file(port), 'a') as lock_file:
            lock_file.close()
        return port

    def choose_random_ros_port(self):
        self._ros_port = self._get_random_open_port(15000, 20000)

    def choose_random_gazebo_port(self):
        self._gazebo_port = self._get_random_open_port(15000, 20000)

    def unlock_port(self, port=None):
        if os.path.exists(self._get_port_lock_file(port)):
            os.remove(self._get_port_lock_file(port))
        self._logger.info('Unlocking port %d' % port)

    def set_env_variables(self):
        os.environ['ROS_MASTER_URI'] = 'http://{}:{}'.format(
            self._ros_host, self._ros_port)
        os.environ['GAZEBO_MASTER_URI'] = 'http://{}:{}'.format(
            self._gazebo_host, self._gazebo_port)

    def get_env_variables(self):
        env_variables = os.environ.copy()
        env_variables['ROS_MASTER_URI'] = 'http://{}:{}'.format(
            self._ros_host, self._ros_port)
        env_variables['GAZEBO_MASTER_URI'] = 'http://{}:{}'.format(
            self._gazebo_host, self._gazebo_port)
        return env_variables
