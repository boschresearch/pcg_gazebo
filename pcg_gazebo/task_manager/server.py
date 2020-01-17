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
from .process_manager import ProcessManager
from ..log import PCG_ROOT_LOGGER


class Server(object):
    def __init__(self):
        self._simulations = dict()
        self._logger = PCG_ROOT_LOGGER
        self._logger.info('PCG server running')

    @property
    def simulations(self):
        return self._simulations

    @property
    def simulation_names(self):
        return self._simulations.keys()

    def create_simulation(self, label='default', ros_host='localhost',
                          ros_port=None, gazebo_host='localhost',
                          gazebo_port=None, anonymous=False,
                          output_log_dir=None):
        assert isinstance(label, str), 'Simulation label must be a string'
        assert len(label) > 0, 'Simulation label should not be empty'

        if label in self._simulations:
            self._logger.error(
                'Simulation with label {} already exists'.format(label))
            return False

        self._simulations[label] = ProcessManager(
            label=label,
            ros_host=ros_host,
            ros_port=ros_port,
            gazebo_host=gazebo_host,
            gazebo_port=gazebo_port,
            output_log_dir=output_log_dir
        )

        return True

    def is_simulation_running(self, tag):
        if tag in self._simulations:
            return self._simulations[tag].is_running()
        else:
            return False

    def has_simulation(self, label):
        return label in self._simulations

    def remove_simulation(self, label):
        self._logger.info('Removing simulation={}'.format(label))
        if label in self._simulations:
            self._simulations[label].kill_all_tasks()
            self._simulations[label].ros_config.unlock_port(
                self._simulations[label].ros_config.ros_port)
            self._simulations[label].ros_config.unlock_port(
                self._simulations[label].ros_config.gazebo_port)
            self._simulations[label].__del__()
            del self._simulations[label]

    def get_simulation(self, label):
        if label not in self._simulations:
            return None

        return self._simulations[label]

    def run_simulations(self, sim_names):
        pass
