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
import datetime
import psutil
import signal
from time import sleep
from threading import Timer, Thread
from ..log import create_logger, get_log_dir


class Task(object):
    def __init__(self, task_name, command, params, config, has_gazebo=False,
                 type=None, required=False, process_timeout=None,
                 task_killed_callback=None, stage=None, output_log_dir=None,
                 simulation_timeout=None):
        assert isinstance(task_name, str)
        assert len(task_name) > 0
        assert callable(task_killed_callback), 'Callback function is not ' \
            'callable'
        assert isinstance(has_gazebo, bool), 'Invalid has_gazebo flag'
        if simulation_timeout is not None:
            assert simulation_timeout >= 0, 'Simulation timeout must ' \
                'be equal or greater than zero'

        self._stage = stage
        self._task_name = task_name
        self._logger = create_logger('task_{}'.format(task_name),
                                     output_dir=output_log_dir)
        self._has_gazebo = has_gazebo
        self._process_config = config
        self._command = command
        self._type = type
        self._is_running = False
        self._log_filename = None
        self._required = required
        self._task_killed_callback = task_killed_callback
        self._process_timeout = None
        self._process_timer = None
        self._process_monitor = None
        self._simulation_timeout = simulation_timeout

        # Create the log directory, if it doesn't exist already
        self._log_dir = get_log_dir() \
            if output_log_dir is None else output_log_dir
        if not os.path.isdir(self._log_dir):
            os.makedirs(self._log_dir)

        cmd_elems = self._command.split()

        if cmd_elems[0] == 'roslaunch':
            self._type = 'roslaunch'
        elif cmd_elems[0] == 'rosrun':
            self._type = 'rosrun'

        if self._type == 'roslaunch' and len(params):
            # For roslaunch no string substitution is necessary, the parameters
            # included in the params dict will be added to the command line
            # as
            #   param_key:=param_value
            for param_name in params:
                self._command += ' {}:='.format(param_name)
                if isinstance(params[param_name], bool):
                    self._command += '{}'.format(
                        'true' if params[param_name] else 'false')
                else:
                    self._command += '{}'.format(params[param_name])
        else:
            # For all other cases, if there are any parameters, it is expected
            # that the parameter can be found in the command string as
            #   {param_name}
            # and they will then be replaced by the assigned value
            for param_name in params:
                self._command = self._command.replace(
                    '{' + param_name + '}', str(params[param_name]))

        self._logger.info('Task created')
        self._logger.info('\tName: {}'.format(self._task_name))
        self._logger.info('\tCommand: {}'.format(self._command))
        self._logger.info(
            '\tROS network configuration: {}'.format(
                self._process_config))
        self._logger.info('\tRuns Gazebo?: {}'.format(self._has_gazebo))
        self._logger.info('\tType: {}'.format(self._type))
        self._logger.info('\tRequired task? {}'.format(self._required))

        if process_timeout is not None:
            if process_timeout > 0:
                self._process_timeout = process_timeout
                self._logger.info('\tProcess timeout: {} seconds'.format(
                    self._process_timeout))

        self._process = None
        self._process_children = None

    def __del__(self):
        self._logger.info('Deleting task <{}>'.format(self._task_name))
        self.kill()

    @property
    def has_gazebo(self):
        return self._has_gazebo

    @property
    def log_filename(self):
        return self._log_filename

    @property
    def log_dir(self):
        return self._log_dir

    @property
    def required(self):
        return self._required

    @property
    def process_timeout(self):
        return self._process_timeout

    @property
    def simulation_timeout(self):
        return self._simulation_timeout

    def _process_monitor(self):
        while self.is_running() and self._process is not None:
            self._logger.info('process monitor...')
            sleep(1)

        self.kill()

    def _simulation_timeout(self):
        self._logger.info(
            'Applying network configuration for simulation timeout')
        self._process_config.set_env_variables()

        while self.is_running() and self._process is not None:
            self._logger.info('Simulation timeout')

    def _on_terminate(self, process):
        try:
            if psutil.pid_exists(process.pid):
                self._logger.warning('Process {} <{}> terminated with exit'
                                     ' code {}'.format(process.pid,
                                                       process.name(),
                                                       process.returncode))
            else:
                self._logger.warning('Process {} already '
                                     'terminated'.format(process.pid))
        except Exception as e:
            self._logger.error(
                'Error in on_terminate function, message=' + str(e))

    def is_running(self):
        if self._process is None:
            return False
        for proc in self.get_all_processes():
            if not psutil.pid_exists(proc.pid):
                self._logger.info(
                    'Task {} is not running'.format(
                        self._task_name))
                return False
        return True

    def kill(self):
        if self._process is None:
            self._logger.warning(
                'Task <{}> - Process object is '
                'invalid'.format(self._task_name))
            return
        if len(self._process_children) == 0:
            self._logger.warning(
                'Task <{}> - No children processes'
                ' found'.format(self._task_name))
            return
        try:
            self._logger.warning(
                'Task <{}> - Killing process '
                'tree...'.format(self._task_name))

            for p in self.get_all_processes():
                if psutil.pid_exists(p.pid):
                    self._logger.warning('Sending SIGINT to child '
                                         'process id=%d', p.pid)
                    p.send_signal(signal.SIGINT)
                    if not psutil.pid_exists(p.pid):
                        self._logger.warning('Child process %d '
                                             'successfully terminated',
                                             p.pid)
                    else:
                        self._logger.warning('Child process %d still '
                                             'running', p.pid)
                else:
                    self._logger.warning('Child process %d is not alive',
                                         p.pid)

            gone, alive = psutil.wait_procs(
                self.get_all_processes(),
                timeout=None,
                callback=self._on_terminate)

            self._logger.warning(
                'Kill processes=\n\t - Gone={}\n\t - '
                'Alive{}'.format(str(gone), str(alive)))

            self._process_timeout_triggered = True
            self._logger.info(
                'Task <{}> - PROCESS TIMEOUT - '
                'finishing process...'.format(
                    self._task_name))
        except Exception as ex:
            self._logger.warning('Error occurred while killing processes, '
                                 'message=%s' % str(ex))

        self._process = None
        self._process_children = None
        self._logger.info(
            'Task <{}> - Process objects were reset'.format(self._task_name))

        if callable(self._task_killed_callback):
            self._logger.info(
                'Calling task <{}> end callback function'.format(
                    self._task_name))
            self._task_killed_callback(self._task_name)
            self._logger.info(
                'Task <{}> - Callback finished'.format(self._task_name))

        self._logger.info(
            'Task <{}> - Processes finished'.format(self._task_name))

    def wait(self, timeout=None):
        if self._process is not None:
            try:
                exit_code = self._process.wait(timeout)
                self._logger.info(
                    'Task <{}> finished, exit_code={}'.format(
                        self._task_name, exit_code))
                return True
            except psutil.TimeoutExpired as ex:
                self._logger.info(
                    'Task <{}> still running, message={}'.format(
                        self._task_name, str(ex)))
                return False
        return False

    def get_all_processes(self):
        proc = psutil.Process(self._process.pid)
        process_children = proc.children(recursive=True)
        process_children.append(proc)
        return process_children

    def has_terminated(self):
        if self._process is not None:
            process_children = self.get_all_processes()
            for p in process_children:
                if not p.is_running():
                    return False
        return True

    def run(self):
        if self.is_running():
            self._logger.warning('Task is already running')
            return
        # Create a log directory for this task's logs
        task_log_dir = os.path.join(self._log_dir, self._task_name)
        if not os.path.isdir(task_log_dir):
            os.makedirs(task_log_dir)

        # Create file for log
        timestamp = datetime.datetime.now().isoformat()
        timestamp = timestamp.replace(':', '_')
        self._log_filename = os.path.join(
            task_log_dir,
            '{}_process_log_{}.log'.format(timestamp, self._task_name))
        logfile = open(self._log_filename, 'a')

        env_variables = self._process_config.get_env_variables()
        # Set ROS_HOME to store the log files from the ROS processes
        env_variables['ROS_HOME'] = os.path.join(task_log_dir, 'ros')
        if not os.path.isdir(env_variables['ROS_HOME']):
            os.makedirs(env_variables['ROS_HOME'])

        self._logger.info('Running command=' + self._command)
        self._logger.info('ROS network configuration=' +
                          str(self._process_config))
        self._logger.info('Process log file=' + self._log_filename)
        self._logger.info('Directory for log files=' + self._log_dir)

        if self._process_timeout is not None:
            self._logger.info(
                'Process timeout={} seconds'.format(
                    self._process_timeout))
            self._process_timer = Timer(self._process_timeout, self.kill)

        self._process = psutil.Popen(
            self._command, shell=True, stdout=logfile, stderr=logfile,
            env=env_variables)

        # Get the process instance
        proc = psutil.Process(self._process.pid)
        # Get all the children processes
        self._process_children = proc.children(recursive=True)
        self._process_children.append(proc)

        if self._process_timer is not None:
            self._process_timer.start()
        else:
            self._logger.info('Starting process monitor')
            self._process_monitor = Thread(target=self._process_monitor)
            self._process_monitor.daemon = True
            self._process_monitor.run()

        self._logger.info('Process created (Name={}, PID={})'.format(
            proc.name(), proc.pid))
