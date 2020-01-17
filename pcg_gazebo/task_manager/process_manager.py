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
import signal
from time import sleep, time
from threading import Thread, Event
from collections import OrderedDict
from .ros_config import ROSConfig
from ..log import create_logger, get_log_dir
from .gazebo_proxy import GazeboProxy
from .task import Task
from .stage import Stage
from .task_templates import TASK_ROS_CORE, \
    TASK_GAZEBO_EMPTY_WORLD, TASK_RVIZ, \
    TASK_SIMULATION_TIMER, TASK_ROSBAG_RECORD_TOPICS, \
    TASK_RQT


class ProcessManager(object):
    def __init__(self, label='default', ros_host='localhost', ros_port=11311,
                 gazebo_host='localhost', gazebo_port=11345,
                 output_log_dir=None):
        assert isinstance(label, str)
        assert len(label) > 0

        self.__LABEL = label
        self._logger = create_logger(name='process_manager',
                                     output_dir=output_log_dir)
        self._log_dir = get_log_dir() \
            if output_log_dir is None else output_log_dir

        if not os.path.isdir(self._log_dir):
            os.makedirs(self._log_dir)

        self._ros_config = ROSConfig(
            ros_host=ros_host,
            ros_port=ros_port,
            gazebo_host=gazebo_host,
            gazebo_port=gazebo_port
        )

        self._tasks = dict()
        self._logger.info('Simulation manager')

        self._stages = OrderedDict()
        # Per default, roscore is always the first stage
        self._stages['roscore'] = Stage('roscore')
        # Pre-simulation tasks must come after roscore, the list can be empty
        self._stages['pre-simulation'] = Stage('pre-simulation')
        # All tasks with no specified stage will be executed last
        self._current_stage = None
        # True if run_all_tasks is called
        self._is_running = Event()
        self._is_running.clear()
        # Run all tasks thread
        self._run_tasks_thread = None

        self._recording_filenames = list()

        try:
            signal.signal(signal.SIGTERM, self._signal_handler)
            signal.signal(signal.SIGINT, self._signal_handler)
        except ValueError as ex:
            self._logger.error(
                'Failed to link signal handler, message={}'.format(ex))

    def __del__(self):
        self._logger.info('Killing process manager and all its tasks...')
        for task in self._tasks:
            if self.is_task_running(task):
                self._logger.info('Task <{}> still running'.format(task))
                self.kill_task(task)
                self._logger.info('Task <{}> killed'.format(task))
            else:
                self._logger.info('Task <{}> not running'.format(task))
        self._logger.info('Unlocking ports')
        self._logger.info(self._ros_config)
        self._ros_config.unlock_port(self._ros_config.ros_port)
        self._ros_config.unlock_port(self._ros_config.gazebo_port)

        if self._run_tasks_thread:
            self._run_tasks_thread.join(timeout=2)
            del self._run_tasks_thread

    @property
    def ros_config(self):
        return self._ros_config

    @property
    def stages(self):
        return self._stages

    @property
    def current_stage(self):
        return self._current_stage

    @property
    def recording_filenames(self):
        return self._recording_filenames

    @property
    def log_dir(self):
        return self._log_dir

    def _signal_handler(self, signal, handler):
        self._logger.warning('SIGNAL RECEIVED=%d', int(signal))
        self.__del__()

    def _task_terminated_callback(self, name):
        if name in self._tasks:
            if self._tasks[name].required:
                self._logger.warning(
                    'Required task <{}> has been terminated,'
                    ' killing all tasks'.format(name))
                for task in self._tasks:
                    if task == name:
                        continue
                    self.kill_task(task)
                    self._logger.info('Task <{}> killed'.format(task))

    def _run_all_tasks(self):
        assert len(self._stages) > 0, 'No stages found'
        assert list(self._stages.keys())[0] == 'roscore', 'First stage must' \
            ' be roscore'
        # TODO: Set the task timeout as configurable
        timeout = 30
        self._is_running.clear()
        for idx, stage in zip(range(len(self._stages)), self._stages):
            self._logger.info('Starting #{} stage={}'.format(idx, stage))
            self._logger.info(
                '  Tasks={}'.format(
                    self._stages[stage].get_tasks()))
            self._current_stage = stage

            if self._stages[stage].test_start_condition():
                self._logger.info(
                    'Stage #{} {} starting conditions fulfilled'.format(
                        idx, stage))
            else:
                self._logger.info(
                    'Stage #{} {} starting conditions failed'.format(
                        idx, stage))
                self.kill_all_tasks()
                self._is_running.clear()
                break

            self._logger.info(
                'Stage #{} {} - running pre-stage '
                'functions'.format(idx, stage))
            if not self._stages[stage].run_pre_stage_fcns():
                self._logger.info(
                    'Stage #{} {} - error while running pre-stage'
                    ' functions'.format(
                        idx, stage))
                self.kill_all_tasks()
                self._is_running.clear()
                break

            if self.run_stage(stage, timeout):
                self._logger.info(
                    'Stage #{} {} started successfully'.format(
                        idx, stage))
            else:
                self.kill_all_tasks()
                self._is_running.clear()
                break

            if self._stages[stage].test_end_condition():
                self._logger.info(
                    'Stage #{} {} end conditions fulfilled'.format(
                        idx, stage))
            else:
                self._logger.info(
                    'Stage #{} {} end conditions failed'.format(
                        idx, stage))
                self.kill_all_tasks()
                self._is_running.clear()
                break

            self._logger.info('Stage #{} {}  - running post-stage '
                              'functions'.format(idx, stage))
            if not self._stages[stage].run_post_stage_fcns():
                self._logger.info(
                    'Stage #{} {}  - error while running post-stage'
                    ' functions'.format(
                        idx, stage))
                self.kill_all_tasks()
                self._is_running.clear()
                break

        self._logger.info('Stage <{}>: ALL TASKS RUNNING'.format(stage))
        self._is_running.set()
        self._logger.info('run_all_tasks: finished')

    def is_running(self):
        return self._is_running.is_set()

    def is_roscore_running(self, timeout=30):
        from . import is_roscore_running
        start_time = time()
        while time() - start_time < timeout:
            if is_roscore_running(self._ros_config.ros_master_uri):
                return True
        return is_roscore_running(self._ros_config.ros_master_uri)

    def is_gazebo_running(self, timeout=30):
        from . import is_gazebo_running
        start_time = time()
        while time() - start_time < timeout:
            if is_gazebo_running(self._ros_config.ros_master_uri):
                return True
        return is_gazebo_running(self._ros_config.ros_master_uri)

    def has_topics(self, topics, timeout=30):
        assert isinstance(topics, list), 'Topics input must be a list'
        for topic in topics:
            assert isinstance(topic, str), 'Topic name must be a string'

        start_time = time()
        has_topics = False
        while time() - start_time < timeout:
            current_topics = self.get_rostopic_list()
            has_topics = True
            if current_topics is not None:
                for topic in topics:
                    if topic not in current_topics:
                        has_topics = False
                        break
                if has_topics:
                    return True
            else:
                has_topics = False
        return has_topics

    def set_rosparam(self, params):
        from . import set_rosparam
        if not self.is_roscore_running(timeout=0):
            self._logger.error(
                'roscore is not running! Cannot publish parameter')
            return False
        assert isinstance(params, dict), 'Parameter structure must be a dict'
        set_rosparam(params, self._ros_config.ros_master_uri)
        return True

    def get_rostopic_list(self):
        from . import get_rostopic_list
        return get_rostopic_list(self._ros_config.ros_master_uri)

    def has_services(self, services, timeout=30):
        from . import get_rosservice_list
        assert timeout > 0, 'Timeout must be greater than zero'
        assert isinstance(services, list), 'Services input must be a list'
        for serv in services:
            assert isinstance(serv, str), 'Service name must be a string'

        start_time = time()
        while time() - start_time < timeout:
            current_serv = get_rosservice_list(self._ros_config.ros_master_uri)
            has_services = True
            if current_serv is not None:
                for serv in services:
                    if serv not in current_serv:
                        has_services = False
                        break
                if has_services:
                    return True
            else:
                has_services = False
        return has_services

    def has_param(self, params, timeout=30):
        from . import get_rosparam_list
        assert timeout > 0, 'Timeout must be greater than zero'
        assert isinstance(params, list), 'Parameters input must be a list'
        for param in params:
            assert isinstance(param, str), 'Parameter name must be a string'

        start_time = time()
        while time() - start_time < timeout:
            current_params = get_rosparam_list(self._ros_config.ros_master_uri)
            has_params = True
            if current_params is not None:
                for param in params:
                    if param not in current_params:
                        has_params = False
                        break
                if has_params:
                    return True
            else:
                has_params = False
        return has_params

    def rosbags_exist(self, bags=list()):
        if len(self._recording_filenames) > 0:
            for rosbag_file in self._recording_filenames:
                if not os.path.isfile(rosbag_file):
                    return False
        if len(bags) > 0:
            for rosbag_file in bags:
                if not os.path.isfile(rosbag_file):
                    return False
        return True

    def run_stage(self, stage, timeout=30):
        from . import is_roscore_running
        assert stage in self._stages, 'Invalid stage name'

        # Test if there are any tasks to run
        # It the stage is a roscore stage, a roscore task will be
        # created if not already available
        if len(self._stages[stage].get_tasks()) == 0 and stage != 'roscore':
            self._logger.info('No tasks for stage <{}>, '
                              'skipping...'.format(stage))
            return True

        if stage == 'roscore':
            self.run_roscore()
            start_time = time()
            while time() - start_time < timeout:
                if is_roscore_running(self._ros_config.ros_master_uri):
                    self._logger.info('roscore is running')
                    return True
                self._logger.info('Waiting for roscore')
                sleep(0.5)
            if not is_roscore_running(self._ros_config.ros_master_uri):
                self._logger.info('roscore is NOT running')
                return False
        else:
            for task in self._stages[stage].get_tasks():
                self._logger.info('Starting task <{}>'.format(task))
                self._tasks[task].run()
            self._logger.info('Waiting for all tasks from stage {}'
                              ' to start...'.format(stage))
            start_time = time()
            while time() - start_time < timeout:
                running = [self._tasks[name].is_running()
                           for name in self._stages[stage].get_tasks()]
                if sum(running) == len(self._stages[stage].get_tasks()):
                    break
                self._logger.info(
                    'Waiting for tasks={}'.format(
                        self._stages[stage].get_tasks()))
                sleep(0.5)
            running = [self._tasks[name].is_running()
                       for name in self._stages[stage].get_tasks()]
            if sum(running) != len(self._stages[stage].get_tasks()):
                self._logger.error('Error! Not all tasks started. Killing '
                                   'remaining tasks')
                return False
            return True

    def get_tasks_from_stage(self, stage_name):
        if stage_name not in self._stages:
            return None
        return self._stages[stage_name].get_tasks()

    def get_gazebo_proxy(self):
        return GazeboProxy(
            ros_host=self.ros_config.ros_host,
            ros_port=self.ros_config.ros_port,
            gazebo_host=self.ros_config.gazebo_host,
            gazebo_port=self.ros_config.gazebo_port,
            timeout=30)

    def create_simulation_timer_task(self, simulation_timeout):
        task = TASK_SIMULATION_TIMER.copy()
        task['params']['timeout'] = simulation_timeout
        self._logger.info('Preparing simulation timer task')
        self._logger.info('Simulation timeout={}'.format(simulation_timeout))
        return self.init_task(**task)

    def create_recording_task(self, filename, topics, stage='pre-simulation',
                              process_timeout=None):
        assert isinstance(topics, list), 'Topics must be given as a list'
        for topic in topics:
            assert isinstance(topic, str), 'Each topic must be a string'

        task = TASK_ROSBAG_RECORD_TOPICS.copy()
        task['required'] = False
        task['process_timeout'] = process_timeout
        task['params']['filename'] = filename
        task['params']['topics'] = ' '.join(['{}'] * len(topics))
        task['params']['topics'].format(*topics)

        if self.init_task(**task):
            self._recording_filenames.append(filename)
            return True
        else:
            return False

    def create_ros_core_task(self, process_timeout=None):
        if self.is_roscore_running(0):
            self._logger.warning('roscore is already running')
            return False
        task = TASK_ROS_CORE.copy()
        # roscore is required per default
        task['required'] = True
        task['process_timeout'] = process_timeout
        task['params']['port'] = self._ros_config.ros_port
        return self.init_task(**task)

    def create_gazebo_empty_world_task(self, required=False,
                                       process_timeout=None,
                                       simulation_timeout=None,
                                       gui=True, paused=False):
        assert isinstance(required, bool), 'Input flag required must be ' \
            'a boolean'
        task = TASK_GAZEBO_EMPTY_WORLD.copy()
        task['required'] = required
        task['process_timeout'] = process_timeout
        task['simulation_timeout'] = simulation_timeout
        task['params']['gui'] = gui
        task['params']['paused'] = paused
        return self.init_task(**task)

    def create_gazebo_task(self, name='gazebo',
                           world='worlds/empty.world', gui=True,
                           physics='ode', paused=False, required=False,
                           process_timeout=None, simulation_timeout=None):

        world_file = world

        # Copy the empty world template
        task_description = TASK_GAZEBO_EMPTY_WORLD.copy()
        task_description['name'] = name
        task_description['params']['world_name'] = world_file
        task_description['params']['gui'] = gui
        task_description['params']['physics'] = physics
        task_description['params']['paused'] = paused
        task_description['required'] = required
        task_description['process_timeout'] = process_timeout
        task_description['simulation_timeout'] = simulation_timeout
        return self.init_task(**task_description)

    def create_rviz_task(self, required=False, process_timeout=None):
        task = TASK_RVIZ.copy()
        task['required'] = required
        task['process_timeout'] = process_timeout
        return self.init_task(**task)

    def create_rqt_task(self, required=False, process_timeout=None):
        task = TASK_RQT.copy()
        task['required'] = required
        task['process_timeout'] = process_timeout
        return self.init_task(**task)

    def add_stage(self, name):
        if name in self._stages:
            self._logger.error('Stage <{}> already exists'.format(name))
            return False
        self._stages[name] = Stage(name)
        if name == 'last-tasks':
            # Move last-tasks always to the end of the execution list
            if sys.version_info >= (3, 2):
                self._stages.move_to_end(name)
        return True

    def init_task(
            self,
            name,
            command,
            has_gazebo,
            type=None,
            params=dict(),
            required=False,
            process_timeout=None,
            simulation_timeout=None,
            stage=None):
        self._logger.info('Initialize task, name={}, command={}'.format(
            name, command))
        if name in self._tasks:
            self._logger.error('Task {} already exists'.format(name))
            return False

        self._logger.info('Test if Gazebo is already running...')
        if has_gazebo and self.is_gazebo_running(0):
            self._logger.error('An instance of Gazebo is already running for'
                               ' network configuration=')
            self._logger.error(self._ros_config)
            return False

        if stage == 'roscore':
            self._logger.info('roscore stage')
            assert 'roscore' in command, 'For the roscore stage only the ' \
                'roscore command is allowed'
            if len(self._stages['roscore'].get_tasks()) == 0:
                self._stages['roscore'].add_task(name)
            else:
                self._logger.error('There is already a roscore task for the'
                                   ' roscore stage')
                return False
        elif stage in self._stages:
            self._logger.info('Stage <{}> already exists'.format(stage))
            if name in self._stages[stage].get_tasks():
                self._logger.error('Task <{}> already exists in stage '
                                   '<{}>'.format(name, stage))
                return False
            self._stages[stage].add_task(name)
        elif stage is None:
            # No stage provided
            stage = 'last-tasks'
            if stage not in self._stages:
                if not self.add_stage(stage):
                    return False
            if name in self._stages[stage].get_tasks():
                self._logger.error('Task <{}> already exists in stage '
                                   '<{}>'.format(name, stage))
                return False

            self._logger.info('Adding <{}> to stage <{}>'.format(name, stage))
            self._stages.get(stage).add_task(name)
        else:
            # Adding a new stage
            self._logger.info('Creating <{}> stage'.format(stage))
            if not self.add_stage(stage):
                return False
            if name in self._stages[stage].get_tasks():
                self._logger.error('Task <{}> already exists in stage '
                                   '<{}>'.format(name, stage))
                return False
            self._logger.info('Adding <{}> to stage <{}>'.format(stage, name))

            self._stages[stage].add_task(name)

        self._logger.info('<{}> task added to <{}> stage'.format(name, stage))

        # Setting a minimal process timeout in case none was provided
        if simulation_timeout is not None and process_timeout is None:
            process_timeout = 100 * simulation_timeout
        elif process_timeout is None:
            process_timeout = 1e4

        self._tasks[name] = Task(
            task_name=name,
            command=command,
            params=params,
            config=self._ros_config,
            has_gazebo=True,
            type=type,
            required=required,
            process_timeout=process_timeout,
            task_killed_callback=self._task_terminated_callback,
            stage=stage,
            output_log_dir=self._log_dir,
            simulation_timeout=simulation_timeout)

        if simulation_timeout is not None:
            if simulation_timeout > 0:
                if self.create_simulation_timer_task(simulation_timeout):
                    self._logger.info('Simulation timer created')
                else:
                    self._logger.warning('Could not create simulation timer')

        self._logger.info('Task <{}> created'.format(name))
        return True

    def run_roscore(self):
        if 'roscore' not in self._tasks:
            self._logger.info('Adding a roscore task')
            self.create_ros_core_task()
            self._logger.info('roscore task added')

        if 'roscore' in self._tasks:
            if not self._tasks['roscore'].is_running():
                self._tasks['roscore'].run()

        timeout = 30
        start_time = time()
        while time() - start_time < timeout:
            if self.is_roscore_running():
                break
        if not self.is_roscore_running():
            raise RuntimeError('roscore did not start')

    def run_task(self, task_name):
        self._is_running.set()
        self.run_roscore()

        if task_name in self._tasks:
            if task_name != 'roscore':
                self._tasks[task_name].run()
                if 'simulation_timer' in self._tasks and \
                        self._tasks[task_name].has_gazebo:
                    self._tasks['simulation_timer'].run()
        self._is_running.clear()

    def run_all_tasks(self):
        if self._run_tasks_thread is not None:
            if self._run_tasks_thread.is_alive():
                self._logger.error('Run all tasks thread is still running')
                return False
            else:
                del self._run_tasks_thread
        self._run_tasks_thread = Thread(target=self._run_all_tasks)
        self._run_tasks_thread.daemon = True
        self._run_tasks_thread.start()

    def kill_task(self, task_name):
        if task_name in self._tasks:
            self._tasks[task_name].kill()
            self._logger.info('Task <{}> killed'.format(task_name))

    def kill_all_tasks(self):
        for task_name in self._tasks:
            self._tasks[task_name].kill()

    def remove_task(self, task_name):
        if task_name in self._tasks:
            self._tasks[task_name].kill()
            del self._tasks[task_name]
        else:
            self._logger.info('No task with name {} found'.format(task_name))

    def get_task_list(self):
        return list(self._tasks.keys())

    def is_task_running(self, task_name):
        if task_name in self._tasks:
            return self._tasks[task_name].is_running()
        else:
            self._logger.warning('Task <{}> does not exist'.format(task_name))
            return False

    def clear_tasks_list(self):
        for name in self._tasks:
            if self._tasks[name].is_running():
                self._tasks[name].kill()

        self._tasks = dict()

    def add_stage_start_condition(self, stage, fcn):
        assert stage in self._stages, 'Invalid stage name'
        assert callable(fcn), 'Invalid function'
        self._logger.info('Adding start condition to stage {}, fcn={}'.format(
            stage, fcn))
        self._stages[stage].add_start_condition(fcn)

    def get_num_start_conditions(self, stage):
        assert stage in self._stages, 'Invalid stage name'
        return self._stages[stage].get_num_start_conditions()

    def add_stage_end_condition(self, stage, fcn):
        assert stage in self._stages, 'Invalid stage name'
        assert callable(fcn), 'Invalid function'
        self._logger.info('Adding end condition to stage {}, fcn={}'.format(
            stage, fcn))
        self._stages[stage].add_end_condition(fcn)

    def get_num_end_conditions(self, stage):
        assert stage in self._stages, 'Invalid stage name'
        return self._stages[stage].get_num_end_conditions()

    def add_pre_stage_fcn(self, stage, fcn):
        assert stage in self._stages, 'Invalid stage name'
        assert callable(fcn), 'Invalid function'
        self._logger.info('Adding pre-stage function to stage {}, '
                          'fcn={}'.format(stage, fcn))
        self._stages[stage].add_pre_stage_fcn(fcn)

    def get_num_pre_stage_fcns(self, stage):
        assert stage in self._stages, 'Invalid stage name'
        return self._stages[stage].get_num_pre_stage_fcns()

    def add_post_stage_fcn(self, stage, fcn):
        assert stage in self._stages, 'Invalid stage name'
        assert callable(fcn), 'Invalid function'
        self._logger.info('Adding post-stage function to stage {}, '
                          'fcn={}'.format(stage, fcn))
        self._stages[stage].add_post_stage_fcn(fcn)

    def get_num_post_stage_fcns(self, stage):
        assert stage in self._stages, 'Invalid stage name'
        return self._stages[stage].get_num_post_stage_fcns()

    def has_required_tasks(self):
        has_required_tasks = False
        for task in self._tasks:
            has_required_tasks = has_required_tasks \
                or self._tasks[task].required
        return has_required_tasks

    def have_all_tasks_ended(self):
        return sum([not self._tasks[name].is_running()
                    for name in self._tasks]) == len(self._tasks)

    def wait(self, task_name=None, timeout=30):
        self._logger.info('Wait for tasks for finish')
        if task_name is None:
            if self._run_tasks_thread is None:
                self._logger.info('Run all tasks thread was created')
                return

            # Wait until is_running is set
            self._is_running.wait(timeout)
            self._run_tasks_thread.join()
            del self._run_tasks_thread
            self._run_tasks_thread = None

            self._logger.info('Waiting for all tasks to finish')

            # First wait for all required tasks to finish
            if self.has_required_tasks():
                required_tasks_running = True
                while required_tasks_running:
                    for name in self._tasks:
                        if self._tasks[name].required:
                            if self._tasks[name].wait(0.001):
                                self._logger.info(
                                    'Required task <{}> was '
                                    'terminated or has not '
                                    'started!'.format(name))
                                required_tasks_running = False
                                break
                            # else:
                                # sleep(1)
            else:
                while not self.have_all_tasks_ended():
                    sleep(0.3)

            # Kill all tasks
            self.kill_all_tasks()
            self._logger.info('All tasks finished')
            self._ros_config.unlock_port(self._ros_config.ros_port)
            self._ros_config.unlock_port(self._ros_config.gazebo_port)
        elif task_name in self._tasks:
            # Wait until is_running is set
            self._is_running.wait(timeout)
            self._tasks[task_name].wait()
            self._logger.info('Task <{}> finished'.format(task_name))
            self._ros_config.unlock_port(self._ros_config.ros_port)
            self._ros_config.unlock_port(self._ros_config.gazebo_port)
        else:
            self._logger.error(
                'Task with name <{}> does not exist'.format(task_name))
