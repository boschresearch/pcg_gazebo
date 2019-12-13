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


class Stage:
    def __init__(self, name):

        assert isinstance(name, str), 'Stage name must be a string'
        assert len(name) > 0, 'Stage name cannot be empty'

        self._name = name
        self._tasks = list()
        self._start_condition_fcns = list()
        self._end_condition_fcns = list()
        self._pre_stage_fcns = list()
        self._post_stage_fcns = list()

    def __str__(self):
        msg = 'Stage {}\n'.format(self._name)
        if len(self._tasks) > 0:
            msg += ' Tasks:\n'
            for task in self._tasks:
                msg += '   - {}\n'.format(task)
        else:
            msg += ' No tasks\n'
        return msg

    def set_name(self, name):
        self._name = name

    def get_name(self):
        return self._name

    def get_tasks(self):
        return self._tasks

    def test_start_condition(self):
        output = True
        for fcn in self._start_condition_fcns:
            output = output and fcn()
        return output

    def test_end_condition(self):
        output = True
        for fcn in self._end_condition_fcns:
            output = output and fcn()
        return output

    def add_task(self, task_name):
        if task_name in self._tasks:
            print('Task <{}> already exists'.format(task_name))
            return False
        else:
            self._tasks.append(task_name)
            return True

    def run_pre_stage_fcns(self):
        try:
            for fcn in self._pre_stage_fcns:
                fcn()
            return True
        except Exception as ex:
            print('Error while running pre-stage functions for stage {},'
                  ' message={}'.format(self._name, ex))
            return False

    def run_post_stage_fcns(self):
        try:
            for fcn in self._post_stage_fcns:
                fcn()
            return True
        except Exception as ex:
            print('Error while running post-stage functions for stage {},'
                  ' message={}'.format(self._name, ex))
            return False

    def add_pre_stage_fcn(self, fcn):
        assert callable(fcn), 'Function is invalid'
        self._pre_stage_fcns.append(fcn)

    def get_num_pre_stage_fcns(self):
        return len(self._pre_stage_fcns)

    def add_post_stage_fcn(self, fcn):
        assert callable(fcn), 'Function is invalid'
        self._post_stage_fcns.append(fcn)

    def get_num_post_stage_fcns(self):
        return len(self._post_stage_fcns)

    def add_start_condition(self, fcn):
        assert callable(fcn), 'Function is invalid'
        self._start_condition_fcns.append(fcn)

    def get_num_start_conditions(self):
        return len(self._start_condition_fcns)

    def add_end_condition(self, fcn):
        assert callable(fcn), 'Function is invalid'
        self._end_condition_fcns.append(fcn)

    def get_num_end_conditions(self):
        return len(self._end_condition_fcns)
