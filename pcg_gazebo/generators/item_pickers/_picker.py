# Copyright (c) 2020 - The Procedural Generation for Gazebo authors
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
from ...utils import is_array


class _Picker(object):
    _LABEL = None

    def __init__(self, items=None, max_num=None):
        self._items = list()
        if is_array(items):
            self._items = items

        self._max_num = dict()
        if isinstance(max_num, dict):
            for tag in items:
                if tag in max_num:
                    self._max_num[tag] = max_num[tag]
                else:
                    self._max_num[tag] = None

        self._counter = dict()

        for tag in self._items:
            self._counter[tag] = 0

    @property
    def items(self):
        return self._items

    @items.setter
    def items(self, values):
        assert is_array(values)
        self._items = values

    @property
    def max_num(self):
        return self._max_num

    @property
    def counter(self):
        return self._counter

    def are_all_counters_max(self):
        for tag in self._counter:
            if self._max_num[tag] is None:
                return False
            if self._counter[tag] < self._max_num[tag]:
                return False
        return True

    def get_counter(self, tag):
        assert tag in self._counter
        return self._counter[tag]

    def increase_counter(self, tag):
        assert tag in self._counter
        self._counter[tag] += 1

    def set_max_num(self, tag, value):
        assert value > 0
        self._max_num[tag] = value
        if tag not in self._items:
            self._items.append(tag)
            self._counter[tag] = 0

    def get_max_num_items(self, tag):
        if tag in self._max_num:
            return self._max_num[tag]
        else:
            return -1

    def reset(self):
        for tag in self._counter:
            self._counter[tag] = 0

    def get_selection(self):
        raise NotImplementedError()
