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
from copy import deepcopy
from ._picker import _Picker
from ... import random


class RandomPicker(_Picker):
    _LABEL = 'random'

    def __init__(self, items=None, max_num=None):
        super(RandomPicker, self).__init__(
            items=items, max_num=max_num)

    def get_selection(self):
        items = deepcopy(self.items)
        item = items[random.choice(range(len(items)))]
        if self.get_max_num_items(item) == self.get_counter(item):
            items.remove(item)
            if len(items) == 0:
                return None
            while len(items) > 0:
                item = items[random.choice(range(len(items)))]

                if self.get_max_num_items(item) == self.get_counter(item):
                    items.remove(item)
                else:
                    break

            if self.get_max_num_items(item) == self.get_counter(item):
                return None
        self.increase_counter(item)
        return item
