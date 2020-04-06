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
import numpy as np
import collections
from ... import random
from ._picker import _Picker


class RouletteWheelPicker(_Picker):
    _LABEL = 'roulette'

    def __init__(self, items, max_num, fitness):
        super(RouletteWheelPicker, self).__init__(
            items=items, max_num=max_num)

        assert isinstance(fitness, dict)

        for tag in items:
            assert tag in fitness
            assert fitness[tag] > 0

        self._fitness = fitness

    @property
    def fitness(self):
        return self._fitness

    def add_item(self, tag, fitness, max_num=None):
        assert fitness > 0, \
            'Fitness for item {} must be' \
            ' greater than zero'.format(fitness)
        if max_num is not None:
            assert max_num > 0, \
                'Max. num for {} must be greater' \
                ' than zero'.format(max_num)
        self._fitness[tag] = fitness
        self._counter[tag] = 0
        self._max_num[tag] = max_num

    def get_selection(self):
        if self.are_all_counters_max():
            return None
        sorted_fitness = collections.OrderedDict(
            sorted(self._fitness.items(), key=lambda kv: kv[1]))

        total_fitness = np.sum(list(sorted_fitness.values()))
        previous = 0.0
        prob = collections.OrderedDict()
        for tag in sorted_fitness:
            prob[tag] = previous + sorted_fitness[tag] / total_fitness
            previous = prob[tag]
        selected = None
        while selected is None:
            sample = random.rand()
            for tag in prob:
                if sample < prob[tag]:
                    if self.get_max_num_items(tag) == self.get_counter(tag):
                        break
                    else:
                        selected = tag
                        break
            if selected is not None:
                break
        if selected is not None:
            self.increase_counter(selected)
        return selected
