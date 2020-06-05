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
from ._picker import _Picker
from ...collection_managers import AssetsManager


class SizePicker(_Picker):
    _LABEL = 'size'

    def __init__(self, items=None, max_num=None):
        super(SizePicker, self).__init__(
            items=items, max_num=max_num)

        self._assets_manager = AssetsManager.get_instance()

        self._volumes = dict()

        for name in items:
            if self._assets_manager.is_model_group(name) or \
                    self._assets_manager.is_gazebo_model(name) or \
                    self._assets_manager.is_model(name):
                model = self._assets_manager.get(name)
                self._volumes[name] = 0
                for mesh in model.get_meshes():
                    self._volumes[name] += mesh.volume
            else:
                raise ValueError(
                    'SizePicker: model {} is not a static asset'.format(name))

    def get_selection(self):
        if len(self._items) == 1:
            if self.get_max_num_items(self._items[0]) == \
                    self.get_counter(self._items[0]):
                return None
            else:
                self.increase_counter(self._items[0])
                return self._items[0]

        volumes = list(self._volumes.values())
        max_volume = np.max(volumes)

        item = None
        while item is None:
            for tag in self._volumes:
                if self._volumes[tag] == max_volume:
                    if self.get_max_num_items(tag) == \
                            self.get_counter(tag):
                        volumes.remove(max_volume)
                        if len(volumes) == 0:
                            return None
                        max_volume = np.max(volumes)
                    else:
                        item = tag
        self.increase_counter(item)
        return item
