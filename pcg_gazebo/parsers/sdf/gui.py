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
from ..types import XMLBase
from .camera import Camera
from .fullscreen import Fullscreen


class GUI(XMLBase):
    _NAME = 'gui'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        fullscreen=dict(creator=Fullscreen, default=[False], optional=True),
        camera=dict(creator=Camera, default=['gui'], optional=True)
    )

    def __init__(self):
        super(GUI, self).__init__()
        self.reset()

    @property
    def fullscreen(self):
        return self._get_child_element('fullscreen')

    @fullscreen.setter
    def fullscreen(self, value):
        self._add_child_element('fullscreen', value)

    @property
    def camera(self):
        return self._get_child_element('camera')

    @camera.setter
    def camera(self, value):
        self._add_child_element('camera', value)
