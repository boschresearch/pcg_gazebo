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
from .bounce import Bounce
from .friction import Friction
from .contact import Contact


class Surface(XMLBase):
    _NAME = 'surface'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        bounce=dict(creator=Bounce, optional=True),
        friction=dict(creator=Friction, default=['surface'], optional=True),
        contact=dict(creator=Contact, default=['collision'], optional=True)
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def bounce(self):
        return self._get_child_element('bounce')

    @bounce.setter
    def bounce(self, value):
        self._add_child_element('bounce', value)

    @property
    def friction(self):
        return self._get_child_element('friction')

    @friction.setter
    def friction(self, value):
        self._add_child_element('friction', value)

    @property
    def contact(self):
        return self._get_child_element('contact')

    @contact.setter
    def contact(self, value):
        self._add_child_element('contact', value)
