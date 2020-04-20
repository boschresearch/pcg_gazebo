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
from .name import Name
from .sdf import SDF
from .description import Description
from .author import Author
from .version import Version


class Model(XMLBase):
    _NAME = 'model'
    _TYPE = 'sdf_config'

    _CHILDREN_CREATORS = dict(
        name=dict(creator=Name, default=['model']),
        sdf=dict(creator=SDF, default=['1.5'], n_elems='+', optional=True),
        description=dict(creator=Description, default=['none']),
        author=dict(creator=Author, n_elems='+', optional=True),
        version=dict(creator=Version, default=['0.1.0'], optional=True)
    )

    def __init__(self):
        super(Model, self).__init__()
        self.reset()

    @property
    def name(self):
        return self._get_child_element('name')

    @name.setter
    def name(self, value):
        self._add_child_element('name', value)

    @property
    def version(self):
        return self._get_child_element('version')

    @version.setter
    def version(self, value):
        self._add_child_element('version', value)

    @property
    def description(self):
        return self._get_child_element('description')

    @description.setter
    def description(self, value):
        self._add_child_element('description', value)

    @property
    def sdfs(self):
        return self._get_child_element('sdf')

    @property
    def authors(self):
        return self._get_child_element('author')

    def add_sdf(self, sdf=None):
        if sdf is not None:
            self._add_child_element('sdf', sdf)
        else:
            sdf = SDF()
            self._add_child_element('sdf', sdf)

    def add_author(self, author=None):
        if author is not None:
            self._add_child_element('author', author)
        else:
            author = Author()
            self._add_child_element('author', author)
