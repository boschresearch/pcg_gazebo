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


class Inertia(XMLBase):
    _NAME = 'inertia'
    _TYPE = 'urdf'

    _ATTRIBUTES = dict(
        ixx='0',
        iyy='0',
        izz='0',
        ixy='0',
        ixz='0',
        iyz='0'
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def ixx(self):
        return float(self.attributes['ixx'])

    @ixx.setter
    def ixx(self, value):
        assert isinstance(value, float) or isinstance(value, int)
        self.attributes['ixx'] = '{}'.format(value)

    @property
    def iyy(self):
        return float(self.attributes['iyy'])

    @iyy.setter
    def iyy(self, value):
        assert isinstance(value, float) or isinstance(value, int)
        self.attributes['iyy'] = '{}'.format(value)

    @property
    def izz(self):
        return float(self.attributes['izz'])

    @izz.setter
    def izz(self, value):
        assert isinstance(value, float) or isinstance(value, int)
        self.attributes['izz'] = '{}'.format(value)

    @property
    def ixy(self):
        return float(self.attributes['ixy'])

    @ixy.setter
    def ixy(self, value):
        assert isinstance(value, float) or isinstance(value, int)
        self.attributes['ixy'] = '{}'.format(value)

    @property
    def ixz(self):
        return float(self.attributes['ixz'])

    @ixz.setter
    def ixz(self, value):
        assert isinstance(value, float) or isinstance(value, int)
        self.attributes['ixz'] = '{}'.format(value)

    @property
    def iyz(self):
        return float(self.attributes['iyz'])

    @iyz.setter
    def iyz(self, value):
        assert isinstance(value, float) or isinstance(value, int)
        self.attributes['iyz'] = '{}'.format(value)

    def to_sdf(self):
        from ..sdf import create_sdf_element

        obj = create_sdf_element('inertia')
        obj.ixx = self.ixx
        obj.iyy = self.iyy
        obj.izz = self.izz
        obj.ixy = self.ixy
        obj.ixz = self.ixz
        obj.iyz = self.iyz
        return obj
