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

# TODO: Finish model XML block - missing frame, joint, plugin, gripper
from ..types import XMLBase
from .link import Link
from .static import Static
from .self_collide import SelfCollide
from .allow_auto_disable import AllowAutoDisable
from .include import Include
from .pose import Pose
from .joint import Joint
from .plugin import Plugin
from .urdf import URDF
from .scale import Scale


class Model(XMLBase):
    _NAME = 'model'
    _TYPE = 'sdf'

    _ATTRIBUTES = dict(
        name='model'
    )

    _ATTRIBUTES_MODES = dict(
        name=['state', 'model']
    )

    _CHILDREN_CREATORS = dict(
        scale=dict(creator=Scale, optional=True, mode='state'),
        link=dict(creator=Link, n_elems='+', optional=True, mode='model'),
        joint=dict(
            creator=Joint, n_elems='+', default=['model'], optional=True),
        include=dict(
            creator=Include, n_elems='+', optional=True, mode='model'),
        pose=dict(creator=Pose, optional=True, mode='model'),
        self_collide=dict(creator=SelfCollide, optional=True, mode='model'),
        static=dict(
            creator=Static, optional=True, mode='model'),
        allow_auto_disable=dict(
            creator=AllowAutoDisable, optional=True, mode='model'),
        plugin=dict(creator=Plugin, n_elems='+', optional=True, mode='model'),
        model=dict(
            creator=None, n_elems='+', default=['model'], optional=True),
        urdf=dict(
            creator=URDF, default=['model'], optional=True, mode='model')
    )

    _MODES = ['state', 'model']

    def __init__(self, mode='model'):
        super(Model, self).__init__()
        if mode == 'state':
            self._CHILDREN_CREATORS['joint']['default'] = ['state']
            self._CHILDREN_CREATORS['model']['default'] = ['state']
        else:
            self._CHILDREN_CREATORS['joint']['default'] = ['model']
            self._CHILDREN_CREATORS['model']['default'] = ['model']
        self.reset(mode=mode)

    @property
    def name(self):
        return self.attributes['name']

    @name.setter
    def name(self, value):
        assert self._is_string(value), 'Name must be a string'
        assert len(value) > 0, 'Name cannot be empty'
        self.attributes['name'] = value

    @property
    def scale(self):
        return self._get_child_element('scale')

    @scale.setter
    def scale(self, value):
        if self._mode != 'state':
            self._mode = 'state'
        self._add_child_element('scale', value)

    @property
    def static(self):
        return self._get_child_element('static')

    @static.setter
    def static(self, value):
        if self._mode != 'model':
            self._mode = 'model'
        self._add_child_element('static', value)

    @property
    def allow_auto_disable(self):
        return self._get_child_element('allow_auto_disable')

    @allow_auto_disable.setter
    def allow_auto_disable(self, value):
        if self._mode != 'model':
            self._mode = 'model'
        self._add_child_element('allow_auto_disable', value)

    @property
    def self_collide(self):
        return self._get_child_element('self_collide')

    @self_collide.setter
    def self_collide(self, value):
        if self._mode != 'model':
            self._mode = 'model'
        self._add_child_element('self_collide', value)

    @property
    def pose(self):
        return self._get_child_element('pose')

    @pose.setter
    def pose(self, value):
        if self._mode != 'model':
            self._mode = 'model'
        self._add_child_element('pose', value)

    @property
    def urdf(self):
        return self._get_child_element('urdf')

    @urdf.setter
    def urdf(self, value):
        if self._mode != 'model':
            self._mode = 'model'
        self._add_child_element('urdf', value)

    @property
    def links(self):
        return self._get_child_element('link')

    @property
    def joints(self):
        return self._get_child_element('joint')

    @property
    def includes(self):
        return self._get_child_element('include')

    @property
    def models(self):
        return self._get_child_element('model')

    @property
    def plugins(self):
        return self._get_child_element('plugin')

    def add_link(self, name, link=None):
        if self._mode != 'model':
            self._mode = 'model'
        if self.links is not None:
            for elem in self.links:
                if elem.name == name:
                    print(
                        'Link element with name {}'
                        ' already exists'.format(name))
                    return
        if link is not None:
            self._add_child_element('link', link)
        else:
            link = Link()
            self._add_child_element('link', link)
        self.children['link'][-1].name = name

    def get_link_by_name(self, name):
        if self.links is None:
            return None
        else:
            for elem in self.links:
                if elem.name == name:
                    return elem
        return None

    def remove_link_by_name(self, name):
        if self.links is None:
            return False
        for i in range(len(self.links)):
            if self.links[i].name == name:
                del self.links[i]
                break
        return True

    def add_joint(self, name, joint=None):
        if self.joints is not None:
            for elem in self.joints:
                if elem.name == name:
                    print(
                        'Joint element with name {}'
                        ' already exists'.format(name))
                    return
        if joint is not None:
            self._add_child_element('joint', joint)
        else:
            joint = Joint()
            self._add_child_element('joint', joint)
        self.children['joint'][-1].name = name

    def get_joint_by_name(self, name):
        if self.joints is None:
            return None
        else:
            for elem in self.joints:
                if elem.name == name:
                    return elem
        return None

    def remove_joint_by_name(self, name):
        if self.joints is None:
            return False
        for i in range(len(self.joints)):
            if self.joints[i].name == name:
                del self.joints[i]
                break
        return True

    def add_model(self, name, model=None):
        if self.models is not None:
            for elem in self.models:
                if elem.name == name:
                    print(
                        'Model element with name {}'
                        ' already exists'.format(name))
                    return
        if model is not None:
            self._add_child_element('model', model)
        else:
            model = Model()
            self._add_child_element('model', model)

        self.children['model'][-1].name = name

    def get_model_by_name(self, name):
        if self.models is None:
            return None
        else:
            for elem in self.models:
                if elem.name == name:
                    return elem
        return None

    def add_include(self, name, include=None):
        if self._mode != 'model':
            self._mode = 'model'
        if include is not None:
            self._add_child_element('include', include)
        else:
            include = Include()
            self._add_child_element('include', include)
        self.children['include'][-1].name = name

    def add_plugin(self, name, plugin=None):
        if plugin is not None:
            self._add_child_element('plugin', plugin)
        else:
            plugin = Plugin()
            self._add_child_element('plugin', plugin)

    def get_plugin_by_name(self, name):
        if self.plugins is None:
            return None
        else:
            for elem in self.plugins:
                if elem.name == name:
                    return elem
        return None

    def has_massless_links(self):
        if self.links is None:
            return False

        for link in self.links:
            if link.inertial is None:
                return True
            if link.inertial.mass.value == 0:
                return True
        return False

    def reset(self, mode=None, with_optional_elements=False):
        if mode is not None:
            if mode == 'state':
                self._CHILDREN_CREATORS['joint']['default'] = ['state']
                self._CHILDREN_CREATORS['model']['default'] = ['state']
            else:
                self._CHILDREN_CREATORS['joint']['default'] = ['model']
                self._CHILDREN_CREATORS['model']['default'] = ['model']
        XMLBase.reset(self, mode, with_optional_elements)
