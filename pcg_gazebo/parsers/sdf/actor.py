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
from .script import Script
from .skin import Skin
from .animation import Animation
from .pose import Pose
from .link import Link
from .joint import Joint
from .plugin import Plugin


class Actor(XMLBase):
    _NAME = 'actor'
    _TYPE = 'sdf'

    _ATTRIBUTES = dict(
        name='actor'
    )

    _CHILDREN_CREATORS = dict(
        script=dict(creator=Script, default=['actor']),
        skin=dict(creator=Skin, optional=True),
        animation=dict(creator=Animation, optional=True, n_elems='+'),
        pose=dict(creator=Pose, optional=True),
        link=dict(creator=Link, n_elems='+', optional=True),
        joint=dict(creator=Joint, n_elems='+', optional=True),
        plugin=dict(creator=Plugin, n_elems='+', optional=True)
    )

    def __init__(self):
        super(Actor, self).__init__()
        self.reset()

    @property
    def name(self):
        return self.attributes['name']

    @name.setter
    def name(self, value):
        assert self._is_string(value), 'Name must be a string'
        assert len(value) > 0, 'Name cannot be empty'
        self.attributes['name'] = value

    @property
    def pose(self):
        return self._get_child_element('pose')

    @pose.setter
    def pose(self, value):
        self._add_child_element('pose', value)

    @property
    def script(self):
        return self._get_child_element('script')

    @script.setter
    def script(self, value):
        self._add_child_element('script', value)

    @property
    def skin(self):
        return self._get_child_element('skin')

    @skin.setter
    def skin(self, value):
        self._add_child_element('skin', value)

    @property
    def animations(self):
        return self._get_child_element('animation')

    @property
    def links(self):
        return self._get_child_element('link')

    @property
    def joints(self):
        return self._get_child_element('joint')

    @property
    def plugins(self):
        return self._get_child_element('plugin')

    def add_link(self, name, link=None):
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

    def add_animation(self, name, animation=None):
        if self.animations is not None:
            for elem in self.joints:
                if elem.name == name:
                    print(
                        'Animation element with name {}'
                        ' already exists'.format(name))
                    return
        if animation is not None:
            self._add_child_element('animation', animation)
        else:
            animation = Animation()
            self._add_child_element('animation', animation)
        self.children['animation'][-1].name = name

    def get_animation_by_name(self, name):
        if self.animations is None:
            return None
        else:
            for elem in self.animations:
                if elem.name == name:
                    return elem
        return None
