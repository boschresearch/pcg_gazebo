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
from ...utils import is_string
from .gravity import Gravity
from .kinematic import Kinematic
from .inertial import Inertial
from .pose import Pose
from .collision import Collision
from .visual import Visual
from .plugin import Plugin
from .sensor import Sensor
from .self_collide import SelfCollide
from .enable_wind import EnableWind
from .light import Light
from .velocity import Velocity
from .acceleration import Acceleration
from .wrench import Wrench
from .frame import Frame


class Link(XMLBase):
    _NAME = 'link'
    _TYPE = 'sdf'

    _ATTRIBUTES = dict(
        name='link'
    )

    _CHILDREN_CREATORS = dict(
        gravity=dict(
            creator=Gravity,
            default=[True],
            n_elems=1,
            optional=True,
            mode='link'),
        light=dict(
            creator=Light,
            n_elems='+',
            optional=True,
            mode='link'),
        kinematic=dict(
            creator=Kinematic,
            n_elems=1,
            optional=True,
            mode='link'),
        inertial=dict(
            creator=Inertial,
            n_elems=1,
            optional=True,
            mode='link'),
        pose=dict(
            creator=Pose,
            n_elems=1,
            optional=True),
        collision=dict(
            creator=Collision,
            default=['link'],
            n_elems='+',
            optional=True),
        visual=dict(
            creator=Visual,
            n_elems='+',
            optional=True,
            mode='link'),
        sensor=dict(
            creator=Sensor,
            n_elems='+',
            optional=True,
            mode='link'),
        plugin=dict(
            creator=Plugin,
            n_elems='+',
            optional=True,
            mode='link'),
        self_collide=dict(
            creator=SelfCollide,
            default=[False],
            optional=True,
            mode='link'),
        enable_wind=dict(
            creator=EnableWind,
            default=[False],
            optional=True,
            mode='link'),
        velocity=dict(
            creator=Velocity,
            default=[[0, 0, 0, 0, 0, 0]],
            optional=True,
            mode='state'),
        acceleration=dict(
            creator=Acceleration,
            default=[[0, 0, 0, 0, 0, 0]],
            optional=True,
            mode='state'),
        wrench=dict(
            creator=Wrench,
            default=[[0, 0, 0, 0, 0, 0]],
            optional=True,
            mode='state'),
        frame=dict(
            creator=Frame,
            n_elems='+',
            optional=True,
            mode='state')
    )

    _MODES = ['link', 'state']

    def __init__(self, mode='link'):
        super(Link, self).__init__()
        if mode == 'link':
            self._CHILDREN_CREATORS['collision']['default'] = ['link']
        else:
            self._CHILDREN_CREATORS['collision']['default'] = ['state']
        self.reset()

    @property
    def name(self):
        return self.attributes['name']

    @name.setter
    def name(self, value):
        assert is_string(value), \
            'Name should be a string, received={},' \
            ' type={}'.format(value, type(value))
        assert len(value) > 0, \
            'Name string should not be empty'
        self.attributes['name'] = value

    @property
    def pose(self):
        return self._get_child_element('pose')

    @pose.setter
    def pose(self, vec):
        self._add_child_element('pose', vec)

    @property
    def gravity(self):
        return self._get_child_element('gravity')

    @gravity.setter
    def gravity(self, value):
        if self._mode != 'link':
            self._mode = 'link'
        self._add_child_element('gravity', value)

    @property
    def self_collide(self):
        return self._get_child_element('self_collide')

    @self_collide.setter
    def self_collide(self, value):
        if self._mode != 'link':
            self._mode = 'link'
        self._add_child_element('self_collide', value)

    @property
    def kinematic(self):
        return self._get_child_element('kinematic')

    @kinematic.setter
    def kinematic(self, value):
        if self._mode != 'link':
            self._mode = 'link'
        self._add_child_element('kinematic', value)

    @property
    def enable_wind(self):
        return self._get_child_element('enable_wind')

    @enable_wind.setter
    def enable_wind(self, value):
        if self._mode != 'link':
            self._mode = 'link'
        self._add_child_element('enable_wind', value)

    @property
    def mass(self):
        if self._mode != 'link':
            self._mode = 'link'
        if 'inertial' in self.children:
            return self.children['inertial'].mass
        else:
            return None

    @mass.setter
    def mass(self, value):
        if 'inertial' not in self.children:
            self.children['inertial'] = Inertial()
        self.children['inertial'].mass = value

    @property
    def center_of_mass(self):
        if self._mode != 'link':
            self._mode = 'link'
        if 'inertial' in self.children:
            return self.children['inertial'].pose.pose[0:3]
        else:
            return None

    @center_of_mass.setter
    def center_of_mass(self, vec):
        if self._mode != 'link':
            self._mode = 'link'
        if 'inertial' not in self.children:
            self.children['inertial'] = Inertial()
        assert self._is_numeric_vector(vec)
        vec = list(vec)
        assert len(vec) == 3

        self.children['inertial'].pose.x = vec[0]
        self.children['inertial'].pose.y = vec[1]
        self.children['inertial'].pose.z = vec[2]

    @property
    def inertia(self):
        if self._mode != 'link':
            self._mode = 'link'
        if 'inertial' in self.children:
            return self.children['inertial'].inertia
        else:
            return None

    @inertia.setter
    def inertia(self, values):
        if 'inertial' not in self.children:
            self.children['inertial'] = Inertial()
        self.children['inertial'].inertia = values

    @property
    def inertial(self):
        return self._get_child_element('inertial')

    @inertial.setter
    def inertial(self, value):
        if self._mode != 'link':
            self._mode = 'link'
        self._add_child_element('inertial', value)

    @property
    def velocity(self):
        return self._get_child_element('velocity')

    @velocity.setter
    def velocity(self, value):
        if self._mode != 'state':
            self._mode = 'state'
        self._add_child_element('velocity', value)

    @property
    def acceleration(self):
        return self._get_child_element('acceleration')

    @acceleration.setter
    def acceleration(self, value):
        if self._mode != 'state':
            self._mode = 'state'
        self._add_child_element('acceleration', value)

    @property
    def wrench(self):
        return self._get_child_element('wrench')

    @wrench.setter
    def wrench(self, value):
        if self._mode != 'state':
            self._mode = 'state'
        self._add_child_element('wrench', value)

    @property
    def frames(self):
        return self._get_child_element('frame')

    @property
    def collisions(self):
        return self._get_child_element('collision')

    @property
    def visuals(self):
        return self._get_child_element('visual')

    @property
    def sensors(self):
        return self._get_child_element('sensor')

    @property
    def plugins(self):
        return self._get_child_element('plugins')

    @property
    def lights(self):
        return self._get_child_element('light')

    def add_collision(self, name, collision=None):
        if self.collisions is not None:
            for elem in self.collisions:
                if elem.name == name:
                    i = 0
                    new_name = '{}_{}'.format(name, i)
                    while self.get_collision_by_name(new_name) is not None:
                        i += 1
                        new_name = '{}_{}'.format(name, i)
                    name = new_name
                    break
        if collision is not None:
            self._add_child_element('collision', collision)
        else:
            collision = Collision()
            self._add_child_element('collision', collision)
        self.children['collision'][-1].name = name

    def get_collision_by_name(self, name):
        if self.collisions is None:
            return None
        else:
            for elem in self.collisions:
                if elem.name == name:
                    return elem
        return None

    def add_visual(self, name, visual=None):
        if self._mode != 'link':
            self._mode = 'link'
        if self.visuals is not None:
            for elem in self.visuals:
                if elem.name == name:
                    i = 0
                    new_name = '{}_{}'.format(name, i)
                    while self.get_visual_by_name(new_name) is not None:
                        i += 1
                        new_name = '{}_{}'.format(name, i)
                    name = new_name
        if visual is not None:
            self._add_child_element('visual', visual)
        else:
            visual = Visual()
            self._add_child_element('visual', visual)
        self.children['visual'][-1].name = name

    def get_visual_by_name(self, name):
        if self.visuals is None:
            return None
        else:
            for elem in self.visuals:
                if elem.name == name:
                    return elem
        return None

    def add_sensor(self, name, sensor=None):
        if self._mode != 'link':
            self._mode = 'link'
        if self.sensors is not None:
            for elem in self.sensors:
                if elem.name == name:
                    print(
                        'Sensor element with name {}'
                        ' already exists'.format(name))
                    return
        if sensor is not None:
            self._add_child_element('sensor', sensor)
        else:
            sensor = Sensor()
            self._add_child_element('sensor', sensor)
        self.children['sensor'][-1].name = name

    def get_sensor_by_name(self, name):
        if self.sensors is None:
            return None
        else:
            for elem in self.sensors:
                if elem.name == name:
                    return elem
        return None

    def add_plugin(self, name, plugin=None):
        if self._mode != 'link':
            self._mode = 'link'
        if self.plugins is not None:
            for elem in self.plugins:
                if elem.name == name:
                    print(
                        'Plugin element with name {}'
                        ' already exists'.format(name))
                    return
        if plugin is not None:
            self._add_child_element('plugin', plugin)
        else:
            plugin = Plugin()
            self._add_child_element('plugin', plugin)
        self.children['plugin'][-1].name = name

    def get_plugin_by_name(self, name):
        if self.plugins is None:
            return None
        else:
            for elem in self.plugins:
                if elem.name == name:
                    return elem
        return None

    def add_light(self, name, light=None):
        if self._mode != 'link':
            self._mode = 'link'
        if self.lights is not None:
            for elem in self.lights:
                if elem.name == name:
                    print(
                        'Light element with name {}'
                        ' already exists'.format(name))
                    return
        if light is not None:
            self._add_child_element('light', light)
        else:
            light = Light()
            self._add_child_element('light', light)
        self.children['light'][-1].name = name

    def get_light_by_name(self, name):
        if self.lights is None:
            return None
        else:
            for elem in self.lights:
                if elem.name == name:
                    return elem
        return None

    def add_frame(self, name=None, frame=None):
        if self._mode != 'state':
            self._mode = 'state'
        if self.frames is not None:
            for elem in self.frames:
                if elem.name == name:
                    print(
                        'Frame element with name {}'
                        ' already exists'.format(name))
                    return
        if frame is not None:
            self._add_child_element('frame', frame)
        else:
            frame = Frame()
            self._add_child_element('frame', frame)
        self._children['frame'][-1].name = name

    def is_valid(self):
        if len(self.attributes) != 1:
            print('Link should have at least one attribute')
            return False
        if 'name' not in self.attributes:
            print('Link should have an attribute <name>')
            return False
        if len(self.attributes['name']) == 0:
            print('Link name attribute is empty')
            return False
        return XMLBase.is_valid(self)
