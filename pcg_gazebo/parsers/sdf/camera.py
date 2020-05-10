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
from .noise import Noise
from .horizontal_fov import HorizontalFOV
from .image import Image
from .clip import Clip
from .save import Save
from .depth_camera import DepthCamera
from .distortion import Distortion
from .pose import Pose
from .name import Name
from .view_controller import ViewController
from .projection_type import ProjectionType
from .frame import Frame
from .plugin import Plugin
from .track_visual import TrackVisual


class Camera(XMLBase):
    _NAME = 'camera'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        noise=dict(
            creator=Noise, default=['gaussian', 'child'], optional=True,
            mode='sensor'),
        horizontal_fov=dict(creator=HorizontalFOV, mode='sensor'),
        image=dict(creator=Image, default=['camera'], mode='sensor'),
        clip=dict(creator=Clip, mode='sensor'),
        save=dict(creator=Save, optional=True, mode='sensor'),
        depth_camera=dict(creator=DepthCamera, optional=True, mode='sensor'),
        distortion=dict(creator=Distortion, optional=True, mode='sensor'),
        pose=dict(creator=Pose, optional=True, mode='gui'),
        name=dict(creator=Name, optional=True, mode='gui'),
        view_controller=dict(
            creator=ViewController, optional=True, mode='gui'),
        projection_type=dict(
            creator=ProjectionType, optional=True, mode='gui'),
        frame=dict(creator=Frame, optional=True, mode='gui'),
        plugin=dict(creator=Plugin, optional=True, mode='gui'),
        track_visual=dict(creator=TrackVisual, optional=True, mode='gui')
    )

    _ATTRIBUTES = dict(
        name='default'
    )

    _MODES = ['sensor', 'gui']

    def __init__(self, mode='sensor'):
        super(Camera, self).__init__()
        self.reset(mode)

    @property
    def name(self):
        return self.attributes['name']

    @name.setter
    def name(self, value):
        assert isinstance(value, str), 'Name should be a string'
        assert len(value) > 0, 'Name should not be an empty string'
        self.attributes['name'] = value

    @property
    def noise(self):
        return self._get_child_element('noise')

    @noise.setter
    def noise(self, value):
        if self._mode != 'sensor':
            self._mode = 'sensor'
        self._add_child_element('noise', value)

    @property
    def horizontal_fov(self):
        return self._get_child_element('horizontal_fov')

    @horizontal_fov.setter
    def horizontal_fov(self, value):
        if self._mode != 'sensor':
            self._mode = 'sensor'
        self._add_child_element('horizontal_fov', value)

    @property
    def image(self):
        return self._get_child_element('image')

    @image.setter
    def image(self, value):
        if self._mode != 'sensor':
            self._mode = 'sensor'
        self._add_child_element('image', value)

    @property
    def clip(self):
        return self._get_child_element('clip')

    @clip.setter
    def clip(self, value):
        if self._mode != 'sensor':
            self._mode = 'sensor'
        self._add_child_element('clip', value)

    @property
    def save(self):
        return self._get_child_element('save')

    @save.setter
    def save(self, value):
        if self._mode != 'sensor':
            self._mode = 'sensor'
        self._add_child_element('save', value)

    @property
    def depth_camera(self):
        return self._get_child_element('depth_camera')

    @depth_camera.setter
    def depth_camera(self, value):
        if self._mode != 'sensor':
            self._mode = 'sensor'
        self._add_child_element('depth_camera', value)

    @property
    def distortion(self):
        return self._get_child_element('distortion')

    @distortion.setter
    def distortion(self, value):
        if self._mode != 'sensor':
            self._mode = 'sensor'
        self._add_child_element('distortion', value)

    @property
    def pose(self):
        return self._get_child_element('pose')

    @pose.setter
    def pose(self, value):
        if self._mode != 'gui':
            self._mode = 'gui'
        self._add_child_element('pose', value)

    @property
    def view_controller(self):
        return self._get_child_element('view_controller')

    @view_controller.setter
    def view_controller(self, value):
        if self._mode != 'gui':
            self._mode = 'gui'
        self._add_child_element('view_controller', value)

    @property
    def projection_type(self):
        return self._get_child_element('projection_type')

    @projection_type.setter
    def projection_type(self, value):
        if self._mode != 'gui':
            self._mode = 'gui'
        self._add_child_element('projection_type', value)

    @property
    def track_visual(self):
        return self._get_child_element('track_visual')

    @track_visual.setter
    def track_visual(self, value):
        self._add_child_element('track_visual', value)

    @property
    def frames(self):
        return self._get_child_element('frame')

    @property
    def plugins(self):
        return self._get_child_element('plugin')

    def add_frame(self, name=None, frame=None):
        if self._mode != 'gui':
            self._mode = 'gui'
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

    def add_plugin(self, name=None, plugin=None):
        if self._mode != 'gui':
            self._mode = 'gui'
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
        self._children['plugin'][-1].name = name
