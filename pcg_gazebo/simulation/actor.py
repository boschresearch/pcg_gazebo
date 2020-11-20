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
from .entity import Entity
from .properties import Mesh, Animation, Script, Pose
from ..parsers.sdf import create_sdf_element, is_sdf_element
from ..utils import is_string, is_array
from ..path import Path
from ..log import PCG_ROOT_LOGGER


class Actor(Entity):
    def __init__(
            self,
            name='actor',
            pose=[0, 0, 0, 0, 0, 0],
            skin=None,
            animations=None,
            script=None):
        super(Actor, self).__init__(name=name, pose=pose)

        self._skin = None
        self._script = None
        self._animations = list()
        # If true, this model can be found in the ROS_PATH or in
        # $HOME/.gazebo/models and can have its SDF file loaded from there
        self._is_gazebo_model = False
        # Name of the source model, in case the model's name and its
        # source do not match
        self._source_model_name = None
        if animations is not None:
            assert is_array(animations), 'Animations input must be a list'
            for item in animations:
                if isinstance(item, Animation) or is_sdf_element(item):
                    self.add_animation(animation=item)
                elif isinstance(item, dict):
                    self.add_animation(**item)

        if skin is not None:
            if is_string(skin):
                self.set_skin(filename=skin)
            elif isinstance(skin, dict):
                self.set_skin(**skin)

        if script is not None:
            if isinstance(script, dict):
                self.set_script(**script)
            else:
                self.set_script(script=script)

    @property
    def skin(self):
        return self._skin

    @property
    def script(self):
        return self._script

    @property
    def animations(self):
        return self._animations

    def copy(self):
        actor = Actor.from_sdf(self.to_sdf())
        actor._is_gazebo_model = self._is_gazebo_model
        return actor

    @property
    def is_gazebo_model(self):
        return self._is_gazebo_model

    @is_gazebo_model.setter
    def is_gazebo_model(self, flag):
        assert isinstance(flag, bool), 'Flag must be a boolean'
        self._is_gazebo_model = flag

    @property
    def source_model_name(self):
        if self._source_model_name is None:
            return self.name
        else:
            return self._source_model_name

    def set_script(self, **kwargs):
        if 'script' in kwargs:
            if isinstance(kwargs['script'], Script):
                self._script = kwargs['script']
            elif is_sdf_element(kwargs['script']):
                self._script = Script.from_sdf(kwargs['script'])
            else:
                raise ValueError('Invalid script input={}'.format(kwargs))
        else:
            self._script = Script(**kwargs)

    def add_animation(self, name=None, **kwargs):
        if name is None and 'animation' in kwargs:
            assert isinstance(kwargs['animation'], Animation) or \
                is_sdf_element(kwargs['animation'])
            name = kwargs['animation'].name
        has_animation = False
        for item in self._animations:
            if item.name == name:
                has_animation = True
                break
        if has_animation:
            PCG_ROOT_LOGGER.error(
                'Animation with name {} already exists'.format(name))
            return False
        if 'animation' in kwargs and \
                isinstance(kwargs['animation'], Animation):
            self._animations.append(kwargs['animation'])
        elif 'animation' in kwargs and \
                is_sdf_element(kwargs['animation']):
            assert kwargs['animation'].xml_element_name == 'animation', \
                'Wrong input SDF element, provided={}'.format(
                    kwargs['animation'].xml_element_name)
            self._animations.append(
                Animation.from_sdf(kwargs['animation']))
        else:
            self._animations.append(Animation(name=name, **kwargs))
        self._animations[-1].name = name
        return True

    def set_skin(self, filename, scale=1):
        assert is_string(filename), 'Filename must be a string'
        assert len(filename) > 0, 'Filename cannot be empty'
        if filename.endswith('.bvh'):
            self._mesh = dict(
                filename=Path(filename), scale=1)
        elif filename.endswith('.dae'):
            self._mesh = Mesh(
                filename=filename, scale=[scale for _ in range(3)])
        else:
            raise ValueError('Invalid animation filename={}'.format(filename))

    def to_sdf(self):
        sdf = create_sdf_element('actor')
        sdf.name = self._name
        if self._script is not None:
            sdf.script = self._script.to_sdf()
        sdf.pose = self._pose.to_sdf()
        for item in self._animations:
            sdf.add_animation(animation=item.to_sdf())
        if self._skin is not None:
            sdf.skin = create_sdf_element('skin')
            if isinstance(self._skin, Mesh):
                if self._skin._uri.model_uri is not None:
                    sdf.skin.filename = self._skin._uri.model_uri
                elif self._skin._uri.package_uri is not None:
                    sdf.skin.filename = self._skin._uri.package_uri
                elif self._skin._uri.file_uri is not None:
                    sdf.skin.filename = self._skin._uri.file_uri
                sdf.skin.scale = [self._skin.scale[0]]
            else:
                if self._skin['filename']._uri.model_uri is not None:
                    sdf.skin.filename = self._skin['filename']._uri.model_uri
                elif self._skin['filename']._uri.package_uri is not None:
                    sdf.skin.filename = self._skin['filename']._uri.package_uri
                elif self._skin['filename']._uri.file_uri is not None:
                    sdf.skin.filename = self._skin['filename']._uri.file_uri
                sdf.skin.scale = [self._skin['scale']]
        return sdf

    @staticmethod
    def from_sdf(sdf):
        name = sdf.name
        if sdf.pose is not None:
            pose = Pose.from_sdf(sdf.pose)
        else:
            pose = Pose()
        actor = Actor(
            name,
            pose,
            skin=sdf.skin,
            animations=sdf.animations,
            script=sdf.script)
        return actor

    @staticmethod
    def from_gazebo_model(name):
        from . import get_gazebo_model_sdf
        PCG_ROOT_LOGGER.info(
            'Importing a Gazebo actor, name={}'.format(name))
        # Update list of Gazebo models
        sdf = get_gazebo_model_sdf(name)

        if sdf is None:
            msg = 'Gazebo model {} not found in the ROS paths'.format(name)
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)
        if sdf.models is None:
            msg = 'No models found in Gazebo model {}'.format(name)
            PCG_ROOT_LOGGER.warning(msg)
            raise ValueError(msg)
        if len(sdf.models) != 1:
            msg = 'Imported SDF file should have one model only'
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)

        model = Actor.from_sdf(sdf.models[0])
        model.is_gazebo_model = True
        model._source_model_name = name
        model.name = name
        return model
