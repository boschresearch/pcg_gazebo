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
import collections
from copy import deepcopy
from .properties import Pose
from .model import SimulationModel
from .light import Light
from .actor import Actor
from .entity import Entity
from ..log import PCG_ROOT_LOGGER
from ..utils import has_string_pattern


class ModelGroup(Entity):
    def __init__(self, name='group', pose=[0, 0, 0, 0, 0, 0],
                 is_ground_plane=False):
        super(ModelGroup, self).__init__(
            name=name, pose=pose)
        self._models = dict()
        self._actors = dict()
        self._lights = dict()
        # Flag to indicate if the model is a ground plane
        self._is_ground_plane = is_ground_plane

    def __ne__(self, other):
        result = self.__eq__(other)
        return not result

    def __eq__(self, other):
        if not isinstance(other, SimulationModel):
            return False
        return self.to_sdf('model') == other.to_sdf('model')

    @property
    def prefix(self):
        prefix = ''
        if self.name != 'default':
            prefix = '{}/'.format(self.name)
        return prefix

    @property
    def is_ground_plane(self):
        return self._is_ground_plane

    @is_ground_plane.setter
    def is_ground_plane(self, flag):
        assert isinstance(flag, bool), 'Input must be a boolean'
        self._is_ground_plane = flag

    @property
    def models(self):
        """`dict`: Models"""
        return self._models

    @property
    def lights(self):
        """`dict`: Lights"""
        return self._lights

    @property
    def actors(self):
        """`dict`: Actors"""
        return self._actors

    @property
    def n_models(self):
        """`int`: Number of models"""
        n_models = 0
        for tag in self._models:
            if isinstance(self._models[tag], SimulationModel):
                n_models += 1
            else:
                n_models += self._models[tag].n_models
        return n_models

    @property
    def n_lights(self):
        """`int`: Number of lights"""
        return len(self._lights)

    @property
    def n_actors(self):
        """`int`: Number of actors"""
        return len(self._actors)

    @property
    def has_mesh(self):
        for tag in self._models:
            if self._models[tag].has_mesh:
                return True
        return False

    def copy(self):
        mg = ModelGroup(name=deepcopy(self._name))
        for tag in self._models:
            mg._models[tag] = self._models[tag].copy()
        for tag in self._lights:
            mg._lights[tag] = self._lights[tag].copy()
        for tag in self._actors:
            mg._actors[tag] = self._actors[tag].copy()
        mg._is_ground_plane = deepcopy(self._is_ground_plane)
        mg._pose = deepcopy(self._pose)
        return mg

    def set_as_ground_plane(self, model_name):
        if model_name in self._models:
            self._models[model_name].set_as_ground_plane()
        else:
            if '/' in model_name:
                group_name = model_name.split('/')[0]
                if group_name in self._models:
                    return self._models[group_name].set_as_ground_plane(
                        model_name)
                else:
                    return False
            else:
                return False
        return True

    def reset_models(self):
        """Reset the list of models."""
        self._models = dict()

    def reset_lights(self):
        """Reset the list of lights."""
        self._lights = dict()

    def reset_actors(self):
        """Reset the list of actors."""
        self._actors = dict()

    def add_model(self, tag, model):
        """Add a model to the world.

        > *Input arguments*

        * `tag` (*type:* `str`): Model's local name in the world. If
        a model with the same name already exists, the model will be
        created with a counter suffix in the format `_i`, `i` being
        an integer.
        * `model` (*type:* `pcg_gazebo.simulation.SimulationModel`):
        Model object

        > *Returns*

        `bool`: `True`, if model could be added to the world.
        """
        assert isinstance(model, (SimulationModel, ModelGroup)), \
            'Input model is not of type' \
            ' SimulationModel nor a ModelGroup'
        if self.model_exists(tag):
            # Add counter suffix to add models with same name
            i = 0
            new_model_name = '{}'.format(tag)
            while self.model_exists(new_model_name):
                i += 1
                new_model_name = '{}_{}'.format(tag, i)
            name = new_model_name
        else:
            name = tag

        self._models[name] = model.copy()
        self._models[name].name = name
        return name

    def rm_model(self, tag):
        """Remove model from group.

        > *Input arguments*

        * `tag` (*type:* `str`): Local name identifier of the
        model to be removed.

        > *Returns*

        `bool`: `True`, if model could be removed, `False` if
        no model with name `tag` could be found in the world.
        """
        if tag in self._models:
            del self._models[tag]
            return True
        return False

    def model_exists(self, tag):
        """Test if a model with name `tag` exists in the world description.

        > *Input arguments*

        * `tag` (*type:* `str`): Local name identifier of the model.

        > *Returns*

        `bool`: `True`, if model exists, `False`, otherwise.
        """
        return tag in self._models

    def add_actor(self, tag, actor):
        """Add an actor to the group.

        > *Input arguments*

        * `tag` (*type:* `str`): Actor's local name in the world. If
        a actor with the same name already exists, the actor will be
        created with a counter suffix in the format `_i`, `i` being
        an integer.
        * `actor` (*type:* `pcg_gazebo.simulation.Actor`):
        Actor object

        > *Returns*

        `bool`: `True`, if model could be added to the world.
        """
        assert isinstance(actor, Actor), \
            'Input model is not of type' \
            ' Actor'
        if self.actor_exists(tag):
            # Add counter suffix to add actors with same name
            i = 0
            new_actor_name = '{}'.format(tag)
            while self.actor_exists(new_actor_name):
                i += 1
                new_actor_name = '{}_{}'.format(tag, i)
            name = new_actor_name
        else:
            name = tag

        self._actors[name] = actor.copy()
        self._actors[name].name = name
        return name

    def rm_actor(self, tag):
        """Remove actor from group.

        > *Input arguments*

        * `tag` (*type:* `str`): Local name identifier of the
        actor to be removed.

        > *Returns*

        `bool`: `True`, if actor could be removed, `False` if
        no actor with name `tag` could be found in the world.
        """
        if tag in self._actors:
            del self._actors[tag]
            return True
        return False

    def actor_exists(self, tag):
        """Test if a actor with name `tag` exists in the group description.

        > *Input arguments*

        * `tag` (*type:* `str`): Local name identifier of the actor.

        > *Returns*

        `bool`: `True`, if actor exists, `False`, otherwise.
        """
        return tag in self._actors

    def add_include(self, include):
        """Add a model via include method.

        > *Input arguments*

        * `include` (*type:* `pcg_gazebo.parsers.sdf.Include`):
        SDF `<include>` element

        > *Returns*

        `bool`: `True`, if model directed by the `include` element
        could be parsed and added to the world.
        """
        from . import get_gazebo_model_sdf, is_gazebo_model
        model_name = include.uri.value.replace('model://', '')
        try:
            model = SimulationModel.from_gazebo_model(model_name)
            # Set model pose, if specified
            if include.pose is not None:
                model.pose = include.pose.value
            # Set the model as static, if specified
            if include.static is not None:
                model.static = include.static.value
            if include.name is not None:
                name = include.name.value
            else:
                name = model_name
            return self.add_model(name, model)
        except ValueError:
            if model_name == 'sun' and not is_gazebo_model('sun'):
                from ..simulation.components import Sun
                self.add_light('sun', Sun())
            else:
                sdf = get_gazebo_model_sdf(model_name)
                if sdf.lights is not None:
                    PCG_ROOT_LOGGER.info(
                        'Loading model {} as light'
                        ' source model'.format(model_name))
                    for light in sdf.lights:
                        light = Light.from_sdf(light)
                        self.add_light(light.name, light)
                        PCG_ROOT_LOGGER.info(
                            'Added light {} from included model {}'.format(
                                light.name, model_name))

    def get_meshes(self, mesh_type='collision', pose_offset=None):
        if mesh_type not in ['collision', 'visual']:
            msg = 'Mesh type to compute the footprints' \
                ' must be either collision or visual' \
                ' geometries, provided={}'.format(mesh_type)
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)

        if pose_offset is not None:
            if not isinstance(pose_offset, Pose):
                msg = 'Invalid pose property object'
                PCG_ROOT_LOGGER.error(msg)
                raise ValueError(msg)
        else:
            pose_offset = Pose()

        combined_pose = pose_offset + self._pose

        meshes = list()
        for tag in self._models:
            meshes = meshes + \
                self._models[tag].get_meshes(mesh_type, combined_pose)

        return meshes

    def get_footprint(
            self,
            mesh_type='collision',
            pose_offset=None,
            use_bounding_box=False,
            z_limits=None):
        if mesh_type not in ['collision', 'visual']:
            msg = 'Mesh type to compute the footprints' \
                ' must be either collision or visual' \
                ' geometries, provided={}'.format(mesh_type)
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)

        if pose_offset is not None:
            if not isinstance(pose_offset, Pose):
                msg = 'Invalid pose property object'
                PCG_ROOT_LOGGER.error(msg)
                raise ValueError(msg)
        else:
            pose_offset = Pose()

        if z_limits is not None:
            if not isinstance(z_limits, collections.Iterable):
                msg = 'Z limits input has to be a list, provided={}'.format(
                    z_limits)
                PCG_ROOT_LOGGER.error(msg)
                raise ValueError(msg)

        combined_pose = pose_offset + self._pose

        footprints = dict()
        for tag in self._models:
            footprint = self._models[tag].get_footprint(
                mesh_type, combined_pose, use_bounding_box, z_limits)
            if footprint is not None:
                footprints[self.name + '::' +
                           self._models[tag].name] = footprint

        PCG_ROOT_LOGGER.info(
            'Footprint computed for model <{}>'.format(
                self.name))
        return footprints

    def get_bounds(self, mesh_type='collision'):
        meshes = self.get_meshes(mesh_type)

        bounds = None
        for mesh in meshes:
            if bounds is None:
                bounds = deepcopy(mesh.bounds)
            else:
                cur_bounds = deepcopy(mesh.bounds)
                for i in range(3):
                    bounds[0, i] = min(bounds[0, i], cur_bounds[0, i])
                for i in range(3):
                    bounds[1, i] = max(bounds[1, i], cur_bounds[1, i])
        return bounds

    def create_scene(self, mesh_type='collision', add_pseudo_color=True,
                     ignore_models=None, add_axis=False):
        from ..visualization import create_scene
        return create_scene(
            list(self.get_models(
                with_group_prefix=True, ignore_models=ignore_models).values()),
            mesh_type, add_pseudo_color, add_axis=add_axis)

    def get_actor(self, name, with_group_prefix=True, use_group_pose=True):
        prefix = self.prefix if with_group_prefix else ''
        if '/' not in name:
            if name not in self._actors:
                PCG_ROOT_LOGGER.warning('No actor {} found in group {}'.format(
                    name, self.name))
                return None

            output = self._actors[name].copy()
            if use_group_pose:
                output.pose = self._pose + output.pose
            output.name = prefix + output.name
        else:
            sub_group_name = name.split('/')[0]
            if sub_group_name not in self._models:
                PCG_ROOT_LOGGER.warning(
                    '<{}> is not a model group in <{}>'.format(
                        sub_group_name, self.name))
                return None
            output = self._models[sub_group_name].get_actor(
                name=name.replace(sub_group_name + '/', ''),
                with_group_prefix=True)
            if use_group_pose:
                output.pose = self._pose + output.pose
            output.name = prefix + output.name
        PCG_ROOT_LOGGER.info('Retrieving actor <{}> from group <{}>'.format(
            output.name, self.name))
        return output

    def get_actors(self, with_group_prefix=True, use_group_pose=True):
        prefix = self.prefix if with_group_prefix else ''
        output_actors = dict()
        for name in self._actors:
            light = self.get_actor(name, with_group_prefix)
            output_actors[light.name] = self.get_actor(
                name, with_group_prefix, use_group_pose)
        for tag in self._models:
            if isinstance(self._models[tag], ModelGroup):
                actors = self._models[tag].get_actors(
                    with_group_prefix=True, use_group_pose=True)
                for name in actors:
                    actors[name].name = prefix + actors[name].name
                    actors[name].pose = actors[name].pose + self._pose
                    output_actors[actors[name].name] = actors[name]
        return output_actors

    def get_model(self, name, with_group_prefix=True, use_group_pose=True):
        prefix = self.prefix if with_group_prefix else ''
        if '/' not in name:
            if name not in self._models:
                PCG_ROOT_LOGGER.warning('No model {} found in group {}'.format(
                    name, self.name))
                return None

            output = self._models[name].copy()
            output.is_ground_plane = self._models[name].is_ground_plane
            if use_group_pose:
                output.pose = self._pose + output.pose
            output.name = prefix + output.name
        else:
            sub_group_name = name.split('/')[0]
            if sub_group_name not in self._models:
                PCG_ROOT_LOGGER.warning(
                    '<{}> is not a model group in <{}>'.format(
                        sub_group_name, self.name))
                return None
            output = self._models[sub_group_name].get_model(
                name=name.replace(sub_group_name + '/', ''),
                with_group_prefix=True)
            if use_group_pose:
                output.pose = self._pose + output.pose
            output.name = prefix + output.name
        PCG_ROOT_LOGGER.info('Retrieving model <{}> from group <{}>'.format(
            output.name, self.name))
        return output

    def get_models(self, with_group_prefix=True, use_group_pose=True,
                   ignore_models=None):
        prefix = self.prefix if with_group_prefix else ''
        output_models = dict()
        if ignore_models is None:
            ignore_models = list()

        def _is_ignored(name):
            for item in ignore_models:
                if has_string_pattern(name, item):
                    return True
            return False

        for name in self._models:
            if _is_ignored(name):
                continue
            if isinstance(self._models[name], SimulationModel):
                model = self.get_model(name, with_group_prefix, use_group_pose)
                output_models[model.name] = model
            elif isinstance(self._models[name], ModelGroup):
                models = self._models[name].get_models(
                    with_group_prefix=True, use_group_pose=True)
                for name in models:
                    models[name].name = prefix + models[name].name
                    if use_group_pose:
                        models[name].pose = models[name].pose + self._pose
                    output_models[models[name].name] = models[name]
        return output_models

    def get_light(self, name, with_group_prefix=True, use_group_pose=True):
        prefix = self.prefix if with_group_prefix else ''
        light = None
        if '/' not in name:
            if name not in self._lights:
                PCG_ROOT_LOGGER.warning(
                    'No light model <{}> found in group {}'.format(
                        name, self.name))
                return None

            light = self._lights[name].copy()
            if use_group_pose:
                light.pose = light.pose + self._pose
            light.name = prefix + light.name
        else:
            sub_group_name = name.split('/')[0]
            if sub_group_name not in self._models:
                PCG_ROOT_LOGGER.warning(
                    '<{}> is not a model group in <{}>'.format(
                        sub_group_name, self.name))
                return None
            light = self._models[sub_group_name].get_light(
                name=name.replace(sub_group_name + '/', ''),
                with_group_prefix=True)
            if use_group_pose:
                light.pose = light.pose + self._pose
            light.name = prefix + light.name
        PCG_ROOT_LOGGER.info(
            'Retrieving light model <{}> from group <{}>'.format(
                light.name, self.name))
        return light

    def get_lights(self, with_group_prefix=True, use_group_pose=True):
        prefix = self.prefix if with_group_prefix else ''
        output_lights = dict()
        for name in self._lights:
            light = self.get_light(name, with_group_prefix)
            output_lights[light.name] = self.get_light(
                name, with_group_prefix, use_group_pose)
        for tag in self._models:
            if isinstance(self._models[tag], ModelGroup):
                lights = self._models[tag].get_lights(
                    with_group_prefix=True, use_group_pose=True)
                for name in lights:
                    lights[name].name = prefix + lights[name].name
                    lights[name].pose = lights[name].pose + self._pose
                    output_lights[lights[name].name] = lights[name]
        return output_lights

    def add_light(self, tag, light):
        """Add light description to the world.

        > *Input arguments*

        * `tag` (*type:* `str`): Name identifier for the plugin. If
        a model with the same name already exists, the model will be
        created with a counter suffix in the format `_i`, `i` being
        an integer.
        * `light` (*type:* `pcg_gazebo.parsers.sdf.Light` or
        `pcg_gazebo.simulation.properties.Light`): Light description
        """
        assert isinstance(light, Light), 'Invalid light object'
        if self.light_exists(tag):
            # Add counter suffix to add lights with same name
            i = 0
            new_light_name = '{}'.format(tag)
            while self.light_exists(new_light_name):
                i += 1
                new_light_name = '{}_{}'.format(tag, i)
            name = new_light_name
        else:
            name = tag

        self._lights[name] = light
        self._lights[name].name = name
        PCG_ROOT_LOGGER.info('Light added, name={}, group={}'.format(
            name, self.name))
        return True

    def rm_light(self, tag):
        """Remove light from world.

        > *Input arguments*

        * `tag` (*type:* `str`): Local name identifier of the
        light to be removed.

        > *Returns*

        `bool`: `True`, if light could be removed, `False` if
        no light with name `tag` could be found in the world.
        """
        if tag in self._lights:
            del self._lights[tag]
            PCG_ROOT_LOGGER.info('Light removed, name={}, group={}'.format(
                tag, self.name))
            return True
        return False

    def light_exists(self, tag):
        """Test if a light with name `tag` exists in the world description.

        > *Input arguments*

        * `tag` (*type:* `str`): Local name identifier of the light.

        > *Returns*

        `bool`: `True`, if light exists, `False`, otherwise.
        """
        return tag in self._lights

    def to_sdf(self, type=None, use_include=True):
        if type not in [None, 'sdf', 'world', 'model']:
            PCG_ROOT_LOGGER.error(
                'A model group can be converted to an <sdf>, '
                '<world> or <model> unique SDF element or returned'
                ' as separate SDF elements for each entity by using'
                ' type as None, received={}'.format(type))
            return None
        from . import is_gazebo_model
        from ..parsers.sdf import create_sdf_element

        if type == 'sdf':
            sdf = create_sdf_element('sdf')
            if self.n_lights > 0 and self.n_models > 0:
                sdf.world = self.to_sdf(type='world')
                return sdf

        sdf_models = dict()
        sdf_lights = dict()
        sdf_includes = dict()

        with_group_prefix = type is None
        use_group_pose = type is None

        models = self.get_models(
            with_group_prefix=with_group_prefix,
            use_group_pose=use_group_pose)
        PCG_ROOT_LOGGER.info(
            'Retrieving models from group <{}>, '
            'with_group_prefix={}, use_group_pose={}'.format(
                self.name, with_group_prefix, use_group_pose))
        for name in models:
            if (models[name].is_gazebo_model or is_gazebo_model(name)) and \
                    use_include:
                include = create_sdf_element('include')
                include.uri = 'model://' + models[name].source_model_name
                include.pose = list(
                    models[name].pose.position) + list(models[name].pose.rpy)
                include.name = name
                include.static = models[name].static
                sdf_includes[name] = include
            else:
                sdf_models[name] = models[name].to_sdf(type='model')

        lights = self.get_lights(
            with_group_prefix=with_group_prefix,
            use_group_pose=use_group_pose)
        for name in lights:
            sdf_lights[name] = lights[name].to_sdf()

        if type is None:
            return sdf_models, sdf_lights, sdf_includes

        # Create a model for the model group
        if len(sdf_models) or len(sdf_includes):
            sdf_model_group = create_sdf_element('model')
            sdf_model_group.pose = self._pose.to_sdf()

            for tag in sdf_models:
                sdf_model_group.add_model(tag, sdf_models[tag])

            for tag in sdf_includes:
                sdf_model_group.add_include(tag, sdf_includes[tag])
        else:
            sdf_model_group = None

        if type == 'world':
            sdf_world = create_sdf_element('world')
            for tag in sdf_lights:
                sdf_world.add_light(tag, sdf_lights[tag])
            if sdf_model_group is not None:
                sdf_world.add_model(self.name, sdf_model_group)
            return sdf_world
        elif type == 'sdf':
            sdf = create_sdf_element('sdf')
            if self.n_lights > 0 and self.n_models == 0:
                for tag in sdf_lights:
                    sdf.add_light(tag, sdf_lights[tag])
            elif self.n_lights == 0 and self.n_models > 0:
                sdf.add_model(self.name, sdf_model_group)
            else:
                return None
            return sdf
        elif type == 'model':
            if self.n_lights > 0:
                PCG_ROOT_LOGGER.error(
                    '[{}] A model does cannot have lights, '
                    'n_lights={}, light names={}'.format(
                        self.name, self.n_lights, list(
                            self.lights.keys())))
                return None
            return sdf_model_group
        return None

    @staticmethod
    def from_sdf(sdf):
        group = None
        if isinstance(sdf, list):
            group = ModelGroup()
            for elem in sdf:
                if elem.xml_element_name == 'model':
                    model = SimulationModel.from_sdf(elem)
                    if model is None:
                        PCG_ROOT_LOGGER.error(
                            'Failed to load model={}'.format(elem))
                    else:
                        group.add_model(model.name, model)
                elif elem.xml_element_name == 'light':
                    light = Light.from_sdf(elem)
                    if light is None:
                        PCG_ROOT_LOGGER.error(
                            'Failed to load light={}'.format(elem))
                    else:
                        group.add_light(light.name, light)
                elif elem.xml_element_name == 'include':
                    group.add_include(elem)
                elif elem.xml_element_name == 'actor':
                    PCG_ROOT_LOGGER.warning(
                        'Adding actors not implemented yet')
        elif sdf.xml_element_name == 'sdf':
            group = ModelGroup()
            if sdf.models is not None:
                for elem in sdf.models:
                    model = SimulationModel.from_sdf(elem)
                    if model is None:
                        PCG_ROOT_LOGGER.error(
                            'Failed to load model={}'.format(elem))
                    else:
                        group.add_model(model.name, model)

            if sdf.lights is not None:
                for elem in sdf.lights:
                    light = Light.from_sdf(elem)
                    if light is None:
                        PCG_ROOT_LOGGER.error(
                            'Failed to load light={}'.format(elem))
                    else:
                        group.add_light(light.name, light)

            # TODO Add load actors function
        return group

    @staticmethod
    def from_gazebo_model(name):
        from . import get_gazebo_model_sdf
        PCG_ROOT_LOGGER.info('Importing a Gazebo model, name={}'.format(name))
        sdf = get_gazebo_model_sdf(name)
        return ModelGroup.from_sdf(sdf)

    def to_gazebo_model(
            self,
            output_dir=None,
            author=None,
            description=None,
            sdf_version='1.6',
            email=None,
            model_name=None,
            model_metaname=None,
            overwrite=False,
            nested=True):
        import os
        import getpass
        from . import is_gazebo_model, get_gazebo_model_path, \
            load_gazebo_models
        from ..parsers.sdf_config import create_sdf_config_element

        PCG_ROOT_LOGGER.info(
            'Exporting <{}> model group as a Gazebo model'.format(
                self.name))
        if output_dir is None:
            # Store the model in $HOME/.gazebo/models
            output_dir = os.path.join(
                os.path.expanduser('~'), '.gazebo', 'models')
            if not os.path.isdir(output_dir):
                os.makedirs(output_dir)
        elif isinstance(output_dir, str):
            assert os.path.isdir(
                output_dir), 'Invalid output directory, dir={}'.format(
                    output_dir)

        if model_name is None:
            model_name = self.name

        if model_metaname is None:
            model_metaname = model_name

        PCG_ROOT_LOGGER.info(
            'Output directory for Gazebo model <{}> = {}'.format(
                model_name, output_dir))

        if author is None or not isinstance(author, str):
            author = getpass.getuser()

        if email is None or not isinstance(email, str):
            email = '{}@email.com'.format(getpass.getuser())

        if description is None or not isinstance(description, str):
            description = ''

        PCG_ROOT_LOGGER.info('Gazebo model details <{}>:'.format(model_name))
        PCG_ROOT_LOGGER.info('\t - Original model name: {}'.format(self.name))
        PCG_ROOT_LOGGER.info('\t - Name: {}'.format(model_metaname))
        PCG_ROOT_LOGGER.info('\t - Author: {}'.format(author))
        PCG_ROOT_LOGGER.info('\t - E-mail: {}'.format(email))
        PCG_ROOT_LOGGER.info('\t - Description: {}'.format(description))
        PCG_ROOT_LOGGER.info('\t - SDF version: {}'.format(sdf_version))

        # Check if a model with the same name already exists
        # in the folder or in the Gazebo resource path
        if is_gazebo_model(model_name):
            existing_model_path = get_gazebo_model_path(model_name)
            PCG_ROOT_LOGGER.warning(
                'Another model <{}> was found at {}'.format(
                    model_name, existing_model_path))

            if '/usr/share' in existing_model_path:
                PCG_ROOT_LOGGER.error(
                    'Cannot create another model with name <{}>,'
                    ' existing model with the same name can be '
                    'found at {}'.format(model_name, existing_model_path))
                return None
            elif os.path.join(
                os.path.expanduser('~'), '.gazebo', 'models') in \
                    existing_model_path and \
                    not overwrite and \
                    output_dir != os.path.dirname(existing_model_path):
                PCG_ROOT_LOGGER.error(
                    'Another model with name <{}> can be found at {}'
                    ' and will not be overwritten'.format(
                        model_name, existing_model_path))
                return None
            elif output_dir == os.path.dirname(existing_model_path) and \
                    not overwrite:
                PCG_ROOT_LOGGER.error(
                    'Another model with name <{}> in the same output '
                    'directory {} and will not be overwritten'.format(
                        model_name, existing_model_path))
                return None

        if self.n_models > 0 and self.n_lights > 0:
            PCG_ROOT_LOGGER.error(
                'To export a model group as a model including'
                ' other entities, the model must have either '
                'models or lights, not both')
            return None

        full_model_dir = os.path.join(output_dir, model_name)

        if not os.path.isdir(full_model_dir):
            os.makedirs(full_model_dir)
            PCG_ROOT_LOGGER.info(
                'Model directory created: {}'.format(full_model_dir))

        manifest_filename = 'model.config'
        model_sdf_filename = 'model.sdf'

        # Create model manifest file
        manifest = create_sdf_config_element('model')
        manifest.name = model_metaname
        manifest.version = sdf_version
        manifest.description = description
        # Add SDF file
        manifest.add_sdf()
        manifest.sdfs[0].version = sdf_version
        manifest.sdfs[0].value = model_sdf_filename
        # Add author
        manifest.add_author()
        manifest.authors[0].name = author
        manifest.authors[0].email = email
        # Export manifest file
        manifest.export_xml(os.path.join(full_model_dir, manifest_filename))

        with open(
            os.path.join(full_model_dir, model_sdf_filename),
                'w+') as sdf_file:
            sdf_file.write('')

        load_gazebo_models(reset=True)

        if self.n_lights > 0:
            # Convert model group to SDF element with
            # nested light elements
            sdf = self.to_sdf(type='sdf')
        elif self.n_models > 0:
            if nested:
                # Convert model group to SDF element with nested
                # model elements
                sdf = self.to_sdf(type='sdf', use_include=False)
                assert sdf is not None, 'Could not convert model group to SDF'
            else:
                PCG_ROOT_LOGGER.info(
                    'Included models and light sources that are not'
                    ' Gazebo models already will be exported separately')
                # Export models that are not Gazebo models already
                for name in self._models:
                    if not is_gazebo_model(name) or overwrite:
                        PCG_ROOT_LOGGER.info(
                            'Exporting model <{}>'.format(name))

                        if isinstance(self._models[name], SimulationModel):
                            self._models[name].to_gazebo_model(
                                output_dir=output_dir,
                                author=author,
                                description=description,
                                sdf_version=sdf_version,
                                email=email,
                                model_name=name,
                                model_metaname=None,
                                overwrite=overwrite)
                        elif isinstance(self._models[name], ModelGroup):
                            self._models[name].to_gazebo_model(
                                output_dir=output_dir,
                                author=author,
                                description=description,
                                sdf_version=sdf_version,
                                email=email,
                                model_name=name,
                                model_metaname=None,
                                overwrite=overwrite,
                                nested=nested)
                    else:
                        PCG_ROOT_LOGGER.info(
                            'Model <{}> is already a Gazebo'
                            ' model, path={}'.format(
                                name, get_gazebo_model_path(name)))

                # Convert model group to SDF element with nested
                # model elements
                sdf = self.to_sdf(type='sdf', use_include=True)
                assert sdf is not None, 'Could not convert model group to SDF'

        assert sdf is not None, 'Could not convert model group to SDF'

        sdf.export_xml(
            os.path.join(
                full_model_dir,
                model_sdf_filename),
            sdf_version)

        return full_model_dir

    def spawn(self, gazebo_proxy=None, robot_namespace=None, pos=[0, 0, 0],
              rot=[0, 0, 0], reference_frame='world', timeout=30, replace=True,
              nested=False):
        from ..task_manager import GazeboProxy, is_gazebo_running
        from time import sleep, time

        if gazebo_proxy is None:
            gazebo_proxy = GazeboProxy()
        if not nested:
            models = self.get_models(with_group_prefix=True)
            for tag in models:
                success = models[tag].spawn(
                    gazebo_proxy=gazebo_proxy,
                    pos=pos,
                    rot=rot,
                    reference_frame=reference_frame,
                    timeout=timeout,
                    replace=replace
                )

                if not success:
                    return False
            return True
        else:
            sdf = self.to_sdf(type='sdf', use_include=False)
            assert sdf is not None, 'Could not convert model group to SDF'
            if robot_namespace is None:
                robot_namespace = self.name

            assert timeout >= 0, 'Timeout should be equal or greater than zero'
            start_time = time()
            while not gazebo_proxy.is_init() and time() - start_time < timeout:
                self._logger.info('Waiting for Gazebo to start...')
                sleep(0.5)

            if not is_gazebo_running(
                    ros_master_uri=gazebo_proxy.ros_config.ros_master_uri):
                self._logger.error('Gazebo is not running!')
                return False

            if replace and robot_namespace in gazebo_proxy.get_model_names():
                self._logger.info('Deleting existing model first')
                if gazebo_proxy.delete_model(robot_namespace):
                    self._logger.info('Done')
                else:
                    self._logger.error('Failed to delete existing model')
                    return False
            return gazebo_proxy.spawn_sdf_model(
                robot_namespace,
                sdf.to_xml_as_str(),
                pos,
                rot,
                reference_frame)
