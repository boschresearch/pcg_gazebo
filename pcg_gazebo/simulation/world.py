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
from __future__ import print_function
import os
import numpy as np
import trimesh
import datetime
from shapely.geometry import Polygon, MultiPolygon, MultiPoint
from shapely.ops import unary_union
from . import Light, SimulationModel, ModelGroup, Entity
from .physics import Physics, ODE, Simbody, Bullet
from .properties import Plugin, Pose, Footprint
from ..parsers import parse_xacro, parse_sdf
from ..parsers.sdf import create_sdf_element
from ..log import PCG_ROOT_LOGGER
from ..utils import is_string, is_array, get_random_point_from_shape, \
    has_string_pattern
from ..generators.occupancy import generate_occupancy_grid
from .. import random


class World(Entity):
    """Abstraction of Gazebo's world description. This class
    contains the settings configuring the world's

    * physics engine
    * models
    * lights
    * plugins
    * gravity

    and can be later exported into a `.world` file that Gazebo
    can parse and execute.

    > *Input arguments*

    * `name` (*type:* `str`, *value:* `default`): Name of the world.
    * `gravity` (*type:* `list`, *default:* `[0, 0, -9.8]`): Acceleration
    of gravity vector.
    * `engine` (*type:* `str`, *default:* `ode`): Name of the default
    physics engine, options are `ode`, `bullet` or `simbody`.
    """
    _PHYSICS_ENGINES = ['ode', 'bullet', 'simbody']

    def __init__(self, name='default', gravity=[0, 0, -9.8], engine='ode'):
        super(World, self).__init__(name=name)
        assert isinstance(gravity, list)
        assert len(gravity) == 3
        for elem in gravity:
            assert isinstance(elem, float) or isinstance(elem, int)
        assert engine in self._PHYSICS_ENGINES

        self._gravity = gravity
        self._name = name
        self._engine = engine
        self._physics = None
        self._model_groups = dict()
        self._plugins = dict()
        self._spherical_coordinates = None
        self._wind = None
        self._gui = None
        self._scene = None
        self._states = list()
        self._atmosphere = None
        self._magnetic_field = None
        self.reset_physics(engine)

    def __str__(self):
        return self.to_sdf().to_xml_as_str(pretty_print=True)

    def __ne__(self, other):
        result = self.__eq__(other)
        return not result

    def __eq__(self, other):
        if not isinstance(other, World):
            return False
        return self.to_sdf() == other.to_sdf()

    @property
    def physics(self):
        """`pcg_gazebo.simulation.physics.Physics`: Physics engine instance"""
        return self._physics

    @physics.setter
    def physics(self, value):
        assert isinstance(value, Physics)
        self._physics = value

    @property
    def pose(self):
        return None

    @pose.setter
    def pose(self, value):
        pass

    @property
    def spherical_coordinates(self):
        return self._spherical_coordinates

    @property
    def wind(self):
        return self._wind

    @property
    def gui(self):
        return self._gui

    @property
    def scene(self):
        return self._scene

    @property
    def magnetic_field(self):
        return self._magnetic_field

    @property
    def engine(self):
        """`str`: Name identifier of the physics engine"""
        return self._engine

    @property
    def gravity(self):
        """`list`: Acceleration of gravity vector"""
        return self._gravity

    @gravity.setter
    def gravity(self, value):
        assert isinstance(value, list)
        assert len(value) == 3
        for elem in value:
            assert isinstance(elem, float) or isinstance(elem, int)
        self._gravity = value

    @property
    def models(self):
        """`dict`: Models"""
        models = dict()
        for name in self._model_groups:
            models.update(
                self._model_groups[name].get_models(
                    with_group_prefix=True))
        return models

    @property
    def lights(self):
        """`dict`: Lights"""
        lights = dict()
        for name in self._model_groups:
            lights.update(
                self._model_groups[name].get_lights(
                    with_group_prefix=True))
        return lights

    @property
    def n_models(self):
        n_models = 0
        for tag in self._model_groups:
            n_models += self._model_groups[tag].n_models
        return n_models

    @property
    def n_lights(self):
        n_lights = 0
        for tag in self._model_groups:
            n_lights += self._model_groups[tag].n_lights
        return n_lights

    @property
    def model_groups(self):
        """`dict`: Model groups"""
        return self._model_groups

    def copy(self):
        world = World.from_sdf(self.to_sdf())
        return world

    def set_as_ground_plane(self, model_name):
        if '/' not in model_name and 'default' in self._model_groups:
            return self._model_groups['default'].set_as_ground_plane(
                model_name)
        else:
            group_name = model_name.split('/')[0]
            sub_model_name = model_name.replace('{}/'.format('group_name'), '')
            if group_name not in self._model_groups:
                PCG_ROOT_LOGGER.error(
                    'Model group <{}> for model <{}> does not exist'.format(
                        group_name, model_name))
                return False
            return self._model_groups[group_name].set_as_ground_plane(
                sub_model_name)
        return True

    def reset_physics(self, engine='ode', *args, **kwargs):
        """Reset the physics engine to its default configuration.

        > *Input arguments*

        * `engine` (*type:* `str`, *default:* `ode`): Name identifier
        of the physics engine, options are `ode`, `bullet` or `simbody`.
        """
        if engine == 'ode':
            self._physics = ODE(*args, **kwargs)
        elif engine == 'bullet':
            self._physics = Bullet(*args, **kwargs)
        elif engine == 'simbody':
            self._physics = Simbody(*args, **kwargs)
        else:
            raise ValueError(
                'Invalid physics engine, received={}'.format(engine))

    def reset_models(self):
        """Reset the list of models."""
        for name in self._model_groups:
            self._model_groups[name].reset_models()
        PCG_ROOT_LOGGER.info('Models groups were resetted')

    def create_model_group(self, name, pose=[0, 0, 0, 0, 0, 0]):
        if name not in self._model_groups:
            self._model_groups[name] = ModelGroup(name=name, pose=pose)
            PCG_ROOT_LOGGER.info('New model group created: {}'.format(name))
        else:
            PCG_ROOT_LOGGER.info('Model group already exists: {}'.format(name))

    def add_include(self, include, group='default'):
        """Add a model via include method.

        > *Input arguments*

        * `include` (*type:* `pcg_gazebo.parsers.sdf.Include`):
        SDF `<include>` element

        > *Returns*

        `bool`: `True`, if model directed by the `include` element
        could be parsed and added to the world.
        """
        if group not in self._model_groups:
            self.create_model_group(group)
        return self._model_groups[group].add_include(include)

    def add_model(self, tag, model, group='default'):
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
        if group not in self._model_groups:
            self.create_model_group(group)
        return self._model_groups[group].add_model(tag, model)

    def add_model_group(self, group, tag=None):
        from copy import deepcopy
        if tag is None:
            tag = deepcopy(group.name)

        if tag in self._model_groups:
            # Add counter suffix to add models with same name
            i = 0
            new_name = '{}'.format(tag)
            while new_name in self._model_groups:
                i += 1
                new_name = '{}_{}'.format(tag, i)
            name = new_name
        else:
            name = tag

        self._model_groups[name] = group
        self._model_groups[name].name = name
        return True

    def rm_model(self, tag, group='default'):
        """Remove model from world.

        > *Input arguments*

        * `tag` (*type:* `str`): Local name identifier of the
        model to be removed.

        > *Returns*

        `bool`: `True`, if model could be removed, `False` if
        no model with name `tag` could be found in the world.
        """
        if group not in self._model_groups:
            PCG_ROOT_LOGGER.error('Model group <{}> not found'.format(group))
            return False
        else:
            return self._model_groups[group].rm_model(tag)

    def model_exists(self, tag, group=None):
        """Test if a model with name `tag` exists in the world description.

        > *Input arguments*

        * `tag` (*type:* `str`): Local name identifier of the model.

        > *Returns*

        `bool`: `True`, if model exists, `False`, otherwise.
        """
        if group is None:
            for name in self._model_groups:
                if self._model_groups[name].model_exists(tag):
                    return True
            return False
        if group not in self._model_groups:
            PCG_ROOT_LOGGER.error('Model group <{}> not found'.format(group))
            return False
        else:
            return self._model_groups[group].model_exists(tag)

    def get_model(self, tag, group=None):
        if group is None:
            if 'default' in self._model_groups:
                if self._model_groups['default'].model_exists(tag):
                    return self._model_groups['default'].get_model(tag)
            if '/' in tag:
                group_name = tag.split('/')[0]
                if group_name not in self._model_groups:
                    return None
                return self._model_groups[group_name].get_model(
                    tag.replace('{}/'.format(group_name), ''))
        else:
            if group not in self._model_groups:
                PCG_ROOT_LOGGER.error(
                    'Model group <{}> does not exist'.format(group))
                return None

    def add_plugin(self, tag, plugin):
        """Add plugin description to the world.

        > *Input arguments*

        * `tag` (*type:* `str`): Name identifier for the plugin. If
        a model with the same name already exists, the model will be
        created with a counter suffix in the format `_i`, `i` being
        an integer.
        * `plugin` (*type:* `pcg_gazebo.parsers.sdf.Plugin` or
        `pcg_gazebo.simulation.properties.Plugin`): Plugin description.
        """
        if self.plugin_exists(tag):
            # Add counter suffix to add plugins with same name
            i = 0
            new_name = '{}'.format(tag)
            while self.plugin_exists(new_name):
                i += 1
                new_name = '{}_{}'.format(tag, i)
            name = new_name
        else:
            name = tag

        if not isinstance(plugin, Plugin):
            plugin = Plugin.from_sdf(plugin)

        self._plugins[name] = plugin

    def rm_plugin(self, tag):
        """Remove plugin from world.

        > *Input arguments*

        * `tag` (*type:* `str`): Local name identifier of the
        plugin to be removed.

        > *Returns*

        `bool`: `True`, if plugin could be removed, `False` if
        no plugin with name `tag` could be found in the world.
        """
        if tag in self._plugins:
            del self._plugins[tag]
            return True
        return False

    def plugin_exists(self, tag):
        """Test if a plugin with name `tag` exists in the world description.

        > *Input arguments*

        * `tag` (*type:* `str`): Local name identifier of the plugin.

        > *Returns*

        `bool`: `True`, if plugin exists, `False`, otherwise.
        """
        return tag in self._plugins

    def add_light(self, tag, light, group='default'):
        """Add light description to the world.

        > *Input arguments*

        * `tag` (*type:* `str`): Name identifier for the plugin. If
        a model with the same name already exists, the model will be
        created with a counter suffix in the format `_i`, `i` being
        an integer.
        * `light` (*type:* `pcg_gazebo.parsers.sdf.Light` or
        `pcg_gazebo.simulation.properties.Light`): Light description
        """
        if group not in self._model_groups:
            self.create_model_group(group)
        PCG_ROOT_LOGGER.info(
            'Adding light to model group, name={}, model={}, group={}'.format(
                tag, light.name, group))
        return self._model_groups[group].add_light(tag, light)

    def rm_light(self, tag, group='default'):
        """Remove light from world.

        > *Input arguments*

        * `tag` (*type:* `str`): Local name identifier of the
        light to be removed.

        > *Returns*

        `bool`: `True`, if light could be removed, `False` if
        no light with name `tag` could be found in the world.
        """
        if group not in self._model_groups:
            PCG_ROOT_LOGGER.error('Model group <{}> not found'.format(group))
            return False
        else:
            return self._model_groups[group].rm_light(tag)

    def light_exists(self, tag, group='default'):
        """Test if a light with name `tag` exists in the world description.

        > *Input arguments*

        * `tag` (*type:* `str`): Local name identifier of the light.

        > *Returns*

        `bool`: `True`, if light exists, `False`, otherwise.
        """
        if group not in self._model_groups:
            PCG_ROOT_LOGGER.error('Model group <{}> not found'.format(group))
            return False
        else:
            return self._model_groups[group].light_exists(tag)

    def to_sdf(self, type='world', with_default_ground_plane=False,
               with_default_sun=False, sdf_version='1.6'):
        """Convert the world description into as `pcg_gazebo` SDF
        element.

        > *Input arguments*

        * `type` (*type:* `str`, *default:* `world`): Type of output SDF
        element to be generated, options are `world` or `sdf`. It is
        important to note that to export the world description into a
        file, it is necessary to have the `sdf` format.
        * `with_default_ground_plane` (*type:* `bool`, *default:* `True`):
        Add Gazebo's default ground plane model to the world.
        * `with_default_sun` (*type:* `bool`, *default:* `True`):
        Add Gazebo's default sun model to the world.

        > *Returns*

        `pcg_gazebo.parsers.sdf.SDF` with a world element in it or
        `pcg_gazebo.parsers.sdf.World`.
        """
        assert type in ['sdf', 'world']
        # Create a parent element of type world to include the physics
        # configuration
        world = create_sdf_element('world')
        # Setting the physics engine
        if self._physics is not None:
            world.physics = self._physics.to_sdf('physics')

        for group_name in self._model_groups:
            sdf_models, sdf_lights, sdf_includes = \
                self._model_groups[group_name].to_sdf()

            for name in sdf_models:
                world.add_model(name, sdf_models[name])

            for name in sdf_lights:
                world.add_light(name, sdf_lights[name])

            for name in sdf_includes:
                world.add_include(include=sdf_includes[name])

        # TODO: Include plugins and actors on the exported file
        for tag in self._plugins:
            world.add_plugin(tag, self._plugins[tag].to_sdf())

        if self._wind is not None:
            world.wind = self._wind

        if self._gui is not None:
            world.gui = self._gui

        if self._scene is not None:
            world.scene = self._scene

        if self._atmosphere is not None:
            world.atmosphere = self._atmosphere

        if self._magnetic_field is not None:
            world.magnetic_field = self._magnetic_field

        if len(self._states) > 0:
            for state in self._states:
                world.add_state(
                    name=state.world_name, state=state)

        if with_default_ground_plane:
            ground_plane = create_sdf_element('include')
            ground_plane.uri = 'model://ground_plane'
            world.add_include(None, ground_plane)

        if with_default_sun:
            sun = create_sdf_element('include')
            sun.uri = 'model://sun'
            world.add_include(None, sun)

        if type == 'world':
            return world

        sdf = create_sdf_element('sdf')
        sdf.world = world
        sdf.sdf_version = sdf_version

        return sdf

    @staticmethod
    def from_sdf(sdf):
        """Parse an `pcg_gazebo.parsers.sdf.World` into a
        `World` class.

        > *Input arguments*

        * `sdf` (*type:* `pcg_gazebo.parsers.sdf.World`):
        SDF world element

        > *Returns*

        `pcg_gazebo.parsers.sdf.World` instance.
        """
        sdf_tree = None
        if is_string(sdf):
            if sdf.endswith('.xacro'):
                sdf_tree = parse_xacro(sdf, output_type='sdf')
            elif sdf.endswith('.world') or sdf.endswith('.sdf'):
                sdf_tree = parse_sdf(sdf)
            else:
                sdf_tree = parse_sdf(sdf)
        else:
            sdf_tree = sdf

        if sdf_tree.xml_element_name == 'sdf':
            if sdf_tree.world is not None:
                sdf_tree = sdf_tree.world

        if sdf_tree.xml_element_name != 'world':
            msg = 'SDF element must be of type <world>'
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)

        world = World()
        if sdf_tree.models is not None:
            for model in sdf_tree.models:
                if '/' in model.name:
                    group_name = model.name.split('/')[0]
                    model_name = model.name.replace(group_name + '/', '')
                else:
                    group_name = 'default'
                    model_name = model.name
                world.add_model(
                    model_name,
                    SimulationModel.from_sdf(model),
                    group=group_name)

        if sdf_tree.lights is not None:
            for light in sdf_tree.lights:
                if '/' in light.name:
                    group_name = light.name.split('/')[0]
                    light_name = light.name.replace(group_name + '/', '')
                else:
                    group_name = 'default'
                    light_name = light.name
                world.add_light(
                    light_name,
                    Light.from_sdf(light),
                    group=group_name)

        if sdf_tree.plugins is not None:
            for plugin in sdf_tree.plugins:
                world.add_plugin(plugin.name, Plugin.from_sdf(plugin))

        if sdf_tree.includes is not None:
            for inc in sdf_tree.includes:
                try:
                    if inc.name is not None:
                        if '/' in inc.name.value:
                            group_name = inc.name.value.split('/')[0]
                            inc_name = inc.name.value.replace(
                                group_name + '/', '')
                        else:
                            group_name = 'default'
                            inc_name = inc.name.value
                        inc.name.value = inc_name
                    else:
                        group_name = 'default'
                    world.add_include(inc, group=group_name)
                except ValueError as ex:
                    PCG_ROOT_LOGGER.error(
                        'Cannot import model <{}>, message={}'.format(
                            inc.uri.value, ex))

        if sdf_tree.gravity is not None:
            world.gravity = sdf_tree.gravity.value

        if sdf_tree.spherical_coordinates is not None:
            world._spherical_coordinates = \
                sdf_tree.spherical_coordinates

        if sdf_tree.wind is not None:
            world._wind = sdf_tree.wind

        if sdf_tree.gui is not None:
            world._gui = sdf_tree.gui

        if sdf_tree.scene is not None:
            world._scene = sdf_tree.scene

        if sdf_tree.states is not None:
            world._states = sdf_tree.states

        if sdf_tree.atmosphere is not None:
            world._atmosphere = sdf_tree.atmosphere

        if sdf_tree.magnetic_field is not None:
            world._magnetic_field = sdf_tree.magnetic_field

        world.physics = Physics.from_sdf(sdf_tree.physics)
        world.name = sdf_tree.name

        return world

    def create_scene(self, mesh_type='collision', add_pseudo_color=True,
                     ignore_models=None, add_axis=True):
        """Return a `trimesh.Scene` with all the world's models.

        > *Input arguments*

        * `mesh_type` (*type:* `str`, *default:* `collision`): Type of mesh
        to be included in the scene, options are `collision` or `visual`.
        * `add_pseudo_color` (*type:* `bool`, *default:* `True`): If `True`,
        set each mesh with a pseudo-color.
        """
        from ..visualization import create_scene
        if ignore_models is None:
            ignore_models = list()

        def _is_ignored(name):
            for item in ignore_models:
                if has_string_pattern(name, item):
                    return True
            return False

        return create_scene(
            [self.models[tag] for tag in self.models if not _is_ignored(tag)],
            mesh_type,
            add_pseudo_color,
            add_axis=add_axis)

    def to_model_group(self, include_models=None, ignore_models=None):
        from . import ModelGroup
        group = ModelGroup()

        if include_models is None:
            include_models = list(self.models.keys()) + \
                list(self.lights.keys())

        if ignore_models is None:
            ignore_models = list()

        def _is_included(name):
            for item in include_models:
                if has_string_pattern(name, item):
                    return True
            return False

        def _is_ignored(name):
            for item in ignore_models:
                if has_string_pattern(name, item):
                    return True
            return False
        for tag in self.models:
            if _is_included(tag) and not _is_ignored(tag):
                group.add_model(tag, self.models[tag])

        for tag in self.lights:
            if _is_included(tag) and not _is_ignored(tag):
                group.add_light(tag, self.lights[tag])

        return group

    def export_as_mesh(
            self,
            mesh_type='collision',
            add_pseudo_color=True,
            format='dae',
            filename=None,
            folder=None,
            ignore_models=None):
        if ignore_models is None:
            ignore_models = list()
        scene = self.create_scene(
            mesh_type, add_pseudo_color, ignore_models=ignore_models)

        export_formats = ['obj', 'dae', 'stl']
        if format not in export_formats:
            PCG_ROOT_LOGGER.error(
                'Invalid mesh export format, options={}'.format(
                    export_formats))
            return None
        if folder is not None:
            assert os.path.isdir(folder), \
                'Invalid output folder, provided={}'.format(folder)
        else:
            folder = '/tmp'

        if filename is not None:
            assert is_string(filename), 'Filename is not a string'
        else:
            filename = 'world'

        mesh_filename = os.path.join(
            folder,
            filename.split('.')[0] + '.' + format)

        if format == 'obj':
            trimesh.exchange.export.export_mesh(
                scene,
                mesh_filename,
                file_type=format,
                include_color=False,
                include_texture=False)
        elif format == 'dae':
            trimesh.exchange.export.export_mesh(
                scene.dump(),
                mesh_filename,
                file_type=format)
        elif format == 'stl':
            mesh = trimesh.boolean.union(scene.dump())
            trimesh.exchange.export.export_mesh(
                mesh,
                mesh_filename,
                file_type=format)

        return mesh_filename

    def plot_footprints(
            self,
            fig=None,
            ax=None,
            fig_width=20,
            fig_height=20,
            mesh_type='collision',
            z_limits=None,
            colormap='magma',
            grid=True,
            ignore_ground_plane=True,
            line_width=1,
            line_style='solid',
            alpha=0.5,
            engine='matplotlib',
            dpi=200,
            ground_plane_models=None):
        """Plot the mesh footprint projections on the XY plane.

        > *Input arguments*

        * `fig` (*type:* `matplotlib.pyplot.Figure` or
        `bokeh.plotting.Figure` , *default:* `None`):
        Figure object. If `None` is provided, the figure will be created.
        * `ax` (*type:* `matplotlib.pyplot.Axes`, *default:* `None`):
        Axes object to add the plot. If `None` is provided, the axes
        object will be created.
        * `fig_width` (*type:* `float` or `int`, *default:* `20`):
        Width of the figure in inches, if `engine` is `matplotlib`,
        or pixels, if `engine` is `bokeh`.
        * `fig_height` (*type:* `float` or `int`, *default:* `20`):
        Height of the figure in inches, if `engine` is `matplotlib`,
        or pixels, if `engine` is `bokeh`.
        * `mesh_type` (*type:* `str`, *default:* `collision`): Type
        of mesh to consider for the footprint computation, options
        are `collision` and `visual`.
        * `z_limits` (*type:* `list`, *default:* `None`): List
        of minimum and maximum Z-levels to consider when sectioning
        the meshes.
        * `colormap` (*type:* `str`, *default:* `magma`): Name of the
        colormap to be used.
        Check [this link](https://matplotlib.org/users/colormaps.html)
        for `matplotlib` colormaps and
        [this link](
        https://bokeh.pydata.org/en/latest/docs/reference/palettes.html)
        for `bokeh` colormaps.
        * `grid` (*type:* `bool`, *default:* `True`): If `True`,
        add grid to the plot.
        * `ignore_ground_plane` (*type:* `bool`, *default:* `True`):
        Ignore the models flagged as ground plane from the plot.
        * `line_width` (*type:* `float`, *default:* `1`):
        Width of the line of each footprint polygon patch.
        * `line_style` (*type:* `str`, *default:* `solid`):
        Style of the line of each footprint polygon patch.
        Check this
        [link](https://matplotlib.org/3.1.0/gallery/lines_bars_and_markers/linestyles.html)
        to see all the line style options.
        * `alpha` (*type:* `float`, *default:* `0.5`): Alpha
        channel value for the footprint objects.
        * `engine` (*type:* `str`, *default:* `matplotlib`):
        Engine to use for the generation of the figure, options
        are `bokeh` and `matplotlib`.
        * `dpi` (*type:* `int`, *default:* `200`): Image's DPI

        > *Returns*

        `matplotlib.pyplot.Figure` or `bokeh.plotting.Figure`.
        """
        if mesh_type not in ['collision', 'visual']:
            msg = 'Mesh type to compute the footprints' \
                ' must be either collision or visual' \
                ' geometries, provided={}'.format(mesh_type)
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)

        if z_limits is None:
            PCG_ROOT_LOGGER.info(
                'Plotting footprints using Z limits: {}'.format(z_limits))

        from ..visualization import plot_footprints
        fig, ax = plot_footprints(
            models=self.models,
            fig=fig,
            ax=ax,
            fig_height=fig_height,
            fig_width=fig_width,
            mesh_type=mesh_type,
            engine=engine,
            line_style=line_style,
            line_width=line_width,
            grid=grid,
            ignore_ground_plane=ignore_ground_plane,
            z_limits=z_limits,
            colormap=colormap,
            dpi=dpi,
            alpha=alpha
        )

        PCG_ROOT_LOGGER.info('Plotting footprints: finished')
        return fig, ax

    def save_occupancy_grid(
            self,
            occupied_thresh=0.65,
            free_thresh=0.196,
            occupied_color=[0, 0, 0],
            free_color=[1, 1, 1],
            unavailable_color=[0.5, 0.5, 0.5],
            output_folder='/tmp',
            output_filename='map.pgm',
            static_models_only=False,
            with_ground_plane=True,
            z_levels=None,
            x_limits=None,
            y_limits=None,
            z_limits=None,
            n_processes=None,
            fig_size=(5, 5),
            fig_size_unit='cm',
            dpi=200,
            axis_x_limits=None,
            axis_y_limits=None,
            exclude_contains=None,
            mesh_type='collision',
            ground_plane_models=None):
        from ..visualization import plot_occupancy_grid
        assert os.path.isdir(output_folder), \
            'Output folder is invalid, folder={}'.format(output_folder)
        if output_filename is None:
            output_filename = 'map.pgm'
        plot_occupancy_grid(
            self.models,
            occupied_thresh=occupied_thresh,
            free_thresh=free_thresh,
            occupied_color=occupied_color,
            free_color=free_color,
            unavailable_color=unavailable_color,
            output_folder=output_folder,
            output_filename=output_filename,
            static_models_only=static_models_only,
            with_ground_plane=with_ground_plane,
            z_levels=z_levels,
            x_limits=x_limits,
            y_limits=y_limits,
            z_limits=z_limits,
            n_processes=n_processes,
            fig_size=fig_size,
            fig_size_unit=fig_size_unit,
            dpi=dpi,
            axis_x_limits=axis_x_limits,
            axis_y_limits=axis_y_limits,
            exclude_contains=exclude_contains,
            mesh_type=mesh_type,
            ground_plane_models=ground_plane_models)
        return os.path.join(output_folder, output_filename)

    def is_free_space(self, model, pose, ignore_models=None,
                      static_collision_checker=None,
                      return_collision_checker=False):
        from ..generators import CollisionChecker
        test_model = None
        if is_string(model):
            test_model = SimulationModel.from_gazebo_model(model)
        elif isinstance(model, SimulationModel):
            test_model = model.copy()
        else:
            raise ValueError(
                'Input model must be either the name of the Gazebo'
                ' model available in GAZEBO_RESOURCE_PATH or in the'
                ' ROS paths, or a SimulationModel object, received={}'.format(
                    type(model)))

        if ignore_models is None:
            ignore_models = list()

        PCG_ROOT_LOGGER.info(
            'Check if model <{}> is in free space, pose={}'.format(
                model.name, pose.to_sdf()))
        test_model.pose = pose

        if static_collision_checker is None:
            PCG_ROOT_LOGGER.info('Creating a collision checker')
            collision_checker = CollisionChecker()

            # Add models to the collision checker
            PCG_ROOT_LOGGER.info(
                'Populating collision checker, # models={}'.format(
                    len(self.models)))
            for tag in self.models:
                for item in ignore_models:
                    if not has_string_pattern(self.models[tag].name, item):
                        collision_checker.add_model(self.models[tag])
            no_collision = \
                not collision_checker.check_collision_with_current_scene(
                    test_model)
            if return_collision_checker:
                return no_collision, collision_checker
            else:
                return no_collision
        else:
            PCG_ROOT_LOGGER.info('Using static collision checker')
            no_collision = \
                not static_collision_checker.check_collision_with_current_scene(  # noqa: E501
                    test_model)
            return no_collision

    def get_bounds(self, mesh_type='collision'):
        from copy import deepcopy
        bounds = None
        PCG_ROOT_LOGGER.info('Compute world <{}> bounds'.format(self.name))
        for tag in self.models:
            model_bounds = self.models[tag].get_bounds()
            if bounds is None:
                bounds = deepcopy(model_bounds)
            else:
                cur_bounds = deepcopy(model_bounds)
                for i in range(3):
                    bounds[0, i] = min(bounds[0, i], cur_bounds[0, i])
                for i in range(3):
                    bounds[1, i] = max(bounds[1, i], cur_bounds[1, i])
        PCG_ROOT_LOGGER.info('World <{}> bounds={}'.format(self.name, bounds))
        return bounds

    def get_free_space_polygon(
            self,
            ground_plane_models=None,
            ignore_models=None,
            x_limits=None,
            y_limits=None,
            free_space_min_area=5e-3):
        if ground_plane_models is None:
            ground_plane_models = list()
        else:
            assert is_array(ground_plane_models), \
                'ground_plane_models must be a list'
        if ignore_models is None:
            ignore_models = list()
        else:
            assert is_array(ignore_models), \
                'ignore_models must be a list'

        if self.n_models == 0:
            assert x_limits is not None, \
                'For an empty world, the x_limits cannot be None'
            assert x_limits[0] < x_limits[1], \
                'Invalid X limits, value={}'.format(x_limits)
            assert y_limits is not None, \
                'For an empty world, the y_limits cannot be None'
            assert y_limits[0] < y_limits[1], \
                'Invalid Y limits, value={}'.format(y_limits)
            return Polygon(
                [
                    [x_limits[0], y_limits[0]],
                    [x_limits[0], y_limits[1]],
                    [x_limits[1], y_limits[1]],
                    [x_limits[1], y_limits[0]],
                    [x_limits[0], y_limits[0]]]
            )

        bounds = self.get_bounds()
        if x_limits is not None:
            assert is_array(x_limits), 'X limits must be an array'
            assert x_limits[0] < x_limits[1], \
                'Invalid X limits, value={}'.format(x_limits)
        else:
            x_limits = [bounds[0, 0], bounds[1, 0]]

        if y_limits is not None:
            assert is_array(y_limits), 'Y limits must be an array'
            assert y_limits[0] < y_limits[1], \
                'Invalid Y limits, value={}'.format(y_limits)
        else:
            y_limits = [bounds[0, 1], bounds[1, 1]]

        free_space_polygon = Polygon(
            [
                [x_limits[0], y_limits[0]],
                [x_limits[0], y_limits[1]],
                [x_limits[1], y_limits[1]],
                [x_limits[1], y_limits[0]],
                [x_limits[0], y_limits[0]]]
        )

        if len(ground_plane_models) == 0:
            for tag in self.models:
                if self.models[tag].is_ground_plane:
                    is_ignored = False
                    for item in ignore_models:
                        if has_string_pattern(self.models[tag].name, item):
                            is_ignored = True
                            break
                    if not is_ignored:
                        ground_plane_models.append(tag)

        filtered_models = dict()
        for tag in self.models:
            if self.models[tag].is_ground_plane:
                filtered_models[tag] = self.models[tag]
            else:
                for item in ground_plane_models:
                    if has_string_pattern(self.models[tag].name, item):
                        filtered_models[tag] = self.models[tag]
                        break

        if len(filtered_models) == 0:
            return free_space_polygon

        PCG_ROOT_LOGGER.info(
            'List of models to compute free space polygon={}'.format(
                list(filtered_models.keys())))

        PCG_ROOT_LOGGER.info(filtered_models)

        occupancy_output = generate_occupancy_grid(
            filtered_models,
            n_processes=4,
            mesh_type='collision',
            ground_plane_models=ground_plane_models)

        if occupancy_output is None:
            return free_space_polygon

        static_footprints = unary_union(
            [occupancy_output['static'][tag]
             for tag in occupancy_output['static']])

        dynamic_footprints = unary_union(
            [occupancy_output['non_static'][tag]
             for tag in occupancy_output['non_static']])

        if occupancy_output['ground_plane'] is not None:
            diff_st_gp = occupancy_output['ground_plane'].difference(
                static_footprints)

            if not diff_st_gp.is_empty and \
                    (diff_st_gp.area /
                        occupancy_output['ground_plane'].area) > 1e-2:
                free_space_polygon = free_space_polygon.intersection(
                    occupancy_output['ground_plane'])

        free_space_polygon = free_space_polygon.difference(
            static_footprints)

        free_space_polygon = free_space_polygon.difference(
            dynamic_footprints)

        free_space_polygon = free_space_polygon.buffer(-1e-3)
        # Remove small free spaces
        if isinstance(free_space_polygon, MultiPolygon):
            for geo in free_space_polygon.geoms:
                if geo.area <= free_space_min_area:
                    free_space_polygon = free_space_polygon.difference(
                        geo.buffer(1e-5))
        free_space_polygon = free_space_polygon.simplify(tolerance=1e-4)
        free_space_polygon = free_space_polygon.buffer(1e-3)

        return free_space_polygon

    def get_random_free_spots(
            self,
            model,
            n_spots=1,
            ground_plane_models=None,
            ignore_models=None,
            x_limits=None,
            y_limits=None,
            z_limits=None,
            roll_limits=None,
            pitch_limits=None,
            yaw_limits=None,
            free_space_min_area=5e-3,
            dofs=None,
            return_free_polygon=False,
            show_preview_2d=False,
            show_preview_3d=False,
            verbose=True,
            max_num_tests=None):
        assert n_spots > 0, 'Number of free spots must be greater than zero'
        if max_num_tests is not None:
            assert max_num_tests > 0, \
                'Stopping criteria to stop trying to find a random' \
                ' free spot must be greater than 0'
        if is_string(model):
            test_model = SimulationModel.from_gazebo_model(model)
        elif isinstance(model, SimulationModel):
            test_model = model.copy()
        else:
            raise ValueError(
                'Input model must be either the name of the Gazebo'
                ' model available in GAZEBO_RESOURCE_PATH or in the'
                ' ROS paths, or a SimulationModel object, received={}'.format(
                    type(model)))

        PCG_ROOT_LOGGER.info(
            'Compute free spaces for model <{}> in world <{}>'.format(
                test_model.name, self.name))

        if z_limits is not None:
            assert z_limits[0] < z_limits[1], \
                'Invalid Z limits, z_limits={}'.format(z_limits)
        else:
            z_limits = [0, 1e4]

        PCG_ROOT_LOGGER.info('Z limits to find free spaces={}'.format(
            z_limits))

        if roll_limits is not None:
            assert roll_limits[0] < roll_limits[1], \
                'Invalid roll limits, roll_limits={}'.format(roll_limits)
        else:
            roll_limits = [-np.pi, np.pi]

        PCG_ROOT_LOGGER.info('Roll limits in rad={}'.format(
            roll_limits))

        if pitch_limits is not None:
            assert pitch_limits[0] < pitch_limits[1], \
                'Invalid pitch limits, pitch_limits={}'.format(pitch_limits)
        else:
            pitch_limits = [-np.pi, np.pi]

        PCG_ROOT_LOGGER.info('Pitch limits in rad={}'.format(
            pitch_limits))

        if yaw_limits is not None:
            assert yaw_limits[0] < yaw_limits[1], \
                'Invalid yaw limits, yaw_limits={}'.format(yaw_limits)
        else:
            yaw_limits = [-np.pi, np.pi]

        PCG_ROOT_LOGGER.info('Yaw limits in rad={}'.format(
            yaw_limits))

        active_dofs = list()
        if dofs is None:
            active_dofs = ['x', 'y']
        else:
            ref_dofs = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
            assert is_array(dofs), \
                'dofs must be a list of active DoFs to vary'
            for item in dofs:
                assert item in ref_dofs, \
                    'Invalid dofs tag provided, dof={}'.format(item)
            active_dofs = dofs
            if 'x' not in active_dofs:
                active_dofs.append('x')
            if 'y' not in active_dofs:
                active_dofs.append('y')

        assert len(active_dofs) > 0, 'List of active DoFs cannot be empty'

        PCG_ROOT_LOGGER.info('Active DoFs={}'.format(active_dofs))

        footprints = test_model.get_footprint()
        combined_footprint = Footprint()
        for tag in footprints:
            combined_footprint.add_polygon(footprints[tag])
        model_footprint = combined_footprint.get_footprint_polygon()

        if model_footprint.is_empty:
            PCG_ROOT_LOGGER.error('Model provided has no valid footprint')
            return None

        PCG_ROOT_LOGGER.info('Computing free space polygon')

        free_space_polygon = self.get_free_space_polygon(
            ground_plane_models=ground_plane_models,
            ignore_models=ignore_models,
            x_limits=x_limits,
            y_limits=y_limits,
            free_space_min_area=free_space_min_area)

        if free_space_polygon.is_empty:
            PCG_ROOT_LOGGER.error(
                'No free space found for the X and Y limits'
                ' given, x_limits={}, y_limits={}'.format(
                    x_limits, y_limits))
            return None

        PCG_ROOT_LOGGER.info('Area of free space polygon={}'.format(
            free_space_polygon.area))

        if model_footprint.area >= free_space_polygon.area:
            PCG_ROOT_LOGGER.error(
                'Model is too big for the available free space')
            return None

        poses = list()
        collision_checker = None
        n_tries = 0
        while len(poses) < n_spots:
            if max_num_tests is not None:
                if n_tries >= max_num_tests:
                    PCG_ROOT_LOGGER.warning(
                        'Max. number of free random spots exceeded={},'
                        ' # spots found={}'.format(
                            max_num_tests, len(poses)))
                    break
            pose = Pose()
            # Generate random point
            xy = get_random_point_from_shape(free_space_polygon)

            pose.x = xy[0]
            pose.y = xy[1]

            # Computing Z component
            if z_limits is not None and 'z' in active_dofs:
                pose.z = random.rand() * (z_limits[1] - z_limits[0]) \
                    + z_limits[0]

            # Computing roll, pitch and yaw
            roll = 0
            if 'roll' in active_dofs:
                roll = random.rand() * (roll_limits[1] - roll_limits[0]) \
                    + roll_limits[0]

            pitch = 0
            if 'pitch' in active_dofs:
                pitch = random.rand() * \
                    (pitch_limits[1] - pitch_limits[0]) \
                    + pitch_limits[0]

            yaw = 0
            if 'yaw' in active_dofs:
                yaw = random.rand() * (yaw_limits[1] - yaw_limits[0]) \
                    + yaw_limits[0]

            pose.rpy = [roll, pitch, yaw]

            # Add the model pose to the computed pose
            pose = pose + test_model.pose

            PCG_ROOT_LOGGER.info('Testing pose={}'.format(pose.to_sdf()))

            if collision_checker is None:
                is_free_space, collision_checker = \
                    self.is_free_space(
                        test_model,
                        pose,
                        ignore_models=ignore_models,
                        return_collision_checker=True)
            else:
                is_free_space = \
                    self.is_free_space(
                        test_model,
                        pose,
                        ignore_models=ignore_models,
                        static_collision_checker=collision_checker,
                        return_collision_checker=True)
            if is_free_space:
                PCG_ROOT_LOGGER.info('Storing free space pose={}'.format(
                    pose.to_sdf()))
                poses.append(pose)
                n_tries = 0
            else:
                PCG_ROOT_LOGGER.info('Collision detected for pose={}'.format(
                    pose.to_sdf()))
                n_tries += 1

        if show_preview_2d:
            PCG_ROOT_LOGGER.info('Plotting free spots in 2D')
            from ..visualization import plot_shapely_geometry
            import matplotlib.pyplot as plt

            fig, ax = plot_shapely_geometry(
                polygon=free_space_polygon)
            fig, ax = plot_shapely_geometry(
                polygon=MultiPoint([[p.x, p.y] for p in poses]),
                fig=fig,
                ax=ax
            )
            plt.show()

        if show_preview_3d:
            PCG_ROOT_LOGGER.info('Plotting free spots in 3D')
            scene = self.create_scene()
            for p in poses:
                temp_model = test_model.copy()
                temp_model.pose = p
                scene.add_geometry(temp_model.get_meshes())
            scene.show()

        if return_free_polygon:
            return poses, free_space_polygon
        else:
            return poses

    def export_to_file(
            self, output_dir=None, filename=None,
            with_default_ground_plane=True, with_default_sun=True,
            models_output_dir=None, overwrite=True, sdf_version='1.6'):
        """Export world to an SDF file that can be used by Gazebo.

        > *Input arguments*

        * `output_dir` (*type:* `str`, *default:* `None`): Path
        to output directory to store the world file.
        * `filename` (*type:* `str`, *default:* `None`): Name of the
        SDF world file
        * `with_default_ground_plane` (*type:* `bool`, *default:*
        `True`): Add the default ground plane model to the world before
        exporting it
        * `with_default_sun` (*type:* `bool`, *default:* `True`): Add
        the default sun model to the world before exporting it

        > *Returns*

        Full name of the exported SDF world file as a `str`
        """
        assert output_dir is not None, 'Output directory cannot be None'
        from . import is_gazebo_model

        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)
            PCG_ROOT_LOGGER.info(
                'Output directory {} created'.format(output_dir))

        if filename is None:
            timestamp = datetime.datetime.now().isoformat()
            timestamp = timestamp.replace(':', '_')
            world_filename = '{}_gazebo.world'.format(timestamp)
        elif isinstance(filename, str):
            world_filename = filename
            if '.world' not in world_filename:
                world_filename += '.world'
        else:
            PCG_ROOT_LOGGER.error('Invalid world filename={}'.format(filename))
            return None

        for tag in self.models:
            if self.models[tag].has_mesh and \
                    not is_gazebo_model(
                        self.models[tag].source_model_name,
                        include_custom_paths=True):
                self.models[tag].to_gazebo_model(
                    sdf_version=sdf_version,
                    output_dir=models_output_dir,
                    overwrite=overwrite,
                    copy_resources=True)

        full_world_filename = os.path.join(output_dir, world_filename)
        sdf = self.to_sdf(
            type='sdf',
            sdf_version=sdf_version,
            with_default_ground_plane=with_default_ground_plane,
            with_default_sun=with_default_sun)
        sdf.export_xml(os.path.join(output_dir, world_filename))
        PCG_ROOT_LOGGER.info('World stored in {}'.format(full_world_filename))
        return full_world_filename
