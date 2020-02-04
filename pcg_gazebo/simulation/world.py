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
from .physics import Physics, ODE, Simbody, Bullet
from .properties import Plugin
from . import Light, SimulationModel, ModelGroup
from ..parsers.sdf import create_sdf_element
from ..log import PCG_ROOT_LOGGER


class World(object):
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
        assert isinstance(name, str)
        assert len(name) > 0
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
        self.reset_physics(engine)

    @property
    def physics(self):
        """`pcg_gazebo.simulation.physics.Physics`: Physics engine instance"""
        return self._physics

    @physics.setter
    def physics(self, value):
        assert isinstance(value, Physics)
        self._physics = value

    @property
    def name(self):
        """`str`: Name of the world"""
        return self._name

    @name.setter
    def name(self, value):
        assert isinstance(value, str)
        assert len(value) > 0
        self._name = value

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
        return name

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

    def to_sdf(self, type='world', with_default_ground_plane=True,
               with_default_sun=True):
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
        if sdf.xml_element_name == 'sdf':
            if sdf.world is not None:
                sdf = sdf.world

        if sdf.xml_element_name != 'world':
            msg = 'SDF element must be of type <world>'
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)

        world = World()
        if sdf.models is not None:
            for model in sdf.models:
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

        if sdf.lights is not None:
            for light in sdf.lights:
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

        if sdf.plugins is not None:
            for plugin in sdf.plugins:
                world.add_plugin(plugin.name, Plugin.from_sdf(plugin))

        if sdf.includes is not None:
            for inc in sdf.includes:
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

        if sdf.gravity is not None:
            world.gravity = sdf.gravity.value

        world.physics = Physics.from_sdf(sdf.physics)
        world.name = sdf.name

        return world

    def create_scene(self, mesh_type='collision', add_pseudo_color=True):
        """Return a `trimesh.Scene` with all the world's models.

        > *Input arguments*

        * `mesh_type` (*type:* `str`, *default:* `collision`): Type of mesh
        to be included in the scene, options are `collision` or `visual`.
        * `add_pseudo_color` (*type:* `bool`, *default:* `True`): If `True`,
        set each mesh with a pseudo-color.
        """
        from ..visualization import create_scene
        return create_scene(
            list(
                self.models.values()),
            mesh_type,
            add_pseudo_color)

    def show(self, mesh_type='collision', add_pseudo_color=True):
        from trimesh.viewer.notebook import in_notebook
        scene = self.create_scene(mesh_type, add_pseudo_color)
        if not in_notebook():
            scene.show()
        else:
            from trimesh.viewer import SceneViewer
            return SceneViewer(scene)

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
            dpi=200):
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
        fig = plot_footprints(
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
        return fig
