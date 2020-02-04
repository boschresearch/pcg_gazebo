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
import os
import datetime
from time import sleep, time
from .. import visualization
from ..log import PCG_ROOT_LOGGER
from ..utils import load_yaml, generate_random_string
from ..task_manager import GazeboProxy, is_gazebo_running
from ..parsers.sdf import create_sdf_element, is_sdf_element
from ..simulation import SimulationModel, Light, World
from ..simulation.physics import ODE, Simbody, Bullet
from .assets_manager import AssetsManager
from .engine_manager import EngineManager


class WorldGenerator:
    """Generation of full Gazebo worlds, including physics engine configuration,
    modes and lights.

    > *Input arguments*

    * `gazebo_proxy` (*type:* `pcg_gazebo.task_manager.GazeboProxy`,
    *default:* `None`): A `GazeboProxy` object to enable spawning of models
    and configuration of the simulation in runtime.

    """

    def __init__(self, gazebo_proxy=None, output_world_dir=None,
                 output_model_dir='/tmp/gazebo_models'):
        if gazebo_proxy is not None:
            assert isinstance(gazebo_proxy, GazeboProxy)
            self._gazebo_proxy = gazebo_proxy

        if output_world_dir is None:
            self._output_world_dir = '/tmp/gazebo_worlds'

        PCG_ROOT_LOGGER.info(
            'Output path for the Gazebo world files: {}'.format(
                self._output_world_dir))

        # Set of world descriptions
        self._world = World()
        self._assets = AssetsManager.get_instance()
        self._engines = EngineManager()
        self._name = None

    def __str__(self):
        msg = 'PCG Generator\n'
        msg += '\t - Assets:\n'
        for tag in self._assets.tags:
            msg += '\t\t - Name = {}, Is Gazebo model? {}\n'.format(
                tag, self._assets.is_gazebo_model(tag))
        return msg

    @property
    def name(self):
        """`str`: Name of the generated world"""
        return self._name

    @property
    def gazebo_proxy(self):
        """`pcg_gazebo.task_manager.GazeboProxy`: Internal instance
        of the `GazeboProxy`
        """
        return self._gazebo_proxy

    @property
    def world(self):
        """`pcg_gazebo.simulation.World`: World abstraction
        instance
        """
        return self._world

    @property
    def assets(self):
        """List of `pcg_gazebo.simulation.SimulationModel`: List of model
        assets that will be used of the world generation.
        """
        return self._assets

    @property
    def engines(self):
        """`dict` of `pcg_gazebo.generators.engines`: Dictionary with the
        model creation engines.
        """
        return self._engines

    @property
    def constraints(self):
        """`dict` of `pcg_gazebo.generators.constraints`: Dictionary with the
        positioning constraints.
        """
        return self._engines.constraints_manager

    def init_gazebo_proxy(
            self,
            ros_host='localhost',
            ros_port=11311,
            gazebo_host='localhost',
            gazebo_port=11345,
            timeout=30,
            ignore_services=None):
        """Initialize a `GazeboProxy` instance to interface with a running
        instance of Gazebo. If a `GazeboProxy` already exists, it will be
        deleted before a new one is created.

        > *Input arguments*

        * `ros_host` (*type:* `str`, *default:* `localhost`): Address of the
        ROS host machine running `roscore`.
        * `ros_port` (*type:* `int`, *default:* `11311`): Port number for
        `roscore`
        * `gazebo_host` (*type:* `str`, *default:* `localhost`): Address of the
        Gazebo server
        * `gazebo_port` (*type:* `int`, *default:* `11345`): Port number of
        the Gazebo server
        """
        if self._gazebo_proxy is not None:
            del self._gazebo_proxy
        self._gazebo_proxy = GazeboProxy(
            ros_host=ros_host,
            ros_port=ros_port,
            gazebo_host=gazebo_host,
            gazebo_port=gazebo_port,
            timeout=timeout,
            ignore_services=ignore_services)
        PCG_ROOT_LOGGER.info(
            'New Gazebo proxy initialized, ROS configuration: {}'.format(
                self._gazebo_proxy.ros_config.__str__()))

    def add_engine(self, tag, engine_name, models, **kwargs):
        """Add a new model creator engine to the internal engines list.

        > *Input arguments*

        * `engine_name` (*type:* `str`): Name of the engine class
        to be created
        * `models` (*type:* list of `str`): Name of the models that
        will be assets to the created engine
        * `kwargs` (*type:* `dict`): Input arguments to the created engine.
        """
        self._engines.add(tag, engine_name, models, **kwargs)

    def add_constraint(self, name, type, **kwargs):
        """Add a new positioning constraint class to the internal
        constraints list.

        > *Input arguments*

        * `name` (*type:* `str`): ID name for the constraint class instance
        * `type` (*type:* `str`): Name of the constraints class to be created
        * `kwargs` (*type:* `dict`): Input arguments for the constraint class
        to be created
        """
        return self._engines.add_constraint(name, type, **kwargs)

    def add_asset(self, *args, **kwargs):
        """Add a new model asset that can be used by the engines and
        added to the generated world.

        > *Input arguments*

        * `model` (*type:* `pcg_gazebo.simulation.SimulationModel`):
        Simulation model
        """
        self._assets.add(*args, **kwargs)

    def set_model_as_ground_plane(self, model_name):
        """Flag a model asset as part of the ground plane. This procedure will
        affect the collision checks during the automatic placement of models in
        the world using the placement engines.

        > *Input arguments*

        * `model_name` (*type:* `str`): Name of the model asset
        """
        return self._assets.set_asset_as_ground_plane(model_name)

    def get_asset(self, name):
        """Return a simulation model asset.

        > *Input arguments*

        * `name` (*type:* `str`): Name of the model asset.

        > *Returns*

        The model asset as `pcg_gazebo.simulation.SimulationModel`.
        `None` if `name` cannot be found in the list of model assets.
        """
        return self._assets.get(name)

    def get_constraint(self, name):
        """Return a positioning constraint configuration.

        > *Input arguments*

        * `param` (*type:* `data_type`, *default:* `data`):
        Parameter description

        > *Returns*

        Description of return values
        """
        return self._engines.constraints_manager.get(name)

    def add_gazebo_model_as_asset(self, gazebo_model_name):
        """Create a model asset by importing a Gazebo model that already
        exists in the resources path of the catkin workspace. The model's
        SDF file will be parsed and converted into a
        `pcg_gazebo.simulation.SimulationModel` instance.

        Models that include lights can also be added, but will not be
        considered assets, they will just be included into the generated
        world SDF file.

        > *Input arguments*

        * `gazebo_model_name` (*type:* `str`): ID name from the Gazebo model
        to be imported

        > *Returns*

        `True` if Gazebo model could be included in the assets list.
        """
        from ..simulation import get_gazebo_model_sdf
        sdf = get_gazebo_model_sdf(gazebo_model_name)

        if hasattr(sdf, 'lights') and sdf.lights is not None:
            return self.add_lights_from_gazebo_model(gazebo_model_name)
        else:
            try:
                model = SimulationModel.from_gazebo_model(gazebo_model_name)
            except ValueError as ex:
                PCG_ROOT_LOGGER.error(
                    'Error loading Gazebo model <{}>, message={}'.format(
                        gazebo_model_name, str(ex)))
                return False

            if model is None:
                PCG_ROOT_LOGGER.error(
                    'Gazebo model with name <{}> '
                    'could not be found'.format(gazebo_model_name))
                return False
            self.add_asset(model)
        return True

    def is_asset(self, name):
        """Return `True` if the model identified by the string `name`
        is part of the list of assets.

        > *Input arguments*

        * `name` (*type:* `str`): Name of the model
        """
        return name in self._assets.tags

    def add_model(self, model, poses):
        """Add an instance of `pcg_gazebo.simulation.SimulationModel` to
        the world in designed poses.

        > *Input arguments*

        * `model` (*type:* `pcg_gazebo.simulation.SimulationModel`):
        Parameter description
        * `poses` (*type:* `list`): List of 6D pose vectors
        """
        self.add_asset(model)
        self.add_engine(
            tag=generate_random_string(5),
            engine_name='fixed_pose',
            models=[model.name],
            poses=poses)

    def add_gazebo_model(self, model_name, pose=[0, 0, 0, 0, 0, 0]):
        """Add an existent Gazebo model to the world in designed poses.

        > *Input arguments*

        * `model_name` (*type:* `str`): ID name of the Gazebo model
        * `pose` (*type:* `list`): 6D pose vector
        """
        if not self.is_asset(model_name):
            self.add_gazebo_model_as_asset(model_name)
        model = SimulationModel.from_gazebo_model(model_name)

        # Model with the same name is already in the world list
        # Add ID identifier to avoid errors when spawning
        i = 0
        new_model_name = '{}'.format(model_name)
        while self._world.model_exists(new_model_name):
            i += 1
            new_model_name = '{}_{}'.format(model_name, i)

        model.name = new_model_name
        # Add new model
        self.add_model(model, pose)
        PCG_ROOT_LOGGER.info(
            'Model <{}> added to world description'.format(model_name))

    def remove_asset(self, name):
        """Remove model asset from the list of assets.

        > *Input arguments*

        * `name` (*type:* `str`): Name of the model

        > *Returns*

        `True`, if model could be removed.
        """
        return self._assets.remove(name)

    def delete_model(self, model_name):
        """Delete a model from the currently running Gazebo instance

        > *Input arguments*

        * `model_name` (*type:* `str`): Name of the model

        > *Returns*

        `True` if the model could be deleted from the simulation.
        """
        if self._gazebo_proxy is None:
            PCG_ROOT_LOGGER.error('Gazebo proxy was not initialized')
            return False
        if not is_gazebo_running(
                ros_master_uri=self._gazebo_proxy.ros_config.ros_master_uri):
            return False
        if self._gazebo_proxy.is_init():
            return self._gazebo_proxy.delete_model(model_name)
        else:
            return False

    def add_lights_from_gazebo_model(self, model_name):
        """Add light models to the generated world from a Gazebo model.

        > *Input arguments*

        * `model_name` (*type:* `str`): Name of the Gazebo model

        > *Returns*

        `True` if the lights could be parsed and added to the world.
        """
        from ..simulation import get_gazebo_model_sdf
        sdf = get_gazebo_model_sdf(model_name)
        if sdf is None:
            PCG_ROOT_LOGGER.error(
                'Model {} is not a Gazebo model'.format(model_name))
            return False
        if sdf.lights is not None:
            PCG_ROOT_LOGGER.info(
                'Input Gazebo model {} contains'
                ' lights, adding to world'.format(
                    model_name))
            for light in sdf.lights:
                PCG_ROOT_LOGGER.info('Add light {}, type={}'.format(
                    light.name, light.type))
                self.world.add_light(light.name, Light.from_sdf(light))
            return True
        else:
            PCG_ROOT_LOGGER.error(
                'Input Gazebo model contains no '
                'lights, model_name={}'.format(model_name))
            return False

    def from_yaml(self, filename):
        config = load_yaml(filename)
        self.from_dict(config)

    def from_dict(self, config):
        """Parse a configuration settings `dict` with all
        information on the list of model assets, engines,
        constraints and lights and instantiate the
        necessary objects.

        An example of a YAML file that can hold this kind of
        information can be seen below:

        ```yaml
        name: world_name
        assets:
        - model_1       # This list holds only Gazebo models
        - model_2
        - model_3
        ground_plane:   # Optional input
        - model_1       # If model_1 is part of the ground_plane,
                        # it should be flagged for collision checking
        constraints:
        - name: kitchen                     # Name identifier
            type: workspace                 # Name of the constraint class
            frame: world
            geometry:
            type: area
            description:
                points:
                - [-6.54833, -4.17127, 0]
                - [-3.24447, -4.17127, 0]
                - [-3.24447, 0.12423, 0]
                - [-6.54833, 0.12423, 0]
        - name: tangent_to_ground_plane     # Name identifier
            type: tangent                   # Name of the constraint class
            frame: world
            reference:
            type: plane
            args:
                origin: [0, 0, 0]
                normal: [0, 0, 1]
        engines:
        - engine_name: fixed_pose
          models:
          - sll_room_empty
          poses:
          - [0, 0, 0, 0, 0, 0]
        - engine_name: random_pose
          models:
          - sll_table_group_futura_seat
          - sll_table_group_futura
          model_picker: size
          max_area: 0.9
          no_collision: false
          max_num:
            sll_table_group_futura_seat: 6
            sll_table_group_futura: 1
          policies:
            - models:
                - sll_table_group_futura_seat
                - sll_table_group_futura
                config:
                - dofs:
                - x
                - y
                policy:
                    name: workspace
                    args: dining_room
                - dofs:
                - z
                - roll
                - pitch
                policy:
                    name: value
                    args: 0
                - dofs:
                - yaw
                policy:
                    name: uniform
                    args:
                    mean: 0
                    min: -3.141592653589793
                    max: 3.141592653589793
          constraints:
            - model: sll_table_group_futura
                constraint: tangent_to_ground_plane
            - model: sll_table_group_futura_seat
                constraint: tangent_to_ground_plane
        lights:
        - name: sun     # Name of the Gazebo model with the light data
        ```

        > *Input arguments*

        * `config` (*type:* `dict`): Configuration settings
        for the world generator

        > *Returns*

        Description of return values
        """
        from ..simulation import is_gazebo_model
        assert isinstance(
            config, dict), 'Input configuration must be a dictionary'

        if 'name' in config:
            self._name = config['name']
            PCG_ROOT_LOGGER.info('Generator name: {}'.format(self._name))

        if 'physics' in config:
            if 'engine' in config['physics']:
                physics_engine = config['physics']['engine']
            else:
                physics_engine = 'ode'

            if 'args' in config['physics']:
                physics_args = config['physics']['args']
            else:
                physics_args = dict()
            self._world.reset_physics(engine=physics_engine, **physics_args)

        if 'assets' in config:
            self._assets.from_dict(config['assets'])

        if 'ground_plane' in config:
            for tag in config['ground_plane']:
                self.set_model_as_ground_plane(tag)

                PCG_ROOT_LOGGER.info(
                    'Set model {} as ground plane'.format(tag))

        if 'engines' in config:
            if not isinstance(config['engines'], list):
                PCG_ROOT_LOGGER.error(
                    '<engines> element in dictionary must be a list')
            else:
                self._engines.from_dict(config['engines'])

            PCG_ROOT_LOGGER.info('Engines:')

            for eng in self._engines.tags:
                PCG_ROOT_LOGGER.info('\t - {}'.format(eng))

        if 'constraints' in config:
            if not isinstance(config['constraints'], list):
                PCG_ROOT_LOGGER.error(
                    '<constraints> element in dictionary must be a list')
            for elem in config['constraints']:
                assert isinstance(
                    elem, dict), 'Constraint description' \
                    ' is not a dictionary={}'.format(elem)
                self._engines.add_constraint(**elem)

            PCG_ROOT_LOGGER.info('Constraints:')

            for tag in self._engines.constraints_manager.tags:
                PCG_ROOT_LOGGER.info(
                    '\t - {}'.format(
                        self._engines.constraints_manager.get(tag)))

        if 'lights' in config:
            PCG_ROOT_LOGGER.info('Lights:')
            for light in config['lights']:
                if 'name' not in light:
                    PCG_ROOT_LOGGER.error('Light item has no name')
                    continue
                if is_gazebo_model(light['name']):
                    self.add_lights_from_gazebo_model(light['name'])

    def spawn_model(self, model, robot_namespace, pos=[0, 0, 0], rot=[0, 0, 0],
                    reference_frame='world', timeout=30, replace=False):
        """Spawn a `pcg_gazebo.simulation.SimulationModel` in
        a running instance of Gazebo. A `GazeboProxy` is required
        for this method to finish successfully.

        > *Input arguments*

        * `model` (*type:* `pcg_gazebo.simulation.SimulationModel`):
        Simulation model to be spawned
        * `robot_namespace` (*type:* `str`): Name under which
        the robot should be spawned in Gazebo
        * `pos` (*type:* `list`, *default:* `[0, 0, 0]`):
        Spawning position wrt reference frame
        * `rot` (*type:* `list`, *default:* `[0, 0, 0]`):
        Roll-Pitch-Yaw angles in radians or a (w, i, j, k)
        quaternion vector.
        * `reference_frame` (*type:* `str`, *default:* `world`):
        Reference frame for the spawning pose
        * `timeout` (*type:* `float`): Timeout in seconds to
        wait for Gazebo to start
        * `replace` (*type:* `bool`, *default:* `False`):
        Replace the model in the simulation in case a model
        with the same name already exists.

        > *Returns*

        `True` if the model could be spawned.
        """
        if self._gazebo_proxy is None:
            PCG_ROOT_LOGGER.error('Gazebo proxy was not initialized')
            return False
        assert timeout >= 0, 'Timeout should be equal or greater than zero'
        start_time = time()
        while not self._gazebo_proxy.is_init() and \
                time() - start_time < timeout:
            PCG_ROOT_LOGGER.info('Waiting for Gazebo to start...')
            sleep(0.5)

        if not is_gazebo_running(
                ros_master_uri=self._gazebo_proxy.ros_config.ros_master_uri):
            PCG_ROOT_LOGGER.error('Gazebo is not running!')
            return False

        if replace and robot_namespace in self._gazebo_proxy.get_model_names():
            PCG_ROOT_LOGGER.info('Deleting existing model first')
            if self._gazebo_proxy.delete_model(robot_namespace):
                PCG_ROOT_LOGGER.info('Done')
            else:
                PCG_ROOT_LOGGER.error('Failed to delete existing model')
                return False

        sdf = model.to_sdf(type='model')
        assert sdf._NAME == 'model', 'SDF element must be of type model'
        assert is_sdf_element(sdf), 'Input is not an SDF element'
        sdf_root = create_sdf_element('sdf')
        sdf_root.reset(mode='model')
        sdf_root.add_model(model=sdf)

        return self._gazebo_proxy.spawn_sdf_model(
            robot_namespace,
            sdf_root.to_xml_as_str(),
            pos,
            rot,
            reference_frame)

    def get_physics_engine(self, engine='ode'):
        """Return an instance of a physics engine as
        `pcg_gazebo.simulation.physics.Physics` object.

        > *Input arguments*

        * `engine` (*type:* `str`): ID name of the physics
        engine, options are `ode`, `bullet` and `simbody`.

        > *Returns*

        An `pcg_gazebo.simulation.physics.Physics` object.
        """
        if engine == 'ode':
            return ODE()
        elif engine == 'bullet':
            return Bullet()
        elif engine == 'simbody':
            return Simbody()
        else:
            return None

    def run_engines(self, attach_models=False):
        """Run all the model placement engines and add the generated
        models in the internal instance of the world representation.

        > *Input arguments*

        * `attach_models` (*type:* `bool`, *default:* `False`): Attach
        the generated models to the existent list of models in the world

        > *Returns*

        `True` if all engines ran successfully.
        """
        if self._engines.size == 0:
            PCG_ROOT_LOGGER.warning('No engines found')
            return False
        if not attach_models:
            self._world.reset_models()
            PCG_ROOT_LOGGER.info('List of models is now empty')

        models = list()
        # Run the fixed pose engines first
        PCG_ROOT_LOGGER.info('Run fixed-pose engines')
        for tag in self._engines.tags:
            engine = self._engines.get(tag)
            if engine.label == 'fixed_pose':
                models = engine.run()
                if models is not None:
                    for model in models:
                        if isinstance(model, SimulationModel):
                            PCG_ROOT_LOGGER.info(
                                'Adding model {} to world'.format(
                                    model.name))
                            self._world.add_model(model.name, model)
                        else:
                            PCG_ROOT_LOGGER.info(
                                'Adding model group {} to world'.format(
                                    model.name))
                            self._world.add_model_group(model, model.name)

        # Run all other engines
        PCG_ROOT_LOGGER.info('Run other engines')
        for tag in self._engines.tags:
            engine = self._engines.get(tag)
            if engine.label != 'fixed_pose':
                PCG_ROOT_LOGGER.info(
                    'Running engine, type={}'.format(
                        engine.label))
                engine.set_fixed_pose_models(list(self._world.models.values()))
                models = engine.run()
                if models is not None:
                    for model in models:
                        if isinstance(model, SimulationModel):
                            PCG_ROOT_LOGGER.info(
                                'Adding model {} to world'.format(
                                    model.name))
                            self._world.add_model(model.name, model)
                        else:
                            PCG_ROOT_LOGGER.info(
                                'Adding model group {} to world'.format(
                                    model.name))
                            self._world.add_model_group(model, model.name)

        PCG_ROOT_LOGGER.info(
            'World model generation'
            'finished, # models={}, model_names={}'.format(
                len(self._world.models), list(self._world.models.keys())))
        return True

    def reset_world(self, name, engine='ode', gravity=[0, 0, -9.8]):
        """Reset the generated world instance to its default state and
        without any models.

        > *Input arguments*

        * `name` (*type:* `str`): Name of the world
        * `engine` (*type:* `str`, *default:* `ode`): Name of the
        physics engine to be used. Options are `ode`, `bullet` or `simbody`.
        * `gravity` (*type:* `list`, *default:* `[0, 0, -9.8]`): Gravitational
        acceleration vector
        """
        self._world = World(name=name, engine=engine, gravity=gravity)

    def set_physics_engine(self, engine, *args, **kwargs):
        if engine in ['ode', 'bullet', 'simbody']:
            self._world.reset_physics(engine, *args, **kwargs)
        elif isinstance(engine, ODE) or isinstance(engine, Simbody) or \
                isinstance(engine, Bullet):
            self._world.physics = engine

    def export_world(self, output_dir=None, filename=None,
                     with_default_ground_plane=True, with_default_sun=True):
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
        if output_dir is None:
            world_dir = self._output_world_dir
        elif isinstance(output_dir, str):
            world_dir = output_dir
        else:
            PCG_ROOT_LOGGER.error(
                'Directory path must be a string, provided={}, type={}'.format(
                    output_dir, type(output_dir)))
            return None

        if not os.path.isdir(world_dir):
            os.makedirs(world_dir)
            PCG_ROOT_LOGGER.info(
                'Output directory {} created'.format(world_dir))

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

        full_world_filename = os.path.join(world_dir, world_filename)
        sdf = self._world.to_sdf(
            type='sdf',
            with_default_ground_plane=with_default_ground_plane,
            with_default_sun=with_default_sun)
        sdf.export_xml(os.path.join(world_dir, world_filename))
        PCG_ROOT_LOGGER.info('World stored in {}'.format(full_world_filename))
        return full_world_filename

    def plot_results(self, fig=None, fig_width=1000, fig_height=800,
                     footprint_geometry='collision', engine='bokeh'):
        """Plot the footprints of models included in the current world
        instance.

        > *Input arguments*

        * `fig` (*type:* a `bokeh` or a `matplotlib` figure,
        *default:* `None`): A figure object. If `fig` is `None`,
        a new figure will be created
        * `fig_width` (*type:* `int`, *default:* `1000`): Width
        of the figure
        * `param` (*type:* `data_type`, *default:* `data`):
        Parameter description

        > *Returns*

        Description of return values
        """
        fig = None

        models = self.world.models
        if self._engines.constraints_manager.size:
            ws_constraints = dict()
            for tag in self._engines.constraints_manager.tags:
                if self._engines.constraints_manager.get(
                        tag)._LABEL == 'workspace':
                    ws_constraints[tag] = \
                        self._engines.constraints_manager.get(
                            tag)
            PCG_ROOT_LOGGER.info(
                'Plotting workspaces={}'.format(
                    list(
                        ws_constraints.keys())))
            fig = visualization.plot_workspaces(
                ws_constraints,
                fig=fig,
                fig_width=fig_width,
                fig_height=fig_height,
                alpha=0.3,
                line_width=2,
                line_style='dashdot',
                engine=engine)

        if len(models):
            PCG_ROOT_LOGGER.info(
                'Plotting model footprints={}'.format(
                    list(
                        models.keys())))
            fig = visualization.plot_footprints(
                models=models,
                fig=fig,
                fig_width=fig_width,
                fig_height=fig_height,
                mesh_type=footprint_geometry,
                n_processes=4,
                alpha=0.5,
                line_width=2,
                engine=engine)

        # if engine == 'matplotlib':
        #     ax = fig.gca()
        #     ax.axis('equal')
        #     ax.grid(True)
        #     ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        #     ax.autoscale(enable=True, axis='both', tight=True)

        PCG_ROOT_LOGGER.info('Plotting footprints: finished')

        return fig

    def init_from_sdf(self, sdf_input):
        if is_sdf_element(sdf_input):
            sdf = sdf_input
        else:
            from ..parsers import parse_sdf
            sdf = parse_sdf(sdf_input)

        self._world = World.from_sdf(sdf)

    def init_from_jinja(self, jinja_filename, params=dict()):
        from ..utils import process_jinja_template
        from ..parsers import parse_sdf
        output_xml = process_jinja_template(jinja_filename, parameters=params)
        sdf = parse_sdf(output_xml)

        self._world = World.from_sdf(sdf)
