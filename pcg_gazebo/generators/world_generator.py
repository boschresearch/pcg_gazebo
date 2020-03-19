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
from time import sleep, time
from .. import visualization
from ..log import PCG_ROOT_LOGGER
from ._generator import _Generator
from ..utils import load_yaml
from ..task_manager import GazeboProxy, is_gazebo_running
from ..parsers.sdf import create_sdf_element, is_sdf_element
from ..simulation import World
from ..simulation.physics import ODE, Simbody, Bullet


class WorldGenerator(_Generator):
    """Generation of full Gazebo worlds, including physics engine configuration,
    modes and lights.

    > *Input arguments*

    * `gazebo_proxy` (*type:* `pcg_gazebo.task_manager.GazeboProxy`,
    *default:* `None`): A `GazeboProxy` object to enable spawning of models
    and configuration of the simulation in runtime.

    """

    def __init__(self, name='default', gazebo_proxy=None,
                 output_world_dir=None, output_model_dir='/tmp/gazebo_models'):
        super(WorldGenerator, self).__init__(name=name)
        if gazebo_proxy is not None:
            assert isinstance(gazebo_proxy, GazeboProxy)
            self._gazebo_proxy = gazebo_proxy

        if output_world_dir is None:
            self._output_world_dir = '/tmp/gazebo_worlds'

        PCG_ROOT_LOGGER.info(
            'Output path for the Gazebo world files: {}'.format(
                self._output_world_dir))

        # Set of world descriptions
        self.init()

    def __str__(self):
        msg = 'PCG Generator\n'
        msg += '\t - Assets:\n'
        for tag in self._assets.tags:
            msg += '\t\t - Name = {}, Is Gazebo model? {}\n'.format(
                tag, self._assets.is_gazebo_model(tag))
        return msg

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
        return self._simulation_entity

    def init(self, name=None):
        if name is None:
            name = self._name
        self._simulation_entity = World(name=name)

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
        super(WorldGenerator, self).from_dict(config)

        if 'physics' in config:
            if 'engine' in config['physics']:
                physics_engine = config['physics']['engine']
            else:
                physics_engine = 'ode'

            if 'args' in config['physics']:
                physics_args = config['physics']['args']
            else:
                physics_args = dict()
            self._simulation_entity.reset_physics(
                engine=physics_engine, **physics_args)

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
        self._simulation_entity = World(
            name=name, engine=engine, gravity=gravity)

    def set_physics_engine(self, engine, *args, **kwargs):
        if engine in ['ode', 'bullet', 'simbody']:
            self._simulation_entity.reset_physics(engine, *args, **kwargs)
        elif isinstance(engine, ODE) or isinstance(engine, Simbody) or \
                isinstance(engine, Bullet):
            self._simulation_entity.physics = engine

    def export_world(
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
        if output_dir is None:
            output_dir = self._output_world_dir
        return self.world.export_to_file(
            output_dir=output_dir,
            filename=filename,
            with_default_ground_plane=with_default_ground_plane,
            with_default_sun=with_default_sun,
            models_output_dir=models_output_dir,
            overwrite=overwrite,
            sdf_version=sdf_version)

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

        self._simulation_entity = World.from_sdf(sdf)

    def init_from_jinja(self, jinja_filename, params=dict()):
        from ..utils import process_jinja_template
        from ..parsers import parse_sdf
        output_xml = process_jinja_template(jinja_filename, parameters=params)
        sdf = parse_sdf(output_xml)

        self._simulation_entity = World.from_sdf(sdf)
