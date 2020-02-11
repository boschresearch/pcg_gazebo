This folder contains a number of examples demonstrating the features
of the `pcg_gazebo` package.
To run the Jupyter notebooks, be sure to install it as

```
pip install jupyterlab
```

# Model generation

## Creating full Gazebo models in Python

In the [example on creating Gazebo models in Python](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sim_creating_models_with_jupyter_notebooks.ipynb), it is demonstrated how to use the tools from the `simulation` module in order to create simulation entities and can be spawned directly into Gazebo.

## Model group generator

In the [model group generator example notebook](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sim_model_group_generators.ipynb), it is demonstrated how to create a model group abstraction in which the model's parameters are re-calculated each time an instance of the model group is created. 
This allows using a single representation of the model group generator as an asset and generating its variations as an instance is created.

## Model factory

In the [model factory example notebook](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sim_model_factory.ipynb), it is demonstrated how to use the `creators` module to create parametrized simulation models and spawn them in Gazebo.

## Generating models from a YAML configuration file

This examples shows how to define a simulation model as a YAML file
and generate static Gazebo models to be stored in your local `$HOME/.pcg/models`
folder.
This example requires you to run the script in the `examples` folder

```
./gen_model_from_factory_config.sh CONFIG_NAME
``` 
 
where `CONFIG_NAME` refers to the name of one the files (without the `.yaml` 
extension) in the `examples/model_factory` folder.
For example, by running

```
./gen_model_from_factory_config.sh box_dynamic_model
```

will generate a box model with mass and moments of inertia and store it in
the folder `$HOME/.pcg/models/pcg_box_dynamic`, including `model.config` and 
`model.sdf` files.

Some YAML files include a batch of model descriptions and therefore will
produce multiple Gazebo models in the `$HOME/.pcg/models` folder.

This example runs the chosen YAML file with the [`run_model_factory`](https://github.com/boschresearch/pcg_gazebo/blob/master/scripts/run_model_factory) script,
that feeds the data into the model factory functions found in the [model factory module](https://github.com/boschresearch/pcg_gazebo/blob/master/pcg_gazebo/generators/creators.py).

To find out more about how to run the model factory from script, run

```
run_model_factory -h
```

To generate the model from the YAML file configuration and spawn it into 
Gazebo, first start Gazebo as

```
roslaunch gazebo_ros empty_world.launch
```

and then use the same `CONFIG_NAME` to run the script

```
./spawn_model_from_factory_config.sh CONFIG_NAME
```

## Generating models from template Jinja files

This examples shows how generate an SDF file for a Gazebo model
from a Jinja template and store static Gazebo models in your local 
`$HOME/.pcg/models` folder.
This example requires you to run the script in the `examples` folder

```
./gen_model_from_template.sh TEMPLATE_NAME
``` 
 
where `TEMPLATE_NAME` refers to the name of one the files (without the `.yaml` 
extension) in the `examples/templates/models` folder.
For example, by running

```
./gen_model_from_template.sh pcg_bouncy_ball
```

will generate single-link spherical model with the necessary settings to allow
it to bounce when colliding with another object.
The model is per default stored in the folder `$HOME/.pcg/models/pcg_bouncy_ball`, including `model.config` and `model.sdf` files.

To find out more about how to run the Jinja processor from script, run

```
process_jinja_template -h
```

This [notebook](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sim_inspect_robot_description.ipynb) also shows an example on how the `kobuki` robot (converted to a [Jinja template to generate the robot description in SDF format](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/robot_description/kobuki/sdf/kobuki.sdf.jinja)) is processed from its templates and imported as an `SimulationModel` object that can be edited and spawned into the simulation.

# World generation

## Dynamically generating worlds from YAML configuration file

The `pcg_gazebo` package includes not only model creators but also engines
that allow creating and placing objects in the world according to pre-defined
policies and respecting certain constraints.

The sample configurations that allow worlds to be dynamically created can be found
in `examples/world_generator/worlds` and by running 

```
./launch_pcg_world.sh WORLD_CONFIG_NAME
```

`WORLD_CONFIG_NAME` being the name of the world configuration file (without the `.yaml` extension) in the `examples/world_generator/worlds` folder.
The resulting world file will be stored in `$HOME/.pcg/worlds` folder.

For example, by calling

```
./launch_pcg_world.sh bouncing_balls_ode
```

generates an environment of 40 spheres placed randomly in the 3D world, all 
of them with the bounce settings enabled so that they will bounce against
the ground and each other.

# Occupancy grid map generation

Occupancy grid maps can be computed from single models, model groups or worlds using ray tracing.
In this [notebook](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/gen_grid_map.ipynb) it is demonstrated how a grid map can be plotted and stored from a generated world.

# Parsers

## Parsing `xacro` files

This [example](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sim_inspect_robot_description.ipynb) it is shown how to import a `xacro` file that will be processed, rendered into an URDF file and imported as a `SimulationModel` object.

# List of Jupyter notebooks

## Simulation

To run some of the notebooks below you may need to source your current ROS distribution and have Gazebo installed in your system since they will also run the simulation.
To source ROS, use

```bash
source /opt/ros/$ROS_DISTRO/setup.bash  # You can also replace $ROS_DISTRO by the distro you are using, e.g. melodic
```

The installation instructions for Gazebo can be found [here](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install).

* [Testing the model's surface collision properties](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sim_surface_collision_properties.ipynb)
* [Using the model factory](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sim_model_factory.ipynb)
* [Inspecting Gazebo models](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sim_inspection_of_gazebo_models.ipynb)
* [Creating models with Jupyter notebooks](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sim_creating_models_with_jupyter_notebooks.ipynb)
* [Stopping Gazebo with a simulation timeout](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sim_run_gazebo_with_simulation_timeout.ipynb)
* [Using model group generators](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sim_model_group_generators.ipynb)
* [Creating sensors](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sim_sensors.ipynb)
* [Creating single-link models](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sim_objects.ipynb)
* [Stopping Gazebo with a process timeout](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sim_run_gazebo_with_process_timeout.ipynb)
* [Handling meshes](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sim_meshes.ipynb)
* [Configuring the different physics engines](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sim_physics_engines.ipynb)
* [Inspecting different formats of robot descriptions](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sim_inspect_robot_description.ipynb)
* [Handling Gazebo materials](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sim_materials.ipynb)

## SDF parsers

The examples for the `sdf` parser is available as Jupyter notebooks.
The list can be seen below.

* [Parsing `<collision>` elements](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sdf_parser_collisions.ipynb)
* [Importing SDF file](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sdf_parser_convert_from_sdf_file.ipynb)
* [Parsing `<geometry>` elements](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sdf_parser_geometries.ipynb)
* [Parsing `<link>`, `<joint>` and `<sensor>` elements](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sdf_parser_links_joints_sensors.ipynb)
* [Parsing `<materials>` elements](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sdf_parser_materials.ipynb)
* [Parsing `<model>` elements](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sdf_parser_models.ipynb)
* [Parsing `<physics>` elements](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sdf_parser_physics_engines.ipynb)
* [Parsing `<plugin>` elements](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sdf_parser_plugins.ipynb)
* [Parsing `<visual>` elements](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sdf_parser_visuals.ipynb)
* [Parsing `<world>` elements](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/sdf_parser_world.ipynb)

## URDF parsers

The examples for the `urdf` parser is available as Jupyter notebooks.
The list can be seen below.

* [Parsing `<collision>` elements](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/urdf_parser_collision.ipynb)
* [Parsing `<geometry>` elements](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/urdf_parser_geometries.ipynb)
* [Parsing `<link>`, `<joint>` and `<sensor>` elements](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/urdf_parser_links_joints_sensors.ipynb)
* [Parsing `<robot>` elements](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/urdf_parser_robots.ipynb)
* [Parsing `<visual>` elements](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/urdf_parser_visual.ipynb)

## Occupancy grid generation

* [Generating an occupancy grid map from a generated world](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/gen_grid_map.ipynb)

## Task manager

* [Using the Gazebo proxy object](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/tm_gazebo_proxy.ipynb)
* [Showcasing the process manager for simulation tasks](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/tm_process_manager.ipynb)
* [Showcasing the setup of conditional stages for sequences of tasks](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/tm_stage_conditions.ipynb)