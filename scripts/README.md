Executable scripts installed with the `pcg_gazebo` package for command line actions.

# `generate_occupancy_map`

Generates an occupancy grid map file in `.pgm` format from a `.world` file.
The world can be provided either as a ROS topic or a file.
Run `generate_occupancy_map -h` for more information.

# `generate_pcg_world`

Generates a Gazebo world file from YAML configuration describing the rules for placement of objects, assets to be used and spatial constraints.

> Examples

* [`launch_pcg_world` script](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/launch_pcg_world.sh)
* [Samples of world configuration files](https://github.com/boschresearch/pcg_gazebo/tree/master/examples/world_generator/worlds)

# `list_gazebo_models`

Lists all the static Gazebo models found in the ROS paths and `$HOME/.gazebo/models`.

> Example 

```
$ list_gazebo_models
```

# `process_jinja_template`

Generates a file from a Jinja template. 
Run `process_jinja_template -h` for more information.

> Examples

* [`gen_model_from_template` script](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/gen_model_from_template.sh)
* [Samples of model templates written in Jinja](https://github.com/boschresearch/pcg_gazebo/tree/master/examples/templates/models)
  
# `run_model_factory`

Runs a model factory function from the [creators](https://github.com/boschresearch/pcg_gazebo/blob/master/pcg_gazebo/generators/creators.py) module using a YAML file input with the model description.
Run `run_model_factory -h` for more information.

> Examples

* [`gen_model_from_factory_config` script](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/gen_model_from_factory_config.sh)
* [Samples of model factory configuration files](https://github.com/boschresearch/pcg_gazebo/tree/master/examples/model_factory)

# `sdf2urdf`

Converts a SDF file into an URDF file.
Run `sdf2urdf -h` for more information.

# `sdflint`

Checks a SDF file for errors.
Run `sdflint -h` for more information.

# `spawn_sdf_model`

Spawns a model described in SDF format in the current running instance of Gazebo.
The input SDF data can be provided via ROS parameter and the simulation can be
set to unpause after the model has been spawned.
This script starts a ROS node and therefore `roscore` must be already running.
Run `spawn_sdf_model -h` for more information.

# `start_gazebo_world`

Starts a Gazebo simulation from a `.world` file.
The `.world` file can be either provided as a file or through a ROS topic.
This script can be used start a world only when description is available via topic.
Run `start_gazebo_world -h` for more information.

# `urdf2sdf`

Converts an URDF file into a SDF file.
Run `urdf2sdf -h` for more information.

# `urdflint`

Checks an URDF file for errors. If a XACRO file is provided, it will be processed using the default `xacro` parser and the resulting URDF will be checked for errors.
Run `urdflint -h` for more information.

# `view_gazebo_model`

Opens a static Gazebo model and displays its geometries.
To see the tags of all Gazebo models available in your system, run `list_gazebo_models`.

> Examples

```
$ view_gazebo_model --model MODEL_NAME # To display the visual meshes
$ view_gazebo_model --model MODEL_NAME --collision # To display the collision meshes
```

# `view_mesh`

Opens a mesh file and displays it.
Run `view_mesh -h` for more information.