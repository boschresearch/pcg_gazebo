Executable scripts installed with the `pcg_gazebo` package for command line actions.

# `pcg-generate-occupancy-map`

Generates an occupancy grid map file in `.pgm` format from a `.world` file.
The world can be provided either as a ROS topic or a file.
Run `generate_occupancy_map -h` for more information.

# `pcg-generate-world`

Generates a Gazebo world file from YAML configuration describing the rules for placement of objects, assets to be used and spatial constraints.

> Examples

* [`launch_pcg_world` script](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/launch_pcg_world.sh)
* [Samples of world configuration files](https://github.com/boschresearch/pcg_gazebo/tree/master/examples/world_generator/worlds)

# `pcg-list-gazebo-models`

Lists all the static Gazebo models found in the ROS paths and `$HOME/.gazebo/models`.

> Example 

```
$ pcg-list-gazebo-models
```

# `pcg-preview-sdf`

Opens a SDF file (either `.sdf` or `.world`) and shows a 3D preview of the geometries and meshes.
The input SDF file must contain either a world or a model.
Beware that if the world or model includes another model or meshes using the prefixes `model://`, that
the included models must be in the Gazebo resources path so that they can be also parsed.

> Example

```bash
pcg-preview-sdf --filename SDF_FILENAME              # Preview of the visual meshes
pcg-preview-sdf --filename SDF_FILENAME --collision  # Preview of the collision meshes
```

# `pcg-preview-urdf`

Opens an URDF file (`.urdf`) or a XACRO file (`.xacro`) that will be parsed into an URDF file, 
and shows a 3D preview of the geometries and meshes.
Beware that if the model includes meshes and includes other XACRO files, they should also
be reachable within the ROS paths so that the model can be parsed.

> Example

```bash
pcg-preview-urdf --filename URDF_OR_XACRO_FILENAME              # Preview of the visual meshes
pcg-preview-urdf --filename URDF_OR_XACRO_FILENAME --collision  # Preview of the collision meshes
```

# `pcg-print-xml-element`

This tool can be called to retrieve information on SDF, URDF or SDF Config elements.
They can be displayed in XML format or the script can list all the children and attributes of an XML element.

> Example 

```bash
pcg-print-xml-element --sdf --list
pcg-print-xml-element --urdf --list
pcg-print-xml-element --sdf-config --list
pcg-print-xml-element --sdf --tag NAME --xml
pcg-print-xml-element --sdf --tag NAME --description
```

# `pcg-process-jinja-template`

Generates a file from a Jinja template. 
Run `pcg-process-jinja-template -h` for more information.

> Examples

* [`gen_model_from_template` script](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/gen_model_from_template.sh)
* [Samples of model templates written in Jinja](https://github.com/boschresearch/pcg_gazebo/tree/master/examples/templates/models)
  
# `pcg-run-model-factory`

Runs a model factory function from the [creators](https://github.com/boschresearch/pcg_gazebo/blob/master/pcg_gazebo/generators/creators.py) module using a YAML file input with the model description.
Run `pcg-run-model-factory -h` for more information.

> Examples

* [`gen_model_from_factory_config` script](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/gen_model_from_factory_config.sh)
* [Samples of model factory configuration files](https://github.com/boschresearch/pcg_gazebo/tree/master/examples/model_factory)

# `pcg-sdf2urdf`

Converts a SDF file into an URDF file.
Run `pcg-sdf2urdf -h` for more information.

# `pcg-sdflint`

Checks a SDF file for errors.
Run `pcg-sdflint -h` for more information.

# `pcg-spawn-sdf-model`

Spawns a model described in SDF format in the current running instance of Gazebo.
The input SDF data can be provided via ROS parameter and the simulation can be
set to unpause after the model has been spawned.
This script starts a ROS node and therefore `roscore` must be already running.
Run `pcg-spawn-sdf-model -h` for more information.

# `pcg-start-gazebo-world`

Starts a Gazebo simulation from a `.world` file.
The `.world` file can be either provided as a file or through a ROS topic.
This script can be used start a world only when description is available via topic.
Run `pcg-start-gazebo-world -h` for more information.

# `pcg-urdf2sdf`

Converts an URDF file into a SDF file.
Run `pcg-urdf2sdf -h` for more information.

# `pcg-urdflint`

Checks an URDF file for errors. If a XACRO file is provided, it will be processed using the default `xacro` parser and the resulting URDF will be checked for errors.
Run `pcg-urdflint -h` for more information.

# `pcg-view-gazebo-model`

Opens a static Gazebo model and displays its geometries.
To see the tags of all Gazebo models available in your system, run `pcg-list-gazebo-models`.

> Examples

```
$ pcg-view-gazebo-model --model MODEL_NAME # To display the visual meshes
$ pcg-view-gazebo-model --model MODEL_NAME --collision # To display the collision meshes
```

# `pcg-view-mesh`

Opens a mesh file and displays it.
Run `pcg-view-mesh -h` for more information.
