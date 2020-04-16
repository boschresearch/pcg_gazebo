Executable scripts installed with the `pcg_gazebo` package for command line actions.

# `pcg-generate-occupancy-map`

Generates an occupancy grid map file in `.pgm` format from a `.world` file.
The world can be provided either as a ROS topic or a file.
Run `generate_occupancy_map -h` for more information.

```bash
pcg-generate-occupancy-map -h
usage: Generate occupancy grid map from a SDF world file or the current scenario in Gazebo
       [-h] [--world-file WORLD_FILE] [--input-topic INPUT_TOPIC] [--xml XML]
       [--from-simulation] [--z-levels Z_LEVELS [Z_LEVELS ...]]
       [--min-z MIN_Z] [--max-z MAX_Z] [--without-ground-plane]
       [--occupied-color OCCUPIED_COLOR] [--free-color FREE_COLOR]
       [--unavailable-color UNAVAILABLE_COLOR] [--output-dir OUTPUT_DIR]
       [--output-filename OUTPUT_FILENAME] [--static-models-only] [--dpi DPI]
       [--figure-width FIGURE_WIDTH] [--figure-height FIGURE_HEIGHT]
       [--figure-size-unit FIGURE_SIZE_UNIT]
       [--exclude-contains EXCLUDE_CONTAINS [EXCLUDE_CONTAINS ...]]
       [--ground-plane-models GROUND_PLANE_MODELS [GROUND_PLANE_MODELS ...]]
       [--map-x-limits MAP_X_LIMITS [MAP_X_LIMITS ...]]
       [--map-y-limits MAP_Y_LIMITS [MAP_Y_LIMITS ...]] [--use-visual]

optional arguments:
  -h, --help            show this help message and exit
  --world-file WORLD_FILE, -w WORLD_FILE
                        SDF world filename
  --input-topic INPUT_TOPIC
                        Receive world XML file per ROS topic
  --xml XML             Receive world XML as string
  --from-simulation, -s
                        Retrieve world description from the current Gazebo
                        simulation
  --z-levels Z_LEVELS [Z_LEVELS ...], -l Z_LEVELS [Z_LEVELS ...]
                        Z levels to compute the grid map from
  --min-z MIN_Z         Minimum height for the Z rays in the ray tracing grid
  --max-z MAX_Z         Maximum height for the Z rays in the ray tracing grid
  --without-ground-plane
                        Ignore ground plane meshes from the map
  --occupied-color OCCUPIED_COLOR
                        Gray-scale color of the occupied cells
  --free-color FREE_COLOR
                        Gray-scale color of the free cells
  --unavailable-color UNAVAILABLE_COLOR
                        Gray-scale color of the unavailable cells
  --output-dir OUTPUT_DIR
                        Output directory to store the map
  --output-filename OUTPUT_FILENAME
                        Name of the output map file
  --static-models-only  Uses only static models for the map construction
  --dpi DPI             Figure DPI
  --figure-width FIGURE_WIDTH
                        Width of the figure
  --figure-height FIGURE_HEIGHT
                        Height of the figure
  --figure-size-unit FIGURE_SIZE_UNIT
                        Figure size unit [cm, m or inch]
  --exclude-contains EXCLUDE_CONTAINS [EXCLUDE_CONTAINS ...]
                        List of keywords for model names to be excluded from
                        the map
  --ground-plane-models GROUND_PLANE_MODELS [GROUND_PLANE_MODELS ...]
                        List of models that will be considered ground plane
  --map-x-limits MAP_X_LIMITS [MAP_X_LIMITS ...]
                        X limits of the output map in meters
  --map-y-limits MAP_Y_LIMITS [MAP_Y_LIMITS ...]
                        Y limits of the output map in meters
  --use-visual          Use visual meshes instead of collision
```

# `pcg-generate-sample-world-with-walls`

Generates a single room with walls with the option to add mesh primitives (e.g. cuboids, cylinders and spheres) to populate it.
The wall Gazebo model is per default stored in the local `$HOME/.gazebo/models` and the world file in `$HOME/.gazebo/worlds`, but both destinations can be configured.

```bash
pcg-generate-sample-world-with-walls -h
usage: pcg-generate-sample-world-with-walls [-h] [--n-rectangles N_RECTANGLES]
                                            [--n-points N_POINTS]
                                            [--wall-thickness WALL_THICKNESS]
                                            [--wall-height WALL_HEIGHT]
                                            [--n-cubes N_CUBES]
                                            [--n-cylinders N_CYLINDERS]
                                            [--n-spheres N_SPHERES]
                                            [--set-random-roll]
                                            [--set-random-pitch]
                                            [--x-room-range X_ROOM_RANGE]
                                            [--y-room-range Y_ROOM_RANGE]
                                            [--world-name WORLD_NAME]
                                            [--export-world-dir EXPORT_WORLD_DIR]
                                            [--export-models-dir EXPORT_MODELS_DIR]
                                            [--preview]

Generates a sample world with walls and objects as primitives

optional arguments:
  -h, --help            show this help message and exit
  --n-rectangles N_RECTANGLES, -r N_RECTANGLES
                        Number of rectangles to merge to generate the room
  --n-points N_POINTS, -p N_POINTS
                        Number of points to triangulate to generate the room
  --wall-thickness WALL_THICKNESS, -t WALL_THICKNESS
                        Thickness of the walls
  --wall-height WALL_HEIGHT, -g WALL_HEIGHT
                        Height of the walls
  --n-cubes N_CUBES, -c N_CUBES
                        Number of cubes to place in the world
  --n-cylinders N_CYLINDERS, -l N_CYLINDERS
                        Number of cylinders to place in the world
  --n-spheres N_SPHERES, -s N_SPHERES
                        Number of spheres to place in the world
  --set-random-roll     Set the roll angle of the placed objects with a random
                        variable
  --set-random-pitch    Set the pitch angle of the placed objects with a
                        random variable
  --x-room-range X_ROOM_RANGE, -x X_ROOM_RANGE
                        Range in X to generate the rectangles or points
  --y-room-range Y_ROOM_RANGE, -y Y_ROOM_RANGE
                        Range in Y to generate the rectangles or points
  --world-name WORLD_NAME, -n WORLD_NAME
                        Name of output world
  --export-world-dir EXPORT_WORLD_DIR
                        Export the world
  --export-models-dir EXPORT_MODELS_DIR
                        Export the models generated
  --preview             Show 3D preview of the created world

Usage:
    pcg-generate-sample-world-with-walls --n-rectangles 1
    pcg-generate-sample-world-with-walls --n-rectangles 10
    pcg-generate-sample-world-with-walls --n-points 20
    pcg-generate-sample-world-with-walls --n-rectangles 10 --n-cubes 5 --n-spheres 5 --n-cylinders 5
    pcg-generate-sample-world-with-walls --n-rectangles 10 --n-cubes 5 --n-spheres 5 --n-cylinders 5 --set-random-roll --set-random-pitch
    pcg-generate-sample-world-with-walls --n-points 20 --n-cubes 10 --preview
```

# `pcg-generate-world`

Generates a Gazebo world file from YAML configuration describing the rules for placement of objects, assets to be used and spatial constraints.

```bash
pcg-generate-world -h
usage: Generate the world using the PCG model placement engines
       [-h] [--config-file CONFIG_FILE]
       [--output-world-file OUTPUT_WORLD_FILE] [--verbose] [--plot]
       [--plot-width PLOT_WIDTH] [--plot-height PLOT_HEIGHT]
       [--output-topic OUTPUT_TOPIC] [--with-sun] [--with-ground-plane]
       [--run] [--physics PHYSICS]

optional arguments:
  -h, --help            show this help message and exit
  --config-file CONFIG_FILE
                        Configuration file (YAML format) with the PCG engines
                        specification and assets lists
  --output-world-file OUTPUT_WORLD_FILE
                        Output SDF world file
  --verbose             Set the output of the world generator as verbose
  --plot                Create bokeh plot of the object and workspace
                        footprints
  --plot-width PLOT_WIDTH
                        Width of the plot
  --plot-height PLOT_HEIGHT
                        Height of the plot
  --output-topic OUTPUT_TOPIC
                        Optional output topic to publish the resulting XML
  --with-sun            Add default sun model to world
  --with-ground-plane   Add default ground_plane model to world
  --run                 Run Gazebo with the generated world
  --physics PHYSICS     Physics engine to start with Gazebo
```

> Examples

* [`launch_pcg_world` script](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/launch_pcg_world.sh)
* [Samples of world configuration files](https://github.com/boschresearch/pcg_gazebo/tree/master/examples/world_generator/worlds)

# `pcg-inspect-asset`

```bash
pcg-inspect-asset -h
usage: List all elements from a model or world [-h] [--filename FILENAME]
                                               [--gazebo-model GAZEBO_MODEL]
                                               [--print-xml]

optional arguments:
  -h, --help            show this help message and exit
  --filename FILENAME, -f FILENAME
  --gazebo-model GAZEBO_MODEL, -g GAZEBO_MODEL
  --print-xml, -p
```

# `pcg-install-gazebo-assets`

```bash
pcg-install-gazebo-assets -h
usage: pcg-install-gazebo-assets [-h] [--tarball TARBALL] [--dir DIR]
                                 [--filename FILENAME]
                                 [--models-path MODELS_PATH] [--add-timestamp]
                                 [--prefix PREFIX] [--name NAME]

Open and install Gazebo assets (media, models and world) in the default Gazebo resource paths <$HOME/.gazebo>

optional arguments:
  -h, --help            show this help message and exit
  --tarball TARBALL, -t TARBALL
                        Filename of the tarball containing the simulation
                        assets. The tarball must contain either (1) a Gazebo
                        model, including at least model.config and model.sdf
                        files or (2) a models and/or a worlds folder
  --dir DIR, -d DIR     A folder containing (1) a Gazebo model, including at
                        least model.config and model.sdf files or (2) a models
                        and/or a worlds folder
  --filename FILENAME, -f FILENAME
                        A SDF, URDF or XACRO file to be installed as a Gazebo
                        model or world
  --models-path MODELS_PATH, -m MODELS_PATH
                        Optional custom models path necessary for parsing the
                        asset to be installed
  --add-timestamp, -a   Adds timestamp to the asset name as a suffix
  --prefix PREFIX, -p PREFIX
                        Prefix string to be added to the asset name installed
  --name NAME, -n NAME  New name of the asset to be installed

Usage:
        pcg-install-gazebo-assets --tarball FILENAME
        pcg-install-gazebo-assets --filename FILENAME
        pcg-install-gazebo-assets --dir FOLDER --prefix PREFIX --add-timestamp --name NEW_ASSET_NAME
        pcg-install-gazebo-assets --tarball FILENAME --prefix PREFIX --add-timestamp
```

# `pcg-list-gazebo-models`

Lists all the static Gazebo models found in the ROS paths and `$HOME/.gazebo/models`.

> Example 

```
$ pcg-list-gazebo-models
```

# `pcg-populate-world`

```bash
pcg-populate-world -h
usage: pcg-populate-world [-h] [--world WORLD] [--config CONFIG]
                          [--models MODEL [MODEL ...]] [--num NUM [NUM ...]]
                          [--min-distance MIN_DISTANCE [MIN_DISTANCE ...]]
                          [--static STATIC [STATIC ...]]
                          [--export-filename EXPORT_FILENAME]
                          [--custom-models-path CUSTOM_MODELS_PATH [CUSTOM_MODELS_PATH ...]]
                          [--random-roll] [--random-pitch] [--random-yaw]
                          [--workspace WORKSPACE] [--tangent-to-ground]
                          [--preview]

This script will either take a Gazebo world (or use an empty one if none is provided) and populate it w
ith models placed within a workspace.The workspace can be either provided as a set of 2D or 3D points, 
the convex hull of the models in the world, or the free space found in the world grid map.

optional arguments:
  -h, --help            show this help message and exit
  --world WORLD, -w WORLD
                        Gazebo world as file or XML input to be populated
  --config CONFIG, -c CONFIG
                        YAML file with the configuration for engines and
                        models to be added to the world
  --models MODEL [MODEL ...], -m MODEL [MODEL ...]
                        Name of Gazebo model to include in the world
  --num NUM [NUM ...], -n NUM [NUM ...]
                        Number of elements to add to world (use -1 if no
                        maximum limit is necessary)
  --min-distance MIN_DISTANCE [MIN_DISTANCE ...], -d MIN_DISTANCE [MIN_DISTANCE ...]
                        Minimum distance between objects (either for all
                        models or per model
  --static STATIC [STATIC ...], -s STATIC [STATIC ...]
                        Whether the model should be static or not
  --export-filename EXPORT_FILENAME, -f EXPORT_FILENAME
                        Name of the filename to export the generated world. If
                        none is provided, the same filename with a datetime
                        suffix will be used.
  --custom-models-path CUSTOM_MODELS_PATH [CUSTOM_MODELS_PATH ...], -p CUSTOM_MODELS_PATH [CUSTOM_MODELS_PATH ...]
                        Custom folder containing Gazebo models
  --random-roll         For random engines, the roll angle will also be
                        randomized
  --random-pitch        For random engines, the pitch angle will also be
                        randomized
  --random-yaw          For random engines, the yaw angle will also be
                        randomized
  --workspace WORKSPACE, -k WORKSPACE
                        Name of model or set of coordinates provided as (1)
                        [(x, y, z), (x, y, z), ...] for a 3D workpace, (2)
                        [(x, y), (x, y), ...] for a 2D workspace, (3) name of
                        models in the provided world that delimit the
                        workspace (e.g. walls). (4) "gridmap" to compute a
                        grid map of the input world and place objects in the
                        free space foundIf no workspace is defined and there
                        is a world with models provided, the workspace will be
                        the convex hull around all existing models. If no
                        world is provided, an area of 20 x 20 m on the ground
                        plane will be set as the workspace
  --tangent-to-ground, -t
                        Place all models tangent to the ground plane
  --preview             Preview the workspace and generated world

    pcg-populate-world --model MODEL_1 MODEL_2 --num 3 3 --min-distance 0.2 --static 1  # Models 1 and 2 must be Gazebo models
    pcg-populate-world --model random_box --num 2  # Add two random boxes to an empty world
    pcg-populate-world --model random_box random_cylinder random_sphere --num -1 -1 -1 --tangent-to-ground  # Add as many random boxes, cylinders and spheres as possible in the default workspace and place them tangent to the ground plane
    pcg-populate-world --model MESH_FILENAME --num 1  # Add mesh as model in the world
    pcg-populate-world --world WORLD_FILE --model MODEL_1 --num 5  # Add MODEL_1 to world loaded from file
```

# `pcg-preview-sdf`

Opens a SDF file (either `.sdf` or `.world`) and shows a 3D preview of the geometries and meshes.
The input SDF file must contain either a world or a model.
Beware that if the world or model includes another model or meshes using the prefixes `model://`, that
the included models must be in the Gazebo resources path so that they can be also parsed.

```bash
pcg-preview-sdf -h
usage: Parse a SDF file (.sdf or .world) and display its geometries and meshes
       [-h] [--filename FILENAME] [--collision]

optional arguments:
  -h, --help            show this help message and exit
  --filename FILENAME, -f FILENAME
                        SDF file (.sdf or .world)
  --collision, -c       View collision meshes
```

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

```bash
pcg-preview-urdf -h
usage: Parse a URDF file and display its geometries and meshes. XACRO files can also be provided and will processed to generate the URDF files.
       [-h] [--filename FILENAME] [--collision]

optional arguments:
  -h, --help            show this help message and exit
  --filename FILENAME, -f FILENAME
                        URDF or XACRO file
  --collision, -c       View collision meshes
``` 

> Example

```bash
pcg-preview-urdf --filename URDF_OR_XACRO_FILENAME              # Preview of the visual meshes
pcg-preview-urdf --filename URDF_OR_XACRO_FILENAME --collision  # Preview of the collision meshes
```

# `pcg-print-xml-element`

This tool can be called to retrieve information on SDF, URDF or SDF Config elements.
They can be displayed in XML format or the script can list all the children and attributes of an XML element.

```bash
pcg-print-xml-element -h
usage: pcg-print-xml-element [-h] [--sdf] [--urdf] [--sdf-config] [--tag TAG]
                             [--list] [--xml] [--description]

Print XML elements (SDF, URDF or SDF Config)

optional arguments:
  -h, --help         show this help message and exit
  --sdf              Retrieve information on SDF element
  --urdf             Retrieve information on URDF element
  --sdf-config       Retrieve information on SDF Config element
  --tag TAG, -t TAG  Name of the XML element
  --list, -l         List tags of all elements available
  --xml, -x          Output the element as XML
  --description, -d  Print information on the children of the XML element

Usage:
    pcg-print-xml-element --sdf --list
    pcg-print-xml-element --urdf --list
    pcg-print-xml-element --sdf-config --list
    pcg-print-xml-element --sdf --tag NAME --xml
    pcg-print-xml-element --sdf --tag NAME --description
```

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

```bash
pcg-process-jinja-template -h
usage: pcg-process-jinja-template [-h] [--input-template INPUT_TEMPLATE]
                                  [--param-file PARAM_FILE]
                                  [--output-filename OUTPUT_FILENAME]
                                  [--include-dir INCLUDE_DIR] [--param PARAM]
                                  [--sdf] [--sdf-config] [--urdf]
                                  [--merge-nested-models]

Parse Jinja template

optional arguments:
  -h, --help            show this help message and exit
  --input-template INPUT_TEMPLATE, -i INPUT_TEMPLATE
                        Input template file
  --param-file PARAM_FILE, -f PARAM_FILE
                        YAML file with parameters to be rendered in the final
                        output
  --output-filename OUTPUT_FILENAME, -o OUTPUT_FILENAME
                        Output file to store the parsed file
  --include-dir INCLUDE_DIR, -d INCLUDE_DIR
                        Input directory for template modules
  --param PARAM, -p PARAM
                        Model parameter to be replaced in the template
  --sdf                 Output template is a SDF file and will be verified
  --sdf-config          Output template is a SDF model configuration file and
                        will be verified
  --urdf                Output template is an URDF file and will be verified
  --merge-nested-models
                        For SDF descriptions, merge nested models before
                        generating final SDF, in case there are any
```

> Examples

* [`gen_model_from_template` script](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/gen_model_from_template.sh)
* [Samples of model templates written in Jinja](https://github.com/boschresearch/pcg_gazebo/tree/master/examples/templates/models)
  
# `pcg-run-model-factory`

Runs a model factory function from the [creators](https://github.com/boschresearch/pcg_gazebo/blob/master/pcg_gazebo/generators/creators.py) module using a YAML file input with the model description.
Run `pcg-run-model-factory -h` for more information.

```bash
pcg-run-model-factory -h
usage: Imports a YAML file with the description of a model and/or description of the factory parameters to generate models procedurally
       [-h] [--config-file CONFIG_FILE] [--config CONFIG] [--print]
       [--store-model] [--store-dir STORE_DIR] [--overwrite] [--spawn]
       [--spawn-random-positions]

optional arguments:
  -h, --help            show this help message and exit
  --config-file CONFIG_FILE, -f CONFIG_FILE
                        YAML file with the model generator configuration
  --config CONFIG, -c CONFIG
                        Enter the configuration in the JSON format as a string
  --print               Print models as SDF
  --store-model         Store the generated models as Gazebo models
  --store-dir STORE_DIR
                        Output directory to store the generated models
  --overwrite           Overwrite models with the same name when storing
  --spawn               Spawn the generated models if Gazebo is running
  --spawn-random-positions
                        Spawn models on random positions
```

> Examples

* [`gen_model_from_factory_config` script](https://github.com/boschresearch/pcg_gazebo/blob/master/examples/gen_model_from_factory_config.sh)
* [Samples of model factory configuration files](https://github.com/boschresearch/pcg_gazebo/tree/master/examples/model_factory)

# `pcg-sdf2urdf`

Converts a SDF file into an URDF file.
Run `pcg-sdf2urdf -h` for more information.

```bash
pcg-sdf2urdf -h
usage: pcg-sdf2urdf [-h] [--param PARAM] [--filename FILENAME] [--xml XML]
                    [--input-topic INPUT_TOPIC]
                    [--output-filename OUTPUT_FILENAME]
                    [--output-parameter OUTPUT_PARAMETER] [--print]
                    [--verbose]

Convert URDF to SDF file

optional arguments:
  -h, --help            show this help message and exit
  --param PARAM, -p PARAM
                        ROS parameter where the SDF robot description is
                        stored
  --filename FILENAME, -f FILENAME
                        Filename name to the SDF robot description
  --xml XML, -x XML     XML text input with the SDF robot description
  --input-topic INPUT_TOPIC, -t INPUT_TOPIC
                        ROS topic that will deliver the XML text input with
                        the SDF robot description
  --output-filename OUTPUT_FILENAME, -o OUTPUT_FILENAME
                        Output file to store the converted URDF file
  --output-parameter OUTPUT_PARAMETER, -r OUTPUT_PARAMETER
                        Output ROS parameter to store the converted URDF file
  --print               Print the file
  --verbose, -v         Run on verbose mode
```

# `pcg-sdflint`

Checks a SDF file for errors.
Run `pcg-sdflint -h` for more information.

```bash
pcg-sdflint -h
usage: pcg-sdflint [-h] [--param PARAM] [--filename FILENAME] [--xml XML]
                   [--print] [--verbose]

Linter for SDF file

optional arguments:
  -h, --help            show this help message and exit
  --param PARAM, -p PARAM
                        ROS parameter where the SDF robot description is
                        stored
  --filename FILENAME, -f FILENAME
                        Filename name to the SDF robot description or XACRO
                        filename to generate it
  --xml XML, -x XML     XML text input with the SDF robot description
  --print               Print the file
  --verbose, -v         Run on verbose mode
```

# `pcg-spawn-sdf-model`

Spawns a model described in SDF format in the current running instance of Gazebo.
The input SDF data can be provided via ROS parameter and the simulation can be
set to unpause after the model has been spawned.
This script starts a ROS node and therefore `roscore` must be already running.
Run `pcg-spawn-sdf-model -h` for more information.

```bash
pcg-spawn-sdf-model -h
usage: pcg-spawn-sdf-model [-h] [--param_name PARAM_NAME]
                           [--robot_name ROBOT_NAME] [--x X] [--y Y] [--z Z]
                           [--roll ROLL] [--pitch PITCH] [--yaw YAW]
                           [--unpause-simulation]

Spawn SDF model after trigger

optional arguments:
  -h, --help            show this help message and exit
  --param_name PARAM_NAME
                        Input parameter name where the SDF content is stored
  --robot_name ROBOT_NAME
                        Name of the robot model
  --x X                 X coordinate of the spawning position in meters
  --y Y                 Y coordinate of the spawning position in meters
  --z Z                 Z coordinate of the spawning position in meters
  --roll ROLL           Roll angle of the spawning position in radians
  --pitch PITCH         Pitch angle of the spawning position in radians
  --yaw YAW             Yaw angle of the spawning position in radians
  --unpause-simulation  Unpause simulation once the model has been spawned
```

# `pcg-start-gazebo-world`

Starts a Gazebo simulation from a `.world` file.
The `.world` file can be either provided as a file or through a ROS topic.
This script can be used start a world only when description is available via topic.
Run `pcg-start-gazebo-world -h` for more information.

```bash
pcg-start-gazebo-world -h
usage: pcg-start-gazebo-world [-h] [--input_topic INPUT_TOPIC]
                              [--input_world_filename INPUT_WORLD_FILENAME]
                              [--physics PHYSICS] [--paused PAUSED]
                              [--gui GUI]

Start Gazebo world from file

optional arguments:
  -h, --help            show this help message and exit
  --input_topic INPUT_TOPIC, -t INPUT_TOPIC
                        ROS topic that will deliver the XML text input with
                        the SDF robot description
  --input_world_filename INPUT_WORLD_FILENAME, -f INPUT_WORLD_FILENAME
                        Input
  --physics PHYSICS     Name of the physics engine
  --paused PAUSED       Start simulation paused
  --gui GUI             Do not start Gazebo client
```

# `pcg-urdf2sdf`

Converts an URDF file into a SDF file.
Run `pcg-urdf2sdf -h` for more information.

```bash
pcg-urdf2sdf -h
usage: pcg-urdf2sdf [-h] [--param PARAM] [--filename FILENAME] [--xml XML]
                    [--input-topic INPUT_TOPIC]
                    [--output-filename OUTPUT_FILENAME]
                    [--output-parameter OUTPUT_PARAMETER]
                    [--output-gazebo-model-path OUTPUT_GAZEBO_MODEL_PATH]
                    [--create-gazebo-model] [--model-name MODEL_NAME]
                    [--print] [--sdf-version SDF_VERSION] [--verbose]

Convert SDF to URDF file

optional arguments:
  -h, --help            show this help message and exit
  --param PARAM, -p PARAM
                        ROS parameter where the URDF robot description is
                        stored
  --filename FILENAME, -f FILENAME
                        Filename name to the URDF robot description
  --xml XML, -x XML     XML text input with the URDF robot description
  --input-topic INPUT_TOPIC, -t INPUT_TOPIC
                        ROS topic that will deliver the XML text input with
                        the URDF robot description
  --output-filename OUTPUT_FILENAME, -o OUTPUT_FILENAME
                        Output file to store the converted SDF file
  --output-parameter OUTPUT_PARAMETER, -r OUTPUT_PARAMETER
                        Output ROS parameter to store the converted SDF file
  --output-gazebo-model-path OUTPUT_GAZEBO_MODEL_PATH, -gp OUTPUT_GAZEBO_MODEL_PATH
                        Output path for the Gazebo model
  --create-gazebo-model, -g
                        Export SDF as a static Gazebo model
  --model-name MODEL_NAME, -m MODEL_NAME
                        Name of the model being generated
  --print               Print the file
  --sdf-version SDF_VERSION
                        Version of the SDF file being generated
  --verbose, -v         Run on verbose mode
```

# `pcg-urdflint`

Checks an URDF file for errors. If a XACRO file is provided, it will be processed using the default `xacro` parser and the resulting URDF will be checked for errors.
Run `pcg-urdflint -h` for more information.

```bash
pcg-urdflint -h
usage: pcg-urdflint [-h] [--param PARAM] [--filename FILENAME] [--xml XML]
                    [--print] [--verbose]

Convert URDF to URDF file

optional arguments:
  -h, --help            show this help message and exit
  --param PARAM, -p PARAM
                        ROS parameter where the URDF robot description is
                        stored
  --filename FILENAME, -f FILENAME
                        Filename name to the URDF robot description or XACRO
                        file to generate it
  --xml XML, -x XML     XML text input with the URDF robot description
  --print               Print the file
  --verbose, -v         Run on verbose mode
```

# `pcg-view-gazebo-model`

Opens a static Gazebo model and displays its geometries.
To see the tags of all Gazebo models available in your system, run `pcg-list-gazebo-models`.

```bash
pcg-view-gazebo-model -h
usage: Open and display a Gazebo model [-h] [--model MODEL] [--collision]

optional arguments:
  -h, --help            show this help message and exit
  --model MODEL, -m MODEL
                        Gazebo model name
  --collision, -c       View collision meshes
```

> Examples

```
$ pcg-view-gazebo-model --model MODEL_NAME # To display the visual meshes
$ pcg-view-gazebo-model --model MODEL_NAME --collision # To display the collision meshes
```

# `pcg-view-mesh`

Opens a mesh file and displays it.
Run `pcg-view-mesh -h` for more information.

```bash
pcg-view-mesh -h
usage: Open and display a mesh [-h] [--filename FILENAME]

optional arguments:
  -h, --help            show this help message and exit
  --filename FILENAME, -f FILENAME
                        Mesh filename
```