# Single room generation

For some applications, generating a sample simple Gazebo scenario with walls and some objects is an interesting feature.
It can be used to test navigation algorithms in different worlds generated via scripts.

## Using the script

After installing the package, you can use the `pcg-generate-sample-world-with-walls` script to generate a simple room with some model primitives (e.g. cuboids, cylinders and spheres) randomly placed in it.

The room can be generated out of a single rectangle, run

```bash
pcg-generate-sample-world-with-walls --n-rectangles 1 --preview
```

> The `--preview` option shows a 3D preview of the generated models. Remove it to avoid opening the `pyglet` window.

Multiple rectangles can also be merged to vary the overall shape of the room. 
To generate the room from multiple rectangles, run

```bash
pcg-generate-sample-world-with-walls --n-rectangles 10 --preview
```

> Per default the `.world` output is stored in `$HOME/.gazebo/worlds` and the wall model generated is stored in `$HOME/.gazebo/models`. Use the inputs `--export-world-dir` and `--export-models-dir`. Be aware that when running the world in Gazebo, the generated model must be reachable via `GAZEBO_MODEL_PATH`. 

> The world name is per default named `pcg_sample.world` and the wall model `pcg_sample_walls`. To change the world name and prefix of the wall name use the input `--world-name`.

Another option is to generate a room out of the triangulation of randomily generated points

```bash
pcg-generate-sample-world-with-walls --n-points 10 --preview
```

### Change the size of the room

```bash
pcg-generate-sample-world-with-walls --n-rectangles 3 --x-room-range 10 --y-room-range 10 --preview
```

```bash
pcg-generate-sample-world-with-walls --n-rectangles 3 --x-room-range 50 --y-room-range 50 --preview
```

### Include basic models to the generated world

It's possible to already include model primitives in within the walls by providing the number of cubes, cylinders and spheres.
The models will be included in the output world file. 
Use the following example to place these objects in a sample room

```bash
pcg-generate-sample-world-with-walls --n-rectangles 3 --n-cubes 4 --n-cylinders 4 --n-spheres 4 --preview
```

> There is a limit on the amount of models Gazebo can load. If models are missing from the originally generated world file, try regenerating it with less primitives.

Other examples can be seen below 

```bash
pcg-generate-sample-world-with-walls --n-rectangles 10 --n-cubes 10 --preview
pcg-generate-sample-world-with-walls --n-rectangles 10 --n-cubes 10 --n-spheres 2 --preview
pcg-generate-sample-world-with-walls --n-rectangles 10 --n-cylinders 10 --preview
pcg-generate-sample-world-with-walls --n-rectangles 10 --n-spheres 10 --preview
```

You can also vary not only the `(x, y, yaw)` pose of the models and set a random roll and pitch as well by using

```bash
pcg-generate-sample-world-with-walls --n-rectangles 3 --n-cubes 5 --n-cylinders 5 --n-spheres 5 --set-random-roll --set-random-pitch --preview
```

### Running the world in Gazebo

As stated before, the default world is stored into `$HOME/.gazebo/worlds`.
The default world is named `pcg_sample.world` and can be changed by using the script's `--world-name`.
When using the default world name, you can run the world with

```bash
gazebo $HOME/.gazebo/worlds/pcg_sample.world
```

```bash
roslaunch gazebo_ros empty_world.launch world_name:=$HOME/.gazebo/worlds/pcg_sample.world
```

```bash
ros2 launch gazebo_ros gazebo.launch.py world:=$HOME/.gazebo/worlds/pcg_sample.world
```
