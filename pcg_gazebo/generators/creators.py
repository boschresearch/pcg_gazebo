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
"""Factory methods to create simulation models."""
import numpy as np
import collections
import itertools
from .components import HingedDoor
from ..log import PCG_ROOT_LOGGER
from ..utils import is_string
from ..simulation import SimulationModel
from multiprocessing.pool import Pool


def _parse_factory_input_as_vector(var):
    """Parse a input argument for the factory methods and return
    its elements as a vector.

    > *Input arguments*

    * `var` (*type:* `float`, `int`, `str`, `unicode`,
    `list`, `numpy.ndarray`): Input variable

    > *Returns*

    List of variables as `list` or `numpy.array` if the inputs are numeric.
    `None` if the type of `var` is not supported.
    """
    if isinstance(
            var,
            float) or isinstance(
            var,
            int) or isinstance(
                var,
                np.int64) or isinstance(
                    var,
            np.float64):
        PCG_ROOT_LOGGER.info('Variable provided as scalar={}'.format(
            var))
        return np.array([var])

    if isinstance(var, collections.Iterable) and not is_string(var):
        PCG_ROOT_LOGGER.info('Variable provided as vector={}'.format(
            var))
        return np.array(var)
    elif is_string(var):
        PCG_ROOT_LOGGER.info('Variable provided as a inline command=' + var)
        try:
            vars = eval(var)
            PCG_ROOT_LOGGER.info(
                'Generated output, fcn={}, output={}'.format(
                    var, vars))
        except Exception as ex:
            PCG_ROOT_LOGGER.warning(
                'Could not evaluate variable lambda function, returning string'
                ', input={}, message={}'.format(var, str(ex)))
            return var

        if isinstance(vars, float) or isinstance(vars, int):
            PCG_ROOT_LOGGER.info('Variable provided as scalar={}'.format(
                var))
            return np.array([vars])
        elif not isinstance(vars, collections.Iterable):
            if callable(vars):
                vars = vars()
            else:
                PCG_ROOT_LOGGER.error(
                    'No value returned after evaluating'
                    ' lambda function, fcn={}'.format(var))
                return None
        else:
            return vars
        PCG_ROOT_LOGGER.info(
            'Variable provided as lambda function={}'.format(var))
    elif callable(var):
        return var()
    else:
        return None


def box(size, mass=0, name='box', pose=[0, 0, 0, 0, 0, 0], color=None,
        visual_parameters=dict(), collision_parameters=dict()):
    """Factory method that returns a box-shaped model with one cuboid link.

    > *Input arguments*

    * `size` (*type:* `list` or `numpy.ndarray`): 3D vector with the
    size of the box for width, length and height, respectively, in meters.
    * `mass` (*type:* `float`, *default:* `0`): Mass of the model. If the
    mass is not greater than zero, the model is set as static.
    * `name` (*type:* `str`, *default:* `'box'`): Name of the model.
    * `pose` (*type:* `list` or `numpy.array`, *default:*
    `[0, 0, 0, 0, 0, 0]`): Origin of the model.
    * `color` (*type:* `str` or `list`, *default:* `None`):
    Color of the model. It can be provided as a
    RGBA vector, `xkcd` for a random [XKCD color](https://xkcd.com/color/rgb/)
    or a specific `xkcd` color name, and/or `random` for a random RGBA color.

    > *Returns*

    A box-shaped `pcg_gazebo.simulation.SimulationModel` instance.
    """
    input_mass = float(_parse_factory_input_as_vector(mass)[0])
    input_size = _parse_factory_input_as_vector(size).tolist()
    input_pose = _parse_factory_input_as_vector(pose).tolist()

    input_col_params = collision_parameters.copy()
    for tag in collision_parameters:
        if tag == 'fdir1':
            input_col_params[tag] = _parse_factory_input_as_vector(
                collision_parameters[tag]).tolist()
        else:
            input_col_params[tag] = float(
                _parse_factory_input_as_vector(
                    collision_parameters[tag])[0])

    model = SimulationModel(name=name)
    model.add_cuboid_link(
        link_name='link',
        mass=input_mass,
        size=input_size,
        color=color,
        visual_parameters=visual_parameters,
        collision_parameters=input_col_params)

    if input_mass <= 0:
        model.static = True

    model.pose = input_pose
    return model


def mesh(visual_mesh, collision_mesh=None,
         use_approximated_collision=False, approximated_collision_model='box',
         visual_mesh_scale=[1, 1, 1], collision_mesh_scale=[1, 1, 1],
         name='mesh', pose=[0, 0, 0, 0, 0, 0], color=None, mass=0,
         inertia=None, use_approximated_inertia=True,
         approximated_inertia_model='box', visual_parameters=dict(),
         collision_parameters=dict()):
    """Create a model based on a mesh input. The options for visual and
    collision meshes are:

    * `visual_mesh` is provided and no `collision_mesh`.
    The collision mesh is then set to be the same as the visual mesh.
    * Both `visual_mesh` and `collision_mesh` are provided
    separately.
    * `visual_mesh` is provided and no `collision_mesh`, but
    `use_approximated_collision` is `True`. In this case the collision geometry
    can be an approximated geometry fitted to the visual mesh. Options for
    the approximation methods are `box`, `sphere` or `cylinder`.

    The same is valid for the moments of inertia. For static models, `mass`
    can be set as 0. Otherwise, the following options are possible:

    * `inertia` provided as `inertia=dict(ixx=0, iyy=0, izz=0, ixy=0, ixz=0,
    iyz=0)`
    * Set `use_approximated_inertia` to `True` and the inertia model will be
    computed for the model using the `approximated_inertia_model` input
    (options are `box`, `sphere` or `cylinder`). In this case the
    approximated geometry will be computed from the visual mesh and
    its dimensions combined with the provided `mass` will be used
    to generate the model's moments of inertia.

    > *Input arguments*

    * `visual_mesh` (*type:* `str` or `trimesh.Trimesh`): Name of the
    visual mesh file or mesh structure
    * `collision_mesh` (*type:* `str` or `trimesh.Trimesh`, *default:* `None`):
    Name of the collision mesh file. If `None` is provided, then
    the visual mesh file will be used as collision geometry
    * `use_approximated_collision` (*type:* `bool`, *default:* `False`): Enable
    computing an approximated collision geometry from the visual mesh.
    * `approximated_collision_model` (*type:* `str`, *default:* `'box'`): Type
    of approximated collision geometry to be derived from the visual
    mesh. Options are `box`, `cylinder` or `sphere`.
    * `visual_mesh_scale` (*type:* `list`, *default:* `[1, 1, 1]`):
    Scaling vector to be applied to the visual mesh
    * `collision_mesh_scale` (*type:* `list`, *default:*
    `[1, 1, 1]`): Scaling vector to be applied to the collision mesh
    * `name` (*type:* `str`, *default:* `'box'`): Name of the model.
    * `pose` (*type:* `list` or `numpy.array`, *default:*
    `[0, 0, 0, 0, 0, 0]`): Origin of the model.
    * `color` (*type:* `str` or `list`, *default:* `None`): Color of
    the model. It can be provided as a RGBA vector, `xkcd` for a random
    [XKCD color](https://xkcd.com/color/rgb/)
    or a specific `xkcd` color name, and/or `random` for a random RGBA color.
    * `mass` (*type:* `float`, *default:* `0`): Mass of the model. If the
    mass is not greater than zero, the model is set as static.
    * `inertia` (*type:* `dict`, *default:* `None`): Optional moments
    of inertia setting to the model in the form of
    `inertia=dict(ixx=0, iyy=0, izz=0, ixy=0, ixz=0, iyz=0)`
    * `use_approximated_inertia` (*type:* `bool`, *default:* `True`): Enable
    computation of moments of inertia based on the `mass` input and
    the approximated inertia model setting based on the dimensions
    of the mesh.
    * `approximated_inertia_model` (*type:* `str`, *default:* `box`): Type of
    geometrical approximation to be computed from the visual
    mesh. The dimensions of the approximated geometry will be
    then used to compute the moments of inertia of the model.
    Options are `box`, `cylinder` or `sphere`.

    > *Returns*

    A `pcg_gazebo.simulation.SimulationModel` instance.
    """

    input_mass = float(_parse_factory_input_as_vector(mass)[0])
    input_visual_mesh_scale = _parse_factory_input_as_vector(
        visual_mesh_scale).tolist()
    input_collision_mesh_scale = _parse_factory_input_as_vector(
        collision_mesh_scale).tolist()
    input_pose = _parse_factory_input_as_vector(pose).tolist()

    input_col_params = collision_parameters.copy()
    for tag in collision_parameters:
        if tag == 'fdir1':
            input_col_params[tag] = _parse_factory_input_as_vector(
                collision_parameters[tag]).tolist()
        else:
            input_col_params[tag] = float(
                _parse_factory_input_as_vector(
                    collision_parameters[tag])[0])

    model = SimulationModel(name=name)
    model.add_link(
        visual_mesh=visual_mesh,
        collision_mesh=collision_mesh,
        use_approximated_collision=use_approximated_collision,
        approximated_collision_model=approximated_collision_model,
        visual_mesh_scale=input_visual_mesh_scale,
        collision_mesh_scale=input_collision_mesh_scale,
        name=name,
        color=color,
        mass=input_mass,
        inertia=inertia,
        use_approximated_inertia=use_approximated_inertia,
        approximated_inertia_model=approximated_inertia_model)
    model.pose = input_pose

    if mass <= 0:
        model.static = True

    return model


def sphere(radius, mass=0, name='sphere', pose=[0, 0, 0, 0, 0, 0],
           color=None, visual_parameters=dict(), collision_parameters=dict()):
    """Return a sphere-shaped simulation model.

    > *Input arguments*

    * `radius` (*type:* `float`): Radius of the sphere.
    * `mass` (*type:* `float`, *default:* `0`): Mass of the model. If the
    mass is not greater than zero, the model is set as static.
    * `name` (*type:* `str`, *default:* `'box'`): Name of the model.
    * `pose` (*type:* `list` or `numpy.array`, *default:*
    `[0, 0, 0, 0, 0, 0]`): Origin of the model.
    * `color` (*type:* `str` or `list`, *default:* `None`): Color
    of the model. It can be provided as a RGBA vector, `xkcd` for
    a random [XKCD color](https://xkcd.com/color/rgb/) or a specific `xkcd`
    color name, and/or `random` for a random RGBA color.

    > *Returns*

    A sphere-shaped `pcg_gazebo.simulation.SimulationModel` instance.
    """
    input_mass = float(_parse_factory_input_as_vector(mass)[0])
    input_radius = float(_parse_factory_input_as_vector(radius)[0])
    input_pose = _parse_factory_input_as_vector(pose).tolist()

    input_col_params = collision_parameters.copy()
    for tag in collision_parameters:
        if tag == 'fdir1':
            input_col_params[tag] = _parse_factory_input_as_vector(
                collision_parameters[tag]).tolist()
        else:
            input_col_params[tag] = float(
                _parse_factory_input_as_vector(
                    collision_parameters[tag])[0])

    model = SimulationModel(name=name)
    model.add_spherical_link(
        link_name='link',
        mass=input_mass,
        radius=input_radius,
        color=color,
        visual_parameters=visual_parameters,
        collision_parameters=input_col_params)

    if input_mass <= 0:
        model.static = True
    model.pose = input_pose
    return model


def cylinder(
        length,
        radius,
        mass=0,
        name='cylinder',
        pose=[0, 0, 0, 0, 0, 0],
        color=None,
        visual_parameters=dict(),
        collision_parameters=dict()):
    """Return a cylinder-shaped simulation model with the rotation axis
    set per default as `[0, 0, 1]`.

    > *Input arguments*

    * `radius` (*type:* `float`): Radius of the cylinder.
    * `length` (*type:* `float`): Length of the cylinder.
    * `mass` (*type:* `float`, *default:* `0`): Mass of the model. If the
    mass is not greater than zero, the model is set as static.
    * `name` (*type:* `str`, *default:* `'box'`): Name of the model.
    * `pose` (*type:* `list` or `numpy.array`, *default:*
    `[0, 0, 0, 0, 0, 0]`): Origin of the model.
    * `color` (*type:* `str` or `list`, *default:* `None`):
    Color of the model. It can be provided as a
    RGBA vector, `xkcd` for a random [XKCD color](https://xkcd.com/color/rgb/)
    or a specific `xkcd` color name, and/or `random` for a random RGBA color.

    > *Returns*

    A cylinder-shaped `pcg_gazebo.simulation.SimulationModel` instance.
    """
    input_mass = float(_parse_factory_input_as_vector(mass)[0])
    input_length = float(_parse_factory_input_as_vector(length)[0])
    input_radius = float(_parse_factory_input_as_vector(radius)[0])
    input_pose = _parse_factory_input_as_vector(pose).tolist()

    input_col_params = collision_parameters.copy()
    for tag in collision_parameters:
        if tag == 'fdir1':
            input_col_params[tag] = _parse_factory_input_as_vector(
                collision_parameters[tag]).tolist()
        else:
            input_col_params[tag] = float(
                _parse_factory_input_as_vector(
                    collision_parameters[tag])[0])

    model = SimulationModel(name=name)
    model.add_cylindrical_link(
        link_name='link',
        mass=input_mass,
        radius=input_radius,
        length=input_length,
        color=color,
        visual_parameters=visual_parameters,
        collision_parameters=input_col_params)

    if input_mass <= 0:
        model.static = True
    model.pose = input_pose
    return model


def box_factory(size, mass=None, name='box', pose=[0, 0, 0, 0, 0, 0],
                use_permutation=True, color=None):
    """Factory function for box-shaped models. It parses the vector `size`
    to generate the boxes. The `mass` can be either a scalar or a vector.
    If `mass` is a scalar, all boxes will have the same mass. If the size
    of the vectors `size` and `mass` are the same, the boxes can be generated
    by associating a `size` vector with a mass by position in the array or they
    can be permutated. If the vectors `size` and `mass` have different lengths,
    only permutation can be performed.

    The `size` and `mass` inputs can also be provided as lambda functions
    as `str`, such as:

    ```python
    size="__import__('numpy').random.random((4, 3))"
    mass="__import__('numpy').arange(1, 10, 4)"
    ```

    > *Input arguments*

    * `size` (*type:* `list` or lambda function as `str`): List of
    3D size vectors
    * `mass` (*type:* `float`,  list of `float` or lambda function as `str`,
    *default:* `None`): Mass of the boxes. If `mass` is `None`, all boxes
    will be static models
    * `pose` (*type:* `list` or `numpy.array`, *default:*
    `[0, 0, 0, 0, 0, 0]`): Origin of the model.
    * `use_permutation` (*type:* `bool`, *default:* `True`): Enable use of
    permutation to associate the `size` elements with the `mass` inputs. If the
    sizes of the `size` and `mass` have different sizes, permutation
    will be used per default.
    * `color` (*type:* `str` or `list`, *default:* `None`): Color of the model.
    It can be provided as a RGBA vector, `xkcd` for a random
    [XKCD color](https://xkcd.com/color/rgb/) or a specific `xkcd`
    color name, and/or `random` for a random RGBA color.

    > *Returns*

    List of `pcg_gazebo.simulation.SimulationModel` instances.
    """

    box_size = _parse_factory_input_as_vector(size)
    if mass is not None:
        box_mass = _parse_factory_input_as_vector(mass)
    else:
        box_mass = None

    if box_size.shape[0] == 3 and len(box_size.shape) == 1:
        box_size = box_size.reshape((1, 3))

    if len(box_size.shape) != 2:
        PCG_ROOT_LOGGER.error(
            'Size of box shapes is invalid, provided={}'.format(
                box_size.shape))
        return list()

    if box_size.shape[1] != 3:
        PCG_ROOT_LOGGER.error(
            'Size of box shapes is invalid, provided={}'.format(
                box_size.shape))
        return list()

    models = list()
    if mass is None:
        for i in range(box_size.shape[0]):
            PCG_ROOT_LOGGER.info(
                '[box_factory] Not using permutation, '
                'generating {} static models'.format(box_size.shape[0]))
            box_name = '{}_{}'.format(name, i)
            models.append(box(
                size=box_size[i, :],
                mass=0,
                name=box_name,
                pose=pose,
                color=color))
    elif not use_permutation:
        if box_size.shape[0] == box_mass.shape[0]:
            PCG_ROOT_LOGGER.info(
                '[box_factory] Not using permutation, '
                'generating {} dynamic models'.format(box_size.shape[0]))
            for i in range(box_size.shape[0]):
                box_name = '{}_{}'.format(name, i)
                models.append(box(
                    size=box_size[i, :],
                    mass=box_mass[i],
                    name=box_name,
                    pose=pose,
                    color=color))
        else:
            PCG_ROOT_LOGGER.info(
                '[box_factory] Since the number of masses and sizes'
                ' provided are different, using permutation and '
                'generating {} models'.format(
                    box_size.shape[0] * box_mass.shape[0]))

    if len(models) == 0:
        PCG_ROOT_LOGGER.info(
            '[box_factory] Using permutation, '
            'generating {} dynamic models'.format(
                box_size.shape[0] * box_mass.shape[0]))
        model_counter = 0
        for box_param in itertools.product(box_size, box_mass):
            box_name = '{}_{}'.format(name, model_counter)
            models.append(box(
                size=box_param[0],
                mass=box_param[1],
                name=box_name,
                pose=pose,
                color=color))
            model_counter += 1

    return models


def sphere_factory(radius, mass=None, name='sphere', pose=[0, 0, 0, 0, 0, 0],
                   use_permutation=True, color=None):
    """Factory function for sphere-shaped models. It
    parses the vector `radius` to generate the spheres.
    The `mass` can be either a scalar or a vector.
    If `mass` is a scalar, all spheres will have the
    same mass. If the size of the vectors `radius` and
    `mass` are the same, the spheres can be generated
    by associating a `radius` value with a mass by position
    in the array or they can be permutated. If the vectors
    `radius` and `mass` have different lengths,
    only permutation can be performed.

    The `radius` and `mass` inputs can also be provided
    as lambda functions as `str`, such as:

    ```python
    radius="__import__('numpy').random.random(2)"
    mass="__import__('numpy').arange(1, 4, 1)"
    ```

    > *Input arguments*

    * `radius` (*type:* `list` or lambda function as `str`):
    List of radius values
    * `mass` (*type:* `float`, list of `float` or lambda
    function as `str`, *default:* `None`): Mass of the boxes.
    If `mass` is `None`, all spheres will be static models
    * `pose` (*type:* `list` or `numpy.array`, *default:*
    `[0, 0, 0, 0, 0, 0]`): Origin of the model.
    * `use_permutation` (*type:* `bool`, *default:* `True`):
    Enable use of permutation to associate the `radius`
    elements with the `mass` inputs. If the sizes of the
    `radius` and `mass` have different sizes, permutation
    will be used per default.
    * `color` (*type:* `str` or `list`, *default:* `None`):
    Color of the model. It can be provided as a RGBA vector,
    `xkcd` for a random [XKCD color](https://xkcd.com/color/rgb/)
    or a specific `xkcd` color name, and/or `random` for a
    random RGBA color.

    > *Returns*

    List of `pcg_gazebo.simulation.SimulationModel` instances.
    """
    sphere_radius = _parse_factory_input_as_vector(radius)
    if mass is not None:
        sphere_mass = _parse_factory_input_as_vector(mass)
    else:
        sphere_mass = None

    PCG_ROOT_LOGGER.info(
        'Generating spheres, radius={},'
        ' mass={}, use_permutation={}, name={}, pose={}'.format(
            sphere_radius,
            sphere_mass,
            use_permutation,
            name,
            pose))

    models = list()

    if mass is None:
        for i in range(sphere_radius.size):
            sphere_name = '{}_{}'.format(name, i)
            models.append(sphere(
                radius=sphere_radius[i],
                mass=0,
                name=sphere_name,
                pose=pose,
                color=color))
    elif not use_permutation:
        if sphere_radius.shape == sphere_mass.shape:
            for i in range(sphere_radius.size):
                sphere_name = '{}_{}'.format(name, i)
                models.append(sphere(
                    radius=sphere_radius[i],
                    mass=sphere_mass[i],
                    name=sphere_name,
                    pose=pose,
                    color=color))

    if len(models) == 0:
        model_counter = 0
        for sphere_param in itertools.product(sphere_radius, sphere_mass):
            sphere_name = '{}_{}'.format(name, model_counter)
            models.append(sphere(
                radius=sphere_param[0],
                mass=sphere_param[1],
                name=sphere_name,
                pose=pose,
                color=color))
            model_counter += 1

    return models


def cylinder_factory(
        length,
        radius,
        mass=None,
        name='cylinder',
        pose=[0, 0, 0, 0, 0, 0],
        use_permutation=True,
        color=None):
    """Factory function for cylinder-shaped models. It parses the
    vectors `radius` and `length` to generate the cylinders. The
    `mass` can be either a scalar or a vector. If `mass` is a scalar,
    all cylinders will have the same mass. If the size of the vectors
    `length`, `radius` and `mass` are the same, the cylinders can be
    generated by associating a `radius` and a `length` value with a
    mass by position in the array or they can be permutated. If the
    vectors `radius` and `length` have different lengths, only
    permutation can be performed.

    The `length`, `radius` and `mass` inputs can also be provided as lambda
    functions as `str`, such as:

    ```python
    length="__import__('numpy').random.random(2)"
    radius="__import__('numpy').random.random(2)"
    mass="__import__('numpy').arange(1, 4, 1)"
    ```

    > *Input arguments*

    * `radius` (*type:* `float`, list of `float` or lambda function as `str`):
    List of radius values
    * `length` (*type:* `float`, list of `float` or lambda function as `str`):
    List of length values
    * `mass` (*type:* `float`, list of `float` or lambda function as `str`,
    *default:* `None`): Mass of the cylinders. If `mass` is `None`, all
    cylinders will be static models
    * `pose` (*type:* `list` or `numpy.array`, *default:*
    `[0, 0, 0, 0, 0, 0]`): Origin of the model.
    * `use_permutation` (*type:* `bool`, *default:* `True`): Enable use of
    permutation to associate the `size` elements with the `mass` inputs. If the
    sizes of the `length` and `radius` have different sizes, permutation will
    be used per default.
    * `color` (*type:* `str` or `list`, *default:* `None`): Color of the model.
    It can be provided as a RGBA vector, `xkcd` for a random
    [XKCD color](https://xkcd.com/color/rgb/) or a specific `xkcd`
    color name, and/or `random` for a random RGBA color.

    > *Returns*

    List of `pcg_gazebo.simulation.SimulationModel` instances.
    """
    cyl_lengths = _parse_factory_input_as_vector(length)
    cyl_radius = _parse_factory_input_as_vector(radius)
    if mass is not None:
        cyl_mass = _parse_factory_input_as_vector(mass)
    else:
        cyl_mass = None

    PCG_ROOT_LOGGER.info(
        'Generating cylinders, length={}, radius={},'
        ' mass={}, use_permutation={}, name={}, pose={}'.format(
            cyl_lengths, cyl_radius, cyl_mass, use_permutation,
            name, pose))

    models = list()

    if not use_permutation:
        if cyl_lengths.shape == cyl_radius.shape:
            for i in range(cyl_lengths.size):
                cyl_name = '{}_{}'.format(name, i)
                if mass is None:
                    models.append(cylinder(
                        cyl_lengths[i],
                        cyl_radius[i],
                        name=cyl_name,
                        pose=pose,
                        color=color))
                elif cyl_mass.size == 1:
                    models.append(cylinder(
                        cyl_lengths[i],
                        cyl_radius[i],
                        cyl_mass[0],
                        name=cyl_name,
                        pose=pose,
                        color=color))
                elif cyl_mass.shape == cyl_lengths.shape:
                    models.append(cylinder(
                        cyl_lengths[i],
                        cyl_radius[i],
                        cyl_mass[i],
                        name=cyl_name,
                        pose=pose,
                        color=color))

    if len(models) == 0:
        model_counter = 0
        if mass is not None:
            for cyl_params in itertools.product(
                    cyl_lengths, cyl_radius, cyl_mass):
                cyl_name = '{}_{}'.format(name, model_counter)
                models.append(
                    cylinder(
                        length=cyl_params[0],
                        radius=cyl_params[1],
                        mass=cyl_params[2],
                        name=cyl_name,
                        pose=pose,
                        color=color))
                model_counter += 1
        else:
            for cyl_params in itertools.product(cyl_lengths, cyl_radius):
                cyl_name = '{}_{}'.format(name, model_counter)
                models.append(
                    cylinder(
                        length=cyl_params[0],
                        radius=cyl_params[1],
                        name=cyl_name,
                        pose=pose,
                        color=color))
                model_counter += 1

    return models


def extrude(
        polygon,
        height,
        thickness=0,
        cap_style='round',
        join_style='round',
        extrude_boundaries=False,
        name='mesh',
        pose=[0, 0, 0, 0, 0, 0],
        color=None,
        mass=0,
        inertia=None,
        use_approximated_inertia=True,
        approximated_inertia_model='box',
        visual_parameters=dict(),
        collision_parameters=dict()):
    from .mesh import extrude

    generated_mesh = extrude(
        polygon=polygon,
        height=height,
        thickness=thickness,
        cap_style=cap_style,
        join_style=join_style,
        extrude_boundaries=extrude_boundaries)

    model = mesh(
        visual_mesh=generated_mesh,
        name=name,
        pose=pose,
        color=color,
        mass=mass,
        inertia=inertia,
        use_approximated_inertia=use_approximated_inertia,
        approximated_inertia_model=approximated_inertia_model,
        visual_parameters=visual_parameters,
        collision_parameters=collision_parameters
    )

    return model


def room(
        polygon,
        wall_height=2,
        wall_thickness=0.1,
        cap_style='square',
        join_style='mitre',
        add_floor=False,
        floor_thickness=0.01,
        name='room',
        pose=[0, 0, 0, 0, 0, 0],
        color=None,
        mass=0,
        separate_models=False,
        visual_parameters=dict(),
        collision_parameters=dict()):
    from .mesh import extrude
    from shapely.geometry import Point, MultiPoint, \
        LineString, MultiLineString

    assert not isinstance(polygon, Point), \
        'A room cannot be created from a point'

    models = list()

    if isinstance(polygon, MultiPoint):
        PCG_ROOT_LOGGER.warning(
            'The input for room construction is a MultiPoint'
            ' object, using the convex hull')
        input_poly = polygon.convex_hull
    else:
        input_poly = polygon

    if isinstance(input_poly, LineString) or \
            isinstance(input_poly, MultiLineString):
        wall_mesh = extrude(
            input_poly,
            height=wall_height,
            thickness=wall_thickness,
            cap_style='flat',
            join_style='mitre',
            extrude_boundaries=False)
    else:
        outer_wall = input_poly.buffer(
            wall_thickness / 2.,
            cap_style=3,
            join_style=2)
        inner_wall = input_poly.buffer(
            -wall_thickness / 2.,
            cap_style=3,
            join_style=2)
        wall_poly = outer_wall.difference(inner_wall)
        wall_mesh = extrude(
            wall_poly,
            height=wall_height,
            extrude_boundaries=False)

    model = SimulationModel(
        name=name if not separate_models else name +
        '_walls')

    model.add_link(
        visual_mesh=wall_mesh,
        collision_mesh=wall_mesh,
        use_approximated_collision=False,
        approximated_collision_model=False,
        visual_mesh_scale=[1, 1, 1],
        collision_mesh_scale=[1, 1, 1],
        name='walls',
        color=color,
        mass=0,
        inertia=None,
        use_approximated_inertia=False,
        pose=[0, 0, wall_height / 2., 0, 0, 0])
    model.pose = pose
    model.static = True

    models.append(model)

    if add_floor:
        if isinstance(input_poly, LineString) or \
                isinstance(input_poly, MultiLineString):
            floor_poly = input_poly.convex_hull
        else:
            floor_poly = input_poly

        floor_poly = floor_poly.buffer(
            wall_thickness / 2.,
            cap_style=3,
            join_style=2)

        floor_mesh = extrude(
            floor_poly,
            height=floor_thickness)

        floor_link_parameters = dict(
            visual_mesh=floor_mesh,
            collision_mesh=floor_mesh,
            use_approximated_collision=False,
            approximated_collision_model=False,
            visual_mesh_scale=[1, 1, 1],
            collision_mesh_scale=[1, 1, 1],
            name='floor',
            color=color,
            mass=0,
            inertia=None,
            use_approximated_inertia=False,
            pose=[0, 0, -floor_thickness / 2., 0, 0, 0])

        if separate_models:
            floor_model = SimulationModel(name=name + '_ground_plane')
            floor_model.add_link(**floor_link_parameters)
            floor_model.static = True
            models.append(floor_model)
        else:
            models[0].add_link(**floor_link_parameters)
    return models


def hinged_door(door_mesh_filename=None, width=0.6,
                thickness=0.04, height=2.0, mass=10,
                set_origin_to_ground=True, fix_to_world=True,
                hand_convention='LH', max_opening_angle=np.pi / 2,
                name='door', frame_mesh_filename=None,
                with_frame=True, frame_width=0.05,
                frame_height=0.05, frame_depth=0.05):
    door = HingedDoor(
        door_mesh_filename=door_mesh_filename,
        width=width,
        thickness=thickness,
        height=height,
        mass=mass,
        set_origin_to_ground=set_origin_to_ground,
        fix_to_world=fix_to_world,
        hand_convention=hand_convention,
        max_opening_angle=max_opening_angle,
        name=name,
        frame_mesh_filename=frame_mesh_filename,
        with_frame=with_frame,
        frame_width=frame_width,
        frame_height=frame_height,
        frame_depth=frame_depth
    )
    return door


def config2models(config):
    """Parse the input `dict` configuration and calls the respective
    model factory.

    > *Input arguments*

    * `config` (*type:* `dict`): Dictionary with the model generation
    rules

    > *Returns*

    List of `pcg_gazebo.simulation.SimulationModel` instances.
    """
    models = list()
    if config['type'] == 'box':
        models.append(box(**config['args']))
    elif config['type'] == 'sphere':
        models.append(sphere(**config['args']))
    elif config['type'] == 'cylinder':
        models.append(cylinder(**config['args']))
    elif config['type'] == 'cylinder_factory':
        models = models + cylinder_factory(**config['args'])
    elif config['type'] == 'sphere_factory':
        models = models + sphere_factory(**config['args'])
    elif config['type'] == 'box_factory':
        models = models + box_factory(**config['args'])
    elif config['type'] == 'mesh':
        models.append(mesh(**config['args']))
    elif config['type'] == 'extrude':
        models.append(extrude(**config['args']))
    elif config['type'] == 'hinged_door':
        models.append(hinged_door(**config['args']))

    return [model.to_sdf() for model in models]


def create_models_from_config(config, n_processes=None):
    """Creation of models from a `dict` configuration input using
    multi-processing.

    > *Input arguments*

    * `config` (*type:* `dict`): Dictionary with the model generation
    rules
    * `n_processes` (*type:* `int`, *default:* `None`): Maximum number of
    processes. If `None`, then use the number of CPUs available.

    > *Returns*

    List of `pcg_gazebo.simulation.SimulationModel` instances.
    """
    if not isinstance(config, list):
        PCG_ROOT_LOGGER.info('Input is not a list of configurations')
        return None

    results = list()

    if n_processes is not None:
        pool = Pool(n_processes)
        output = pool.map(config2models, config)
        for item in output:
            results = results + item
    else:
        for c in config:
            results = results + config2models(c)

    generated_models = list()
    for sdf in results:
        generated_models.append(SimulationModel.from_sdf(sdf))
    return generated_models
