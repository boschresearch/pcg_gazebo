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
import collections
from .properties import Inertial, Collision, Visual, \
    Pose, Footprint, Plugin
from .sensors import Sensor
from .entity import Entity
from .light import Light
from ..log import PCG_ROOT_LOGGER
from ..parsers.sdf import create_sdf_element
from ..parsers.sdf_config import create_sdf_config_element
from ..utils import is_string


class Link(Entity):
    """Representation of a simulated `link` or a single-link `model`.

    > *Input arguments*

    * `name` (*type:* `str`, *value:* `object`): Name of the object.
    * `creation_time` (*type:* `float`, *default:* `None`): Timestamp
    of the creation of the object in Gazebo.
    * `life_timeout` (*type:* `float`, *default:* `None`): Timeout in which
    to remove the object from the simulation (**not implemented**).
    """

    def __init__(self,
                 name='object',
                 creation_time=None,
                 life_timeout=None,
                 pose=[0, 0, 0, 0, 0, 0],
                 inertial=None,
                 static=False,
                 self_collide=False,
                 kinematic=False,
                 gravity=True,
                 visuals=None,
                 collisions=None):
        super(Link, self).__init__(
            name=name, pose=pose)
        assert isinstance(name, str), 'Name must be a string'
        assert len(name) > 0, 'Name cannot be an empty string'
        self._ros_namespace = None
        self._life_timeout = life_timeout
        self._creation_time = creation_time

        self._delete_model_func = None
        self._is_init = False
        self._properties = dict()

        # Initialize link parameters
        self._inertial = None
        self._static = False
        self._self_collide = False
        self._kinematic = False
        self._gravity = True
        self._collisions = list()
        self._visuals = list()
        self._sensors = dict()
        self._lights = dict()
        self._plugins = dict()

        self._include_in_sdf = dict(
            collision=True,
            visual=True)

        self.pose = pose
        self.static = static
        self.self_collide = self_collide
        self.kinematic = kinematic

        if inertial is not None:
            self._inertial = Inertial.create_inertia(**inertial)

        if collisions is not None:
            for collision in collisions:
                if isinstance(collision, dict):
                    self.add_collision(Collision(**collision))
                elif isinstance(collision, Collision):
                    self.add_collision(collision)
                else:
                    PCG_ROOT_LOGGER.error(
                        'Invalid collision element input={}'.format(collision))

        if visuals is not None:
            for visual in visuals:
                if isinstance(visual, dict):
                    self.add_visual(Visual(**visual))
                elif isinstance(visual, Visual):
                    self.add_visual(visual)
                else:
                    PCG_ROOT_LOGGER.error(
                        'Invalid visual element input={}'.format(visual))

    def __str__(self):
        return self.to_sdf().to_xml_as_str(pretty_print=True)

    @staticmethod
    def create_link_from_mesh(
            name='link',
            visual_mesh=None,
            collision_mesh=None,
            use_approximated_collision=False,
            approximated_collision_model='box',
            visual_mesh_scale=[1, 1, 1],
            collision_mesh_scale=[1, 1, 1],
            pose=[0, 0, 0, 0, 0, 0],
            color=None,
            mass=0,
            inertia=None,
            use_approximated_inertia=True,
            approximated_inertia_model='box',
            visual_parameters=dict(),
            collision_parameters=dict()):
        """Factory method to build a link or single-link model from a mesh.
        This method allows not only assigning a mesh as a visual and collision
        geometry, but also using geometrical approximations of the input mesh
        to create, for example, a collision mesh, or computing the moments of
        inertia.

        > *Input arguments*

        * `name` (*type:* `str`, *default:* `link`): Name of the link.
        * `visual_mesh` (*type:* `str` or `trimesh.Trimesh`,
        *default:* `None`): Filename to the visual mesh file or a mesh object.
        * `collision_mesh` (*type:* `str` or `trimesh.Trimesh`,
        *default:* `None`): Filename to the collision mesh file. If the
        input is `None` and `use_approximated_collision` is
        `False`, the visual mesh will be also set as collision mesh.
        * `use_approximated_collision` (*type:* `bool`, *default:* `False`):
        If `True`, the collision geometry will be approximated from the
        visual mesh geometry into a model given by the
        `approximated_collision_model` input.
        * `approximated_collision_model` (*type:* `str`, *default:* `box`):
        Name of the geometry to which the visual geometry will be approximated
        to generated the collision mesh, options are `box`, `cylinder` or
        `sphere`.
        * `visual_mesh_scale` (*type:* `list`, *default:* `[1, 1, 1]`):
        Scaling factors for the visual mesh in X, Y and Z directions.
        * `collision_mesh_scale` (*type:* `list`, *default:* `[1, 1, 1]`):
        Scaling factors for the collision mesh in X, Y and Z directions.
        * `pose` (*type:* `list`, *default:* `[0, 0, 0, 0, 0, 0]`): Link's pose
        with respect to the model frame.
        * `color` (*type:* `list` or `str`, *default:* `None`): Color
        set to the visual mesh. If `None` is provided, no color is set
        and the mesh will inherit the material of the mesh file. If the
        input is `random`, a random RGB color is generated. This input
        can also be set as `xkcd` for a random
        [`xkcd` color](https://xkcd.com/color/rgb/) name, or a string
        with the name of a specific `xkcd` color (e.g., `teal`).
        Otherwise, the input can be an RGB vector as a `list`.
        * `mass` (*type:* `float`, *default:* `0`): Mass of the link
        in kilograms. If the mass is not greater than zero, the link will
        be set as static.
        * `inertia` (*type:* `dict`, *default:* `None`): Moments of
        inertia of the link. This input can be either a dictionary
        defined as `dict(ixx=0, iyy=0, izz=0, ixy=0, ixz=0, iyz=0)` or
        `None`. If `None` is provided, `use_approximated_inertia` is
        `True` and `mass` is greater than zero, the moments of inertia
        will be computed from an approximated visual mesh geometry
        given by the input `approximated_inertia_model`.
        * `use_approximated_inertia` (*type:* `bool`, *default:*
        `True`): If `True` and `mass` is greater tha zero, the moments
        of inertia of the link will be computed from a approximated
        visual mesh model described by `approximated_inertia_model`.
        * `approximated_inertia_model` (*type:* `str`, *default:* `box`):
        Type of geometry approximation to be applied to the visual geometry.
        The dimensions of the geometry will then be used to compute the
        moments of inertia. Options are `box`, `cylinder` or `sphere`.

        > *Returns*

        `pcg_gazebo.simulation.Link` instance.
        """
        link = Link(name=name)

        # Import necessary libraries
        import trimesh
        from .properties import Inertial

        link.enable_visual()
        link.add_empty_visual(name='visual')

        link.get_visual_by_name('visual').set_mesh_as_geometry(
            mesh=visual_mesh, scale=visual_mesh_scale)
        # TODO Enable dict configuration of visual elements

        mesh = trimesh.Scene()
        mesh.add_geometry(link.get_meshes('visual'))
        # Use the meshes's centroid as offset
        link.get_visual_by_name('visual').pose.position = -1 * mesh.centroid

        link.enable_collision()
        link.add_empty_collision(name='collision')

        if use_approximated_collision:
            if approximated_collision_model == 'box':
                size = mesh.bounding_box.bounds[1, :] - \
                    mesh.bounding_box.bounds[0, :]
                # Compute the origin and dimensions of the visual meshes's
                # oriented bounding box
                link.get_collision_by_name(
                    'collision').set_box_as_geometry(size)
            elif approximated_collision_model == 'cylinder':
                cylinder_kwargs = trimesh.bounds.minimum_cylinder(mesh)
                link.get_collision_by_name(
                    'collision').set_cylinder_as_geometry(
                        length=cylinder_kwargs['height'],
                        radius=cylinder_kwargs['radius'])
            elif approximated_collision_model == 'sphere':
                center, radius = trimesh.nsphere.minimum_nsphere(mesh)
                link.get_collision_by_name('collision').set_sphere_as_geometry(
                    radius=radius)
        else:
            if collision_mesh is None:
                link.get_collision_by_name('collision').set_mesh_as_geometry(
                    mesh=visual_mesh, scale=visual_mesh_scale)
            else:
                link.get_collision_by_name('collision').set_mesh_as_geometry(
                    mesh=collision_mesh, scale=collision_mesh_scale)

        mesh = trimesh.Scene()
        mesh.add_geometry(link.get_meshes('collision'))
        link.get_collision_by_name(
            'collision').pose.position = -1 * mesh.centroid
        link.get_collision_by_name(
            'collision').set_physics(**collision_parameters)

        if color is not None:
            if color == 'random':
                link.get_visual_by_name('visual').set_color()
            elif color == 'xkcd':
                link.get_visual_by_name('visual').set_xkcd_color()
            elif is_string(color):
                link.get_visual_by_name('visual').set_xkcd_color(color)
            elif isinstance(color, collections.Iterable) and \
                    len(list(color)) == 3:
                link.get_visual_by_name('visual').set_color(*color)

        # Setting the approximated inertia
        if use_approximated_inertia and mass > 0:
            mesh = trimesh.Scene()
            mesh.add_geometry(link.get_meshes('collision'))
            assert approximated_inertia_model in \
                ['box', 'cylinder', 'sphere'], \
                'Invalid model for approximated inertia,' \
                ' provided={}'.format(
                    approximated_inertia_model)
            if approximated_inertia_model == 'box':
                size = mesh.bounding_box.bounds[1, :] - \
                    mesh.bounding_box.bounds[0, :]
                link.inertial = Inertial.create_cuboid_inertia(mass, *size)
            elif approximated_inertia_model == 'sphere':
                center, radius = trimesh.nsphere.minimum_nsphere(mesh)
                link.inertial = Inertial.create_solid_sphere_inertia(
                    mass, float(radius))
            elif approximated_inertia_model == 'cylinder':
                cylinder_kwargs = trimesh.bounds.minimum_cylinder(mesh)
                link.inertial = Inertial.create_solid_cylinder_inertia(
                    mass,
                    cylinder_kwargs['radius'],
                    cylinder_kwargs['height'], axis=[0, 0, 1])
        elif not use_approximated_inertia and mass > 0:
            assert isinstance(inertia, dict), \
                'Moments of inertia must be provided as a dictionary'
            link.inertial = Inertial()
            link.inertial.mass = mass
            for tag in inertia:
                assert tag in ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'], \
                    'Invalid moment of inertia tag={}'.format(tag)
                setattr(link.inertial, tag, inertia[tag])
        link.pose = pose
        return link

    @property
    def inertial(self):
        """`pcg_gazebo.simulation.properties.Inertial`:
        Description of the object's moments of inertia.
        """
        return self._inertial

    @inertial.setter
    def inertial(self, value):
        assert isinstance(value, Inertial), 'Invalid inertial input'
        self._inertial = value

    @property
    def static(self):
        """`bool`: Flag to indicate if object is static"""
        return self._static

    @static.setter
    def static(self, value):
        assert isinstance(value, bool) or value in [0, 1], \
            'Input should be a boolean, 0 or 1'
        self._static = bool(value)

    @property
    def self_collide(self):
        """`bool`: Self-collision flag"""
        return self._self_collide

    @self_collide.setter
    def self_collide(self, value):
        assert isinstance(value, bool) or value in [0, 1], \
            'Input should be a boolean, 0 or 1'
        self._self_collide = value

    @property
    def kinematic(self):
        """`bool`: Flag to indicate if the model is purely kinematic"""
        return self._kinematic

    @kinematic.setter
    def kinematic(self, value):
        assert isinstance(value, bool) or value in [0, 1], \
            'Input should be a boolean, 0 or 1'
        self._kinematic = value

    @property
    def gravity(self):
        """`bool`: Flag to that link is affected by gravity"""
        return self._gravity

    @gravity.setter
    def gravity(self, value):
        assert isinstance(value, bool) or value in [0, 1], \
            'Input should be a boolean, 0 or 1'
        self._gravity = value

    @property
    def life_timeout(self):
        """`float`: Life timeout timestamp for this object,
        if it represents a single-link model
        """
        return self._life_timeout

    @life_timeout.setter
    def life_timeout(self, value):
        assert isinstance(value, float) or isinstance(value, int)
        assert value > 0
        self._life_timeout = value

    @property
    def creation_time(self):
        """`float`: Time of creation of this object, if
        it represents a single-link model.
        """
        return self._creation_time

    @creation_time.setter
    def creation_time(self, value):
        assert isinstance(value, float) or isinstance(value, int)
        assert value > 0
        self._creation_time = value

    @property
    def collisions(self):
        """List of `pcg_gazebo.simulation.properties.Collision`:
        List of collision models
        """
        return self._collisions

    @property
    def visuals(self):
        """List of `pcg_gazebo.simulation.properties.Visual`:
        List of visual models
        """
        return self._visuals

    @property
    def has_mesh(self):
        for visual in self._visuals:
            if visual.geometry.is_mesh:
                return True
        for collision in self._collisions:
            if collision.geometry.is_mesh:
                return True
        return False

    def enable_collision(self):
        """Enable the inclusion of the collision models
        in the exported SDF description.
        """
        self._include_in_sdf['collision'] = True

    def disable_collision(self):
        """Disable the inclusion of the collision models
        in the exported SDF description.
        """
        self._include_in_sdf['collision'] = False

    def enable_visual(self):
        """Enable the inclusion of the visual models
        in the exported SDF description.
        """
        self._include_in_sdf['visual'] = True

    def disable_visual(self):
        """Disable the inclusion of the collision models
        in the exported SDF description.
        """
        self._include_in_sdf['visual'] = False

    def get_collision_by_name(self, name):
        """Return the collision model associated with the input
        name identifier.

        > *Input arguments*

        * `name` (*type:* `str`): Name of the collision model.

        > *Returns*

        `pcg_gazebo.simulation.properties.Collision`, or `None`
        if not collision with the given name is found.
        """
        for col in self._collisions:
            if col.name == name:
                return col
        return None

    def has_collision(self, name):
        """Test if a collision with the input name exists.

        > *Input arguments*

        * `name` (*type:* `str`): Name of the collision model

        > *Returns*

        `bool`: `True`, if a collision model exists, `False, otherwise.
        """
        for col in self._collisions:
            if col.name == name:
                return True
        return False

    def get_visual_by_name(self, name):
        """Return the visual model associated with the input
        name identifier.

        > *Input arguments*

        * `name` (*type:* `str`): Name of the visual model.

        > *Returns*

        `pcg_gazebo.simulation.properties.Visual`, or `None`
        if not visual with the given name is found.
        """
        for vis in self._visuals:
            if vis.name == name:
                return vis
        return None

    def has_visual(self, name):
        """Test if a visual with the input name exists.

        > *Input arguments*

        * `name` (*type:* `str`): Name of the visual model

        > *Returns*

        `bool`: `True`, if a visual model exists, `False, otherwise.
        """
        for vis in self._visuals:
            if vis.name == name:
                return True
        return False

    def add_empty_visual(self, name='visual'):
        """Create an empty visual model and add it to the object.

        > *Input arguments*

        * `name` (*type:* `str`, *default:* `visual`): Name of the visual
        model.

        > *Returns*

        `bool`: `True` if visual model could be created
        and added to the object.
        `False` if another visual with the same name already exists.
        """
        if self.has_visual(name):
            PCG_ROOT_LOGGER.error(
                'Visual with name {} already exists in link {}'.format(
                    name, self.name))
            return False
        self._visuals.append(Visual(name))
        return True

    def add_visual(self, visual):
        """Add visual model to the object. If a visual element
        with the same name already exists, a suffix will be added
        to the name in the format `_i`, `i` being an integer.

        > *Input arguments*

        * `visual` (*type:* `pcg_gazebo.simulation.properties.Visual`):
        Visual element

        > *Returns*

        `bool`: `True`, if visual element could be added to object.
        """
        visual_name = visual.name
        if self.has_visual(visual.name):
            i = 0
            visual_name = '{}_{}'.format(visual_name, i)
            while self.has_visual(visual_name):
                i += 1
                visual_name = '{}_{}'.format(visual_name, i)
        visual.name = visual_name
        self._visuals.append(visual)
        return True

    def add_empty_collision(self, name='collision'):
        """Create an empty collision model and add it to the object.

        > *Input arguments*

        * `name` (*type:* `str`, *default:* `collision`): Name of the collision
        model.

        > *Returns*

        `bool`: `True` if collision model could be created and
        added to the object.
        `False` if another collision with the same name already exists.
        """
        if self.has_collision(name):
            PCG_ROOT_LOGGER.error(
                'Collision with name {} already exists in link {}'.format(
                    name, self.name))
            return False
        self._collisions.append(Collision(name))
        return True

    def add_collision(self, collision):
        """Add collision model to the object. If a collision element
        with the same name already exists, a suffix will be added
        to the name in the format `_i`, `i` being an integer.

        > *Input arguments*

        * `collision` (*type:* `pcg_gazebo.simulation.properties.Collision`):
        Collision element

        > *Returns*

        `bool`: `True`, if collision element could be added to object.
        """
        collision_name = collision.name
        if self.has_collision(collision.name):
            i = 0
            collision_name = '{}_{}'.format(collision_name, i)
            while self.has_collision(collision_name):
                i += 1
                collision_name = '{}_{}'.format(collision_name, i)

        collision.name = collision_name
        self._collisions.append(collision)
        return True

    def to_sdf(self, type='link', name='model', sdf_version='1.6',
               resource_prefix='', model_folder=None, copy_resources=False):
        """Convert object to an SDF element. The object can be converted
        to different SDF elements according to the `type` input

        * `collision`: SDF collision element
        * `visual`: SDF visual element
        * `link`: SDF link element with collision and visual properties
        * `model`: single-link SDF model element
        * `sdf`: SDF file format with a nested model element.

        > *Input arguments*

        * `type` (*type:* `str`): Type of output SDF element, options are
        `collision`, `visual`, `link`, `model`, `sdf`.
        * `name` (*type:* `str`, *default:* `model`): Name of the output object
        * `sdf_version` (*type:* `str`, *default:* `1.6`): Version
        of the output SDF element

        > *Returns*

        `pcg_gazebo.parsers.types.XMLBase`: SDF element instance.
        """
        assert type in ['collision', 'visual', 'link', 'model', 'sdf'], \
            'Type must be either visual, collision, link or model'
        assert isinstance(name, str), 'Name must be a string'
        assert len(name) > 0, 'Name string cannot be empty'

        if type == 'collision':
            output = list()
            for obj in self._collisions:
                if len(resource_prefix) == 0:
                    rp = self.name
                else:
                    rp = '{}_{}'.format(
                        resource_prefix,
                        self.name)
                output.append(
                    obj.to_sdf(
                        resource_prefix=rp,
                        model_folder=model_folder,
                        copy_resources=copy_resources))
            if len(output) == 1:
                return output[0]
            else:
                return output

        if type == 'visual':
            output = list()
            for obj in self._visuals:
                if len(resource_prefix) == 0:
                    rp = self.name
                else:
                    rp = '{}_{}'.format(
                        resource_prefix,
                        self.name)
                output.append(
                    obj.to_sdf(
                        resource_prefix=rp,
                        model_folder=model_folder,
                        copy_resources=copy_resources))
            if len(output) == 1:
                return output[0]
            else:
                return output

        # Create a link for the plane, initially empty
        link = create_sdf_element('link')
        link.name = self._name
        link.kinematic = self.kinematic
        link.gravity = self.gravity
        link.self_collide = self.self_collide

        # Add collision elements
        if self._include_in_sdf['collision']:
            for item in self._collisions:
                if len(resource_prefix) == 0:
                    rp = self.name
                else:
                    rp = '{}_{}'.format(
                        resource_prefix,
                        self.name)
                sdf = item.to_sdf(
                    resource_prefix=rp,
                    model_folder=model_folder,
                    copy_resources=copy_resources)
                link.add_collision(sdf.name, sdf)
        # Add visual elements
        if self._include_in_sdf['visual']:
            for item in self._visuals:
                if len(resource_prefix) == 0:
                    rp = self.name
                else:
                    rp = '{}_{}'.format(
                        resource_prefix,
                        self.name)
                sdf = item.to_sdf(
                    resource_prefix=rp,
                    model_folder=model_folder,
                    copy_resources=copy_resources)
                link.add_visual(sdf.name, sdf)

        for tag in self._sensors:
            link.add_sensor(tag, self._sensors[tag].to_sdf())

        for tag in self._plugins:
            link.add_plugin(tag, self._plugins[tag].to_sdf())

        for tag in self._lights:
            link.add_light(tag, self._lights[tag].to_sdf())

        if self._inertial is not None:
            link.inertial = self._inertial.to_sdf()

        if type == 'link':
            link.pose = self.pose.to_sdf()
            return link

        model = create_sdf_element('model')
        # Reset SDF model structure with the additional optional elements
        model.pose = self.pose.to_sdf()
        model.static = self.static
        model.allow_auto_disable = False
        model.name = self.name

        model.add_link(self._name, link)

        if type == 'model':
            return model

        sdf = create_sdf_element('sdf')
        sdf.reset('model')

        sdf.version = sdf_version
        sdf.add_model(model.name, model)
        return sdf

    @staticmethod
    def from_sdf(sdf):
        """Factory method to generate a `pcg_gazebo.simulation.Link` instance
        from an SDF instance. Only links can be parsed.

        > *Input arguments*

        * `sdf` (*type:* `pcg_gazebo.parsers.sdf.Link`): SDF object

        > *Returns*

        `pcg_gazebo.simulation.Link`: Simulation object instance
        """
        assert sdf._NAME == 'link', 'Only links can be parsed'
        link = Link()
        link.name = sdf.name
        if sdf.pose is not None:
            link.pose = Pose.from_sdf(sdf.pose)
        link.self_collide = \
            False if sdf.self_collide is None else sdf.self_collide.value
        link.kinematic = \
            False if sdf.kinematic is None else sdf.kinematic.value
        link.gravity = \
            True if sdf.gravity is None else sdf.gravity.value

        if sdf.inertial is not None:
            link._inertial = Inertial.from_sdf(sdf.inertial)

        if sdf.visuals is not None:
            for visual in sdf.visuals:
                if not link.add_visual(Visual.from_sdf(visual)):
                    raise AttributeError(
                        'Could not import visual element {}'.format(
                            visual.name))
        if sdf.collisions is not None:
            for collision in sdf.collisions:
                if not link.add_collision(Collision.from_sdf(collision)):
                    raise AttributeError(
                        'Could not import collision element {}'.format(
                            collision.name))
        if sdf.sensors is not None:
            for sensor in sdf.sensors:
                if not link.add_sensor(sensor.name, Sensor.from_sdf(sensor)):
                    raise AttributeError(
                        'Could not import sensor element {}'.format(
                            sensor.name))
        if sdf.lights is not None:
            for light in sdf.lights:
                if not link.add_light(light.name, Light.from_sdf(light)):
                    raise AttributeError(
                        'Could not import light element {}'.format(
                            light.name))
        return link

    def export_to_gazebo_model(
            self,
            output_dir,
            name='model',
            sdf_version='1.6',
            version='0.1.0',
            author_names=None,
            author_emails=None,
            description='',
            generate_sdf_with_version=False):
        """Export the object as a Gazebo model, in the format

        ```
        model_dir/
            model.sdf
            model.config
        ```

        > *Input arguments*

        * `output_dir` (*type:* `str`): Name of the directory where the model
        directory will be stored.
        * `name` (*type:* `str`, *default:* `model`): Name of the model
        * `sdf_version` (*type:* `str`, *default:* `1.6`): Version of the
        SDF format
        * `version` (*type:* `str`, *default:* `0.1.0`): Gazebo model version
        * `author_names` (*type:* `list`, *default:* `None`): List of authors
        * `author_emails` (*type:* `list`, *default:* `None`): List of e-mails
        * `description` (*type:* `str`): Model description
        * `generate_sdf_with_version` (*type:* `bool`,
        *default:* `False`): Parameter description

        > *Returns*

        `bool`: `True`, if Gazebo model files were exported successfully.
        """
        assert len(author_names) == len(author_emails), 'List of author' \
            ' names and e-mails must be equal'
        sdf_config = create_sdf_config_element('model')
        sdf_config.name = name
        sdf_config.version = version
        sdf_config.description = description
        if author_names is None:
            author_names = list()
        if author_emails is None:
            author_emails = list()

        if len(author_emails) == len(author_names):
            for a_name, a_email in zip(author_names, author_emails):
                sdf_config.add_author()
                sdf_config.authors[-1].name = a_name
                sdf_config.authors[-1].email = a_email

        if generate_sdf_with_version:
            output_sdf_file = 'model_{}.sdf'.format(
                sdf_version.replace('.', '_'))
        else:
            output_sdf_file = 'model.sdf'

        sdf_config.add_sdf()
        sdf_config.sdfs[-1].version = sdf_version
        sdf_config.sdfs[-1].value = output_sdf_file

        if not os.path.isdir(output_dir):
            PCG_ROOT_LOGGER.error(
                'Output directory does not exist, dir={}'.format(output_dir))
            return False

        output_model_dir = os.path.join(output_dir, name)
        if os.path.isdir(output_model_dir):
            PCG_ROOT_LOGGER.error(
                'A model with the same name already exists in output'
                ' directory, dir={}'.format(output_dir))
            return False

        os.makedirs(output_model_dir)

        sdf = self.to_sdf('sdf', name=name, sdf_version=sdf_version)
        sdf.export_xml(os.path.join(output_model_dir, output_sdf_file))
        sdf_config.export_xml(os.path.join(output_model_dir, 'model.config'))
        return True

    def add_inertial(self, mass):
        """This function must be implemented by derived classes."""
        raise NotImplementedError()

    def update_inertial(self):
        """This function must be implemented by derived classes."""
        raise NotImplementedError()

    def update_collision(self):
        """This function must be implemented by derived classes."""
        raise NotImplementedError()

    def update_visual(self):
        """This function must be implemented by derived classes."""
        raise NotImplementedError()

    def add_sensor(self, name, sensor):
        """Add sensor associated to the link.

        > *Input arguments*

        * `name` (*type:* `str`): Name of the sensor
        * `sensor` (*type:* `pcg_gazebo.simulation.sensors.Sensor`):
        Sensor description

        > *Returns*

        `bool`: `True`, if sensor could be added to link.
        """
        if name in self._sensors:
            PCG_ROOT_LOGGER.error(
                'Sensor with name {} already exists for link {}'.format(
                    name, self.name))
            return False

        self._sensors[name] = sensor
        return True

    def add_light(self, name, light):
        if name in self._lights:
            PCG_ROOT_LOGGER.error(
                'Light with name {} already exists for link {}'.format(
                    name, self.name))
            return False

        self._lights[name] = light
        return True

    def add_plugin(self, name='', filename='', plugin=None, **kwargs):
        if plugin is None:
            self._plugins[name] = Plugin(
                name=name,
                filename=filename)
            self._plugins[name].params = kwargs.copy()
        else:
            self._plugins[plugin.name] = plugin

    def to_markers(self):
        """Generate `visualization_msgs/Marker` instances from the visual and/or
        collision entities.

        > *Returns*

        `visualization_msgs/MarkerArray`
        """
        markers = list()
        for visual in self._visuals:
            marker = visual.to_marker()
            if marker is None:
                continue
            marker.ns = self._name
            marker.pose.position.x = visual.pose.position[0]
            marker.pose.position.y = visual.pose.position[1]
            marker.pose.position.z = visual.pose.position[2]

            marker.pose.orientation.w = visual.pose.quat.w
            marker.pose.orientation.x = visual.pose.quat.x
            marker.pose.orientation.y = visual.pose.quat.y
            marker.pose.orientation.z = visual.pose.quat.z

            markers.append(marker)
        return markers

    def get_footprint(self, mesh_type='collision', pose_offset=None,
                      use_bounding_box=False, z_limits=None):
        """Returns the `shapely._GEOMETRIES.Polygon` or
        `shapely._GEOMETRIES.MultiPolygon` that represent the
        projection of the visual or collision meshes on the XY
        plane.

        > *Input arguments*

        * `mesh_type` (*type:* `str`, *default:* `collision`):
        Origin of the meshes, options are `visual` or `collision`.
        * `pose_offset` (*type:* `data_type`, *default:* `None`):
        Pose offset to be applied to all meshes before the
        footprint is computed
        * `use_bounding_box` (*type:* `bool`, *default:* `False`):
        Use the mesh's bounding box for the footprint calculation
        * `z_limits` (*type:* `list`, *default:* `None`): Minimum
        and maximum limits in the Z direction were the meshes
        will be sectioned.

        > *Returns*

        `shapely._GEOMETRIES.Polygon` or `shapely._GEOMETRIES.MultiPolygon`
        """
        assert mesh_type in ['collision', 'visual'], \
            'Origin of footprints must be either collision' \
            ' or visual geometries'
        if pose_offset is not None:
            assert isinstance(
                pose_offset, Pose), 'Invalid pose property object'
        else:
            pose_offset = Pose()

        link_footprint = Footprint()
        geometries = list()
        poses = list()

        if mesh_type == 'visual':
            for visual in self._visuals:
                geometries.append(visual.geometry)
                poses.append(visual.pose)
        else:
            for collision in self._collisions:
                geometries.append(collision.geometry)
                poses.append(collision.pose)

        combined_pose = pose_offset + self._pose

        for pose, geometry in zip(poses, geometries):
            geo_pose = combined_pose + pose

            footprint = geometry.get_footprint(
                geo_pose.position, geo_pose.quat, use_bounding_box, z_limits)

            if footprint is not None:
                link_footprint.add_polygon(footprint.get_footprint_polygon())
        footprint_poly = link_footprint.get_footprint_polygon()
        return footprint_poly

    def get_meshes(self, mesh_type='collision', pose_offset=None):
        """Return all the meshes associated with this link.

        > *Input arguments*

        * `mesh_type` (*type:* `str`, *default:* `collision`): Type of mesh
        to be returned, options are `visual` or `collision`.
        * `pose_offset` (*type:* `list`, *default:* `None`): Pose offset
        to be applied to all meshes.

        > *Returns*

        List of `trimesh` meshes.
        """
        assert mesh_type in ['collision', 'visual'], \
            'Origin of footprints must be either collision' \
            ' or visual geometries'
        if pose_offset is not None:
            assert isinstance(
                pose_offset, Pose), 'Invalid pose property object'
        else:
            pose_offset = Pose()

        geometries = list()
        poses = list()

        if mesh_type == 'visual':
            for visual in self._visuals:
                geometries.append(visual.geometry)
                poses.append(visual.pose)
        else:
            for collision in self._collisions:
                geometries.append(collision.geometry)
                poses.append(collision.pose)

        meshes = list()
        combined_pose = pose_offset + self._pose

        for pose, geometry in zip(poses, geometries):
            geometry_pose = combined_pose + pose

            geo_meshes = geometry.get_mesh(
                geometry_pose.position, geometry_pose.quat)
            if geo_meshes is not None:
                meshes += geo_meshes

        return meshes

    def get_bounds(self, mesh_type='collision'):
        """Return the bounds of the link with respect to its meshes.

        > *Input arguments*

        * `mesh_type` (*type:* `str`, *default:* `collision`): Type of mesh,
        options are `visual` or `collision`.

        > *Returns*

        `dict`: Meshes' bounds
        """
        meshes = self.get_meshes(mesh_type)

        bounds = None
        for mesh in meshes:
            if bounds is None:
                bounds = mesh.bounds
            else:
                cur_bounds = mesh.bounds
                for i in range(3):
                    bounds[0, i] = min(bounds[0, i], cur_bounds[0, i])
                for i in range(3):
                    bounds[1, i] = max(bounds[1, i], cur_bounds[1, i])
        return bounds

    def create_scene(self, mesh_type='collision', add_pseudo_color=True,
                     add_axis=True):
        from ..visualization import create_scene
        return create_scene(
            [self], mesh_type, add_pseudo_color, add_axis=add_axis)
