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
from ...log import PCG_ROOT_LOGGER
from ...utils import is_string
from ..model import SimulationModel
from ..link import Link
from ..properties import Visual, \
    Collision
import numpy as np


class HingedDoor(SimulationModel):
    _HAND_CONVENTIONS = ['LH', 'RH', 'LHR', 'RHR']

    def __init__(self, door_mesh_filename=None,
                 width=0.6, thickness=0.04, height=2.0, mass=10,
                 set_origin_to_ground=True, fix_to_world=True,
                 hand_convention='LH', max_opening_angle=np.pi / 2,
                 name='door', frame_mesh_filename=None,
                 with_frame=True, frame_width=0.05,
                 frame_height=0.05, frame_depth=0.05):
        super(HingedDoor, self).__init__()
        self._hand_convention = hand_convention
        self._max_opening_angle = max_opening_angle
        # Name of the door model
        self.name = name
        # Name of the links
        self._hinge_joint_name = 'hinge'
        self._door_link_name = 'door'
        self._frame_link_name = 'frame'

        if door_mesh_filename is not None:
            self.create_door_from_mesh(door_mesh_filename)
        else:
            self.create_rectangular_door_link(
                width=width,
                thickness=thickness,
                height=height,
                mass=mass,
                set_origin_to_ground=set_origin_to_ground)

        if with_frame:
            if frame_mesh_filename is not None:
                pass
            else:
                self.create_frame(
                    width=frame_width,
                    height=frame_height,
                    depth=frame_depth)

    @property
    def hand_conventions(self):
        return self._HAND_CONVENTIONS

    def _set_door_model_origin_to_ground(self):
        if self._door_link_name not in self.link_names:
            PCG_ROOT_LOGGER.warning(
                '[{}] Door link does not exist, '
                'model origin will not be set'.format(
                    self._name))

        height = self._links[self._door_link_name].get_bounds()[1, 2] - \
            self._links[self._door_link_name].get_bounds()[0, 2]

        self.pose = [0, 0, height / 2, 0, 0, 0]
        return False

    def _add_hinge_joint(self):
        if self._door_link_name not in self.link_names:
            PCG_ROOT_LOGGER.warning(
                '[{}] Door link does not exist, '
                'hinge joint cannot be created'.format(
                    self._name))
            return False

        if self._hinge_joint_name in self.joint_names:
            self.rm_joint(self._hinge_joint_name)

        width = self._links[self._door_link_name].get_bounds()[1, 1] - \
            self._links[self._door_link_name].get_bounds()[0, 1]

        if self._hand_convention in ['LH', 'LHR']:
            joint_pose = [0, width / 2, 0, 0, 0, 0]
        elif self._hand_convention in ['RH', 'RHR']:
            joint_pose = [0, -width / 2, 0, 0, 0, 0]
        else:
            joint_pose = [0 for _ in range(6)]

        if self._hand_convention in ['LH', 'RHR']:
            axis_limits = dict(lower=0, upper=self._max_opening_angle)
        elif self._hand_convention in ['RH', 'LHR']:
            axis_limits = dict(lower=-self._max_opening_angle, upper=0)

        self.add_joint(
            name=self._hinge_joint_name,
            parent='world',
            child=self._door_link_name,
            pose=joint_pose,
            joint_type='revolute',
            axis_limits=axis_limits,
            use_parent_model_frame=False)

        return True

    def create_door_from_mesh(self, mesh, set_origin_to_ground=True):
        from trimesh.geometry import align_vectors
        from ...transformations import quaternion_from_matrix
        from ...simulation.properties import Pose

        if is_string(mesh):
            if self._door_link_name in self.link_names:
                self.rm_link(self._door_link_name)

            door_link = Link.create_link_from_mesh(
                name=self._door_link_name,
                visual_mesh=mesh,
                use_approximated_collision=True,
                approximated_collision_model='box',
                use_approximated_inertia=True,
                approximated_inertia_model='box'
            )
            door_link.self_collide = False
            self.add_link(name=self._door_link_name, link=door_link)

            # Test if the mesh is aligned with X axis
            # if not, set the visual and collision geometries poses'
            # accordingly to correct the mesh rotation
            link_mesh = \
                self._links[self._door_link_name].visuals[0].geometry._mesh
            center, normal = link_mesh.plane_fit()

            transform = align_vectors(normal, np.array([1, 0, 0]))
            q = quaternion_from_matrix(transform)
            if not np.allclose(q[0:3], 0):
                pose = self._links[self._door_link_name].visuals[0].pose
                self._links[self._door_link_name].visuals[0].pose = \
                    pose + Pose(rot=q)
                pose = self._links[self._door_link_name].collisions[0].pose
                self._links[self._door_link_name].collisions[0].pose = \
                    pose + Pose(rot=q)

        self._add_hinge_joint()
        if set_origin_to_ground:
            self._set_door_model_origin_to_ground()

    def create_rectangular_door_link(
            self,
            width=0.6,
            thickness=0.04,
            height=2.0,
            mass=10,
            set_origin_to_ground=True):
        assert width > 0, 'Door width must be greater than zero'
        assert thickness > 0, 'Door thickness must be greater than zero'
        assert height > 0, 'Door height must be greater than zero'
        assert mass > 0, 'Door mass must be greater than zero'

        if self._door_link_name in self.link_names:
            self.rm_link(self._door_link_name)

        self.add_cuboid_link(
            link_name=self._door_link_name,
            mass=mass,
            size=[thickness, width, height])
        self._links[self._door_link_name].self_collide = False

        self._add_hinge_joint()
        if set_origin_to_ground:
            self._set_door_model_origin_to_ground()

    def create_frame(self, width=0.05, height=0.05,
                     depth=0.05):
        assert self._door_link_name in self._links, 'Door link was not created'

        if self._frame_link_name in self._links:
            self.rm_link(self._frame_link_name)

        frame_link = Link(name=self._frame_link_name)
        frame_link.self_collide = False

        # Get door bounds
        door_bounds = self.links[self._door_link_name].get_bounds()

        # Left frame
        frame_size = [
            depth,
            width,
            door_bounds[1, 2] - door_bounds[0, 2]]

        side_collision = Collision(name='left_frame_collision')
        side_collision.set_box_as_geometry(frame_size)

        side_visual = Visual(name='left_frame_visual')
        side_visual.set_box_as_geometry(frame_size)

        # Add left frame
        frame_link.add_collision(Collision(
            name='left_frame_collision',
            pose=[0, door_bounds[0, 1] - frame_size[1] / 2, 0, 0, 0, 0],
            geometry_type='box',
            geometry_args=dict(size=frame_size)))

        frame_link.add_visual(Visual(
            name='left_frame_visual',
            pose=[0, door_bounds[0, 1] - frame_size[1] / 2, 0, 0, 0, 0],
            geometry_type='box',
            geometry_args=dict(size=frame_size)))

        # Add right frame
        frame_link.add_collision(Collision(
            name='right_frame_collision',
            pose=[0, door_bounds[1, 1] + frame_size[1] / 2, 0, 0, 0, 0],
            geometry_type='box',
            geometry_args=dict(size=frame_size)))

        frame_link.add_visual(Visual(
            name='right_frame_visual',
            pose=[0, door_bounds[1, 1] + frame_size[1] / 2, 0, 0, 0, 0],
            geometry_type='box',
            geometry_args=dict(size=frame_size)))

        # Add top frame
        # Calculate geometry of top frame
        frame_size = [
            depth,
            door_bounds[1, 1] - door_bounds[0, 1] + 2 * width,
            height]

        frame_link.add_collision(Collision(
            name='top_frame_collision',
            pose=[0, 0, door_bounds[1, 2] + frame_size[2] / 2, 0, 0, 0],
            geometry_type='box',
            geometry_args=dict(size=frame_size)))
        frame_link.add_visual(Visual(
            name='top_frame_visual',
            pose=[0, 0, door_bounds[1, 2] + frame_size[2] / 2, 0, 0, 0],
            geometry_type='box',
            geometry_args=dict(size=frame_size)))

        self.add_link(name='frame', link=frame_link)

        self.add_joint(
            name=self._frame_link_name + '_to_world',
            parent='world',
            child=frame_link.name,
            pose=[0, 0, 0, 0, 0, 0],
            joint_type='revolute',
            axis_limits=dict(lower=0, upper=0))
