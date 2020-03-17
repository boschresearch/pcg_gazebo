# Copyright (c) 2020 - The Procedural Generation for Gazebo authors
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
import trimesh
from trimesh.viewer.notebook import in_notebook
from ..log import PCG_ROOT_LOGGER
from .properties import Pose
from ..utils import is_string, is_array


class Entity(object):
    def __init__(self, name='', pose=[0, 0, 0, 0, 0, 0]):
        # Name of the entity
        self._name = None
        self.name = name
        # Pose of the model wrt to world
        self._pose = Pose()
        self.pose = pose

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        assert is_string(value), \
            'Name should be a string'
        assert len(value) > 0, \
            'Name cannot be an empty string'
        self._name = value

    @property
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, vec):
        if isinstance(vec, Pose):
            self._pose = vec
        else:
            assert is_array(vec), \
                'Input pose vector must be iterable'
            assert len(vec) == 6 or len(vec) == 7, \
                'Pose must be given as position and Euler angles (x, y, z, ' \
                'roll, pitch, yaw) or position and quaternions (x, y, z, ' \
                'qx, qy, qz, qw)'
            for item in vec:
                assert isinstance(item, float) or isinstance(item, int), \
                    'All elements in pose vector must be a float or an integer'

            self._pose = Pose(pos=vec[0:3], rot=vec[3::])

    def create_scene(self):
        raise NotImplementedError(
            'Entity objects cannot generate scenes')

    def show(self, *args, **kwargs):
        scene = self.create_scene(*args, **kwargs)
        if not in_notebook():
            scene.show()
        else:
            return trimesh.viewer.SceneViewer(scene)

    def export_as_mesh(
            self,
            format='obj',
            filename=None,
            folder=None,
            *args,
            **kwargs):
        scene = self.create_scene(
            add_axis=False, *args, **kwargs)

        export_formats = ['obj', 'dae', 'stl']
        if format not in export_formats:
            PCG_ROOT_LOGGER.error(
                'Invalid mesh export format, options={}'.format(
                    export_formats))
            return None
        if folder is not None:
            assert os.path.isdir(folder), \
                'Invalid output folder, provided={}'.format(folder)
        else:
            folder = '/tmp'

        if filename is not None:
            assert is_string(filename), 'Filename is not a string'
        else:
            filename = self.name

        mesh_filename = os.path.join(
            folder,
            filename.split('.')[0] + '.' + format)

        if format == 'obj':
            trimesh.exchange.export.export_mesh(
                scene,
                mesh_filename,
                file_type=format,
                include_color=False,
                include_texture=False)
        elif format == 'dae':
            trimesh.exchange.export.export_mesh(
                scene.dump(),
                mesh_filename,
                file_type=format)
        elif format == 'stl':
            mesh = trimesh.boolean.union(scene.dump())
            trimesh.exchange.export.export_mesh(
                mesh,
                mesh_filename,
                file_type=format)

        return mesh_filename
