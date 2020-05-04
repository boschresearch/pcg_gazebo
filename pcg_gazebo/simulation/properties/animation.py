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
from .mesh import Mesh
from ...path import Path
from ...utils import is_string
from ...parsers.sdf import create_sdf_element


class Animation(object):
    def __init__(self, name='animation', interpolate_x=False,
                 filename=None, scale=1):
        self._name = None
        self._interpolate_x = interpolate_x
        self._mesh = None

        if filename is not None:
            self.set_mesh(filename, scale)
        self.name = name

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        assert is_string(value), 'Name must be a string'
        assert len(value) > 0, 'Name cannot be empty'
        self._name = value

    @property
    def filename(self):
        if self._mesh is None:
            return None
        if isinstance(self._mesh, dict):
            return self._mesh['filename'].original_uri
        else:
            return self._mesh._uri.original_uri

    @property
    def scale(self):
        if self._mesh is None:
            return None
        if isinstance(self._mesh, dict):
            return self._mesh['scale']
        else:
            return self._mesh.scale[0]

    @property
    def interpolate_x(self):
        return self._interpolate_x

    def set_mesh(self, filename, scale=1):
        assert is_string(filename), 'Filename must be a string'
        assert len(filename) > 0, 'Filename cannot be empty'
        if filename.endswith('.bvh'):
            self._mesh = dict(
                filename=Path(filename), scale=scale)
        elif filename.endswith('.dae'):
            self._mesh = Mesh(
                filename=filename, scale=[scale for _ in range(3)])
        else:
            raise ValueError('Invalid animation filename={}'.format(filename))

    def to_sdf(self):
        if self._mesh is None:
            return None
        sdf = create_sdf_element('animation')
        sdf.name = self._name
        sdf.interpolate_x = self._interpolate_x
        if isinstance(self._mesh, Mesh):
            if self._mesh._uri.model_uri is not None:
                sdf.filename = self._mesh._uri.model_uri
            elif self._mesh._uri.package_uri is not None:
                sdf.filename = self._mesh._uri.package_uri
            elif self._mesh._uri.file_uri is not None:
                sdf.filename = self._mesh._uri.file_uri
            sdf.scale = [self._mesh.scale[0]]
        else:
            if self._mesh['filename']._uri.model_uri is not None:
                sdf.filename = self._mesh['filename']._uri.model_uri
            elif self._mesh['filename']._uri.package_uri is not None:
                sdf.filename = self._mesh['filename']._uri.package_uri
            elif self._mesh['filename']._uri.file_uri is not None:
                sdf.filename = self._mesh['filename']._uri.file_uri
            sdf.scale = [self._mesh['scale']]
        return sdf

    @staticmethod
    def from_sdf(sdf):
        animation = Animation(
            name=sdf.name,
            interpolate_x=sdf.interpolate_x.value,
            filename=sdf.filename.value,
            scale=sdf.scale.value[0]
        )
        return animation
