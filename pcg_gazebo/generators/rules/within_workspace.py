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
from .rule import Rule
from ..constraints_manager import ConstraintsManager
from ...utils import is_string, is_boolean
from ...simulation.properties import Pose


class WithinWorkspace(Rule):
    _NAME = 'workspace'

    def __init__(self, dofs=None, workspace=None):
        assert is_string(workspace), \
            'Workspace identifier must be a string,' \
            ' received={}'.format(workspace)
        ws_dofs = dict()
        ws_dofs['x'] = True
        ws_dofs['y'] = True
        if dofs is not None:
            if isinstance(dofs, list):
                ws_dofs['z'] = 'z' in dofs
            elif isinstance(dofs, dict):
                if 'z' in dofs:
                    ws_dofs['z'] = dofs['z']
        for tag in ['roll', 'pitch', 'yaw']:
            ws_dofs[tag] = False
        self._constraints_manager = ConstraintsManager.get_instance()
        self._workspace_tag = workspace
        self._workspace = None
        super(WithinWorkspace, self).__init__(dofs=ws_dofs)

    @property
    def dofs(self):
        return self._dofs

    @dofs.setter
    def dofs(self, value):
        assert isinstance(value, dict), 'Input dofs variable must be a dict'
        assert 'x' in value and 'y' in value, \
            'Input should contain at least x and y flags'
        assert value['x'], 'X should always be true'
        assert value['y'], 'Y should always be true'
        for tag in ['roll', 'pitch', 'yaw']:
            if tag in value:
                assert not value[tag], \
                    '{} should always be false for a rule' \
                    ' of type <workspace>'.format(tag)
        for tag in self._dofs:
            if tag in value:
                assert is_boolean(value[tag]), \
                    'DOF element <{}> must be a boolean, ' \
                    'received={}, type={}'.format(
                        value[tag], type(value[tag]))
                self._dofs[tag] = value[tag]

    @property
    def workspace_tag(self):
        return self._workspace_tag

    @property
    def workspace(self):
        if self._workspace is None:
            self._workspace = self._constraints_manager.get(
                self._workspace_tag)
            assert self._workspace.type == 'workspace', \
                'Constraint <{}> is not a workspace constraint'.format(
                    self._workspace_tag)
        return self._workspace

    def get_pose(self):
        assert self._constraints_manager.has_element(self._workspace_tag), \
            'Workspace <{}> could not be found in the list of' \
            ' constraints, options are={}'.format(
                self._workspace_tag, self._constraints_manager.tags)

        pose = Pose()
        point = self.workspace.get_random_position()
        pose.x = point.xy[0][0]
        pose.y = point.xy[1][0]

        if point.has_z and self.dofs['z']:
            pose.z = 0
        return pose

    @staticmethod
    def example():
        sample = Rule.example()
        sample['workspace'] = 'name_of_workspace'
        sample['tag'] = WithinWorkspace._NAME
        return sample
