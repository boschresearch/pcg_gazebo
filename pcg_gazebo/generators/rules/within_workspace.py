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


class WithinWorkspace(Rule):
    _NAME = 'workspace'

    def __init__(self, dofs=None, workspace=None):
        assert is_string(workspace), \
            'Workspace identifier must be a string,' \
            ' received={}'.format(workspace)
        if dofs is None:
            dofs = dict()
        dofs['x'] = True
        dofs['y'] = True
        for tag in ['roll', 'pitch', 'yaw']:
            dofs[tag] = False
        self._constraints_manager = ConstraintsManager.get_instance()
        self._workspace_tag = workspace
        self._workspace = None
        super(WithinWorkspace, self).__init__(dofs=dofs)

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

    def get_pose(self):
        assert self._constraints_manager.has_element(self._workspace_tag), \
            'Workspace <{}> could not be found in the list of' \
            ' constraints, options are={}'.format(
                self._workspace_tag, self._constraints_manager.tags)
        if self._workspace is None:
            self._workspace = self._constraints_manager.get(
                self._workspace_tag)
            assert self._workspace.type == 'workspace', \
                'Constraint <{}> is not a workspace constraint'.format(
                    self._workspace_tag)
        pose = self._workspace.get_random_position()
        if pose.has_z and not self.dofs['z']:
            pose.z = 0
        return pose

    @staticmethod
    def example():
        sample = Rule.example()
        sample['workspace'] = 'name_of_workspace'
        return sample
