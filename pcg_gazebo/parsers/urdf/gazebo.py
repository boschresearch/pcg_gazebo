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

from ..types import XMLBase
from ..gazebo.kp import Kp
from ..gazebo.kd import Kd
from ..gazebo.material import Material
from ..gazebo.max_contacts import MaxContacts
from ..gazebo.max_vel import MaxVel
from ..gazebo.min_depth import MinDepth
from ..gazebo.mu1 import Mu1
from ..gazebo.mu2 import Mu2
from ..gazebo.provide_feedback import ProvideFeedback
from ..gazebo.self_collide import SelfCollide
from ..gazebo.stop_cfm import StopCFM
from ..gazebo.stop_erp import StopERP


class Gazebo(XMLBase):
    _NAME = 'gazebo'
    _TYPE = 'urdf'

    _CHILDREN_CREATORS = dict(
        mu1=dict(
            creator=Mu1, default=[0], mode='link', optional=True),
        mu2=dict(
            creator=Mu2, default=[0], mode='link', optional=True),
        material=dict(
            creator=Material, default=['none'], mode='link', optional=True),
        kp=dict(
            creator=Kp, default=[1e12], mode='link', optional=True),
        kd=dict(
            creator=Kd, default=[1], mode='link', optional=True),
        maxContacts=dict(
            creator=MaxContacts, default=[20], mode='link', optional=True),
        minDepth=dict(
            creator=MinDepth, default=[0], mode='link', optional=True),
        maxVel=dict(
            creator=MaxVel, default=[0.01], mode='link', optional=True),
        selfCollide=dict(
            creator=SelfCollide, default=[False], mode='link', optional=True),
        provideFeedback=dict(
            creator=ProvideFeedback, default=[False], mode='joint',
            optional=True),
        stopCfm=dict(
            creator=StopCFM, default=[0.0], mode='joint', optional=True),
        stopErp=dict(
            creator=StopERP, default=[0.2], mode='joint', optional=True)
    )

    _MODES = ['none', 'link', 'joint', 'robot']

    def __init__(self, mode='none', sdf_elements=dict()):
        XMLBase.__init__(self)
        self._has_custom_elements = True
        self.reset(mode)
        for tag in sdf_elements:
            if not self._add_child_element(tag, sdf_elements[tag]):
                raise AttributeError(
                    'Failed to create SDF element {}'.format(tag))

    @property
    def reference(self):
        if 'reference' in self.attributes:
            return self.attributes['reference']
        else:
            return None

    @reference.setter
    def reference(self, value):
        self.attributes['reference'] = value

    @property
    def mu1(self):
        return self._get_child_element('mu1')

    @mu1.setter
    def mu1(self, value):
        if self._mode != 'link':
            self.reset(mode='link')
        self._add_child_element('mu1', value)

    @property
    def mu2(self):
        return self._get_child_element('mu2')

    @mu2.setter
    def mu2(self, value):
        if self._mode != 'link':
            self.reset(mode='link')
        self._add_child_element('mu2', value)

    @property
    def material(self):
        return self._get_child_element('material')

    @material.setter
    def material(self, value):
        if self._mode != 'link':
            self.reset(mode='link')
        self._add_child_element('material', value)

    @property
    def kp(self):
        return self._get_child_element('kp')

    @kp.setter
    def kp(self, value):
        if self._mode != 'link':
            self.reset(mode='link')
        self._add_child_element('kp', value)

    @property
    def kd(self):
        return self._get_child_element('kd')

    @kd.setter
    def kd(self, value):
        if self._mode != 'link':
            self.reset(mode='link')
        self._add_child_element('kd', value)

    @property
    def selfCollide(self):
        return self._get_child_element('selfCollide')

    @selfCollide.setter
    def selfCollide(self, value):
        if self._mode != 'link':
            self.reset(mode='link')
        self._add_child_element('selfCollide', value)

    @property
    def minDepth(self):
        return self._get_child_element('minDepth')

    @minDepth.setter
    def minDepth(self, value):
        if self._mode != 'link':
            self.reset(mode='link')
        self._add_child_element('minDepth', value)

    @property
    def maxVel(self):
        return self._get_child_element('maxVel')

    @maxVel.setter
    def maxVel(self, value):
        if self._mode != 'link':
            self.reset(mode='link')
        self._add_child_element('maxVel', value)

    @property
    def maxContacts(self):
        return self._get_child_element('maxContacts')

    @maxContacts.setter
    def maxContacts(self, value):
        if self._mode != 'link':
            self.reset(mode='link')
        self._add_child_element('maxContacts', value)

    @property
    def stopCfm(self):
        return self._get_child_element('stopCfm')

    @stopCfm.setter
    def stopCfm(self, value):
        if self._mode != 'joint':
            self.reset(mode='joint')
        self._add_child_element('stopCfm', value)

    @property
    def stopErp(self):
        return self._get_child_element('stopErp')

    @stopErp.setter
    def stopErp(self, value):
        if self._mode != 'joint':
            self.reset(mode='joint')
        self._add_child_element('stopErp', value)

    @property
    def provideFeedback(self):
        return self._get_child_element('provideFeedback')

    @provideFeedback.setter
    def provideFeedback(self, value):
        if self._mode != 'joint':
            self.reset(mode='joint')
        self._add_child_element('provideFeedback', value)

    def is_valid_element(self, name):
        from ..sdf import create_sdf_element

        if name in self._CHILDREN_CREATORS:
            return True
        return create_sdf_element(name) is not None

    def _add_child_element(self, name, value=None):
        from ..sdf import create_sdf_element

        if name not in self._CHILDREN_CREATORS:
            if value is None:
                elem = create_sdf_element(name)
            else:
                elem = create_sdf_element(name)
                assert elem is not None, 'Invalid SDF element'
                if elem.has_value():
                    if isinstance(value, dict):
                        if 'attributes' in value:
                            for tag in value['attributes']:
                                setattr(elem, tag, value['attributes'][tag])
                        if 'value' in value:
                            elem.value = value['value']
                        else:
                            for tag in value:
                                if tag == 'attributes':
                                    continue
                                elem._add_child_element(tag, value[tag])
                    else:
                        elem.value = value
                else:
                    if issubclass(value.__class__, XMLBase):
                        if name == value._NAME:
                            elem = value
                    elif isinstance(value, dict):
                        for tag in value:
                            if tag == 'attributes':
                                for att in value[tag]:
                                    setattr(elem, att, value[tag][att])
                            else:
                                if elem._NAME != 'empty':
                                    if isinstance(value[tag], list):
                                        if sum([isinstance(x.__class__, dict)
                                                for x in value[tag]]) == 0:
                                            elem._add_child_element(
                                                tag, value[tag])
                                        else:
                                            for item in value[tag]:
                                                elem._add_child_element(
                                                    tag, item)
                                    else:
                                        try:
                                            elem._add_child_element(
                                                tag, value[tag])
                                        except AssertionError as ex:
                                            self.log_error(
                                                'Error parsing XML'
                                                ' element', ex)

            if elem is None:
                self.log_warning('SDF element {} does not exist'.format(name))
                return False
            else:
                if elem.xml_element_name not in self.children:
                    self.children[name] = list()
                self.children[name].append(elem)
                return True
        else:
            XMLBase._add_child_element(self, name, value)
