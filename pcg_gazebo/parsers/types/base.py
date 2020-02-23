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
from copy import deepcopy
import collections
import numpy as np
import sys
from lxml import etree
from lxml.etree import Element, SubElement
from ...log import PCG_ROOT_LOGGER
from ...utils import is_scalar, is_boolean, is_array
from .. import convert_from_string

if sys.version_info[0] == 2:
    # Importing a io.open compatible with Python 3.x
    from io import open


class XMLBase(object):
    # Name of this XML block (e.g. joint)
    _NAME = ''
    _TYPE = ''
    _CHILDREN_CREATORS = dict()
    _ATTRIBUTES = dict()
    _ATTRIBUTES_VERSIONS = dict()
    _MODES = list()
    _VALUE_OPTIONS = list()
    _FORMAT_VERSIONS = ['1.4', '1.5', '1.6']
    _VALUE_TYPE = ''

    def __init__(self, min_value=None, max_value=None):
        # Block attributes
        self._attributes = dict()
        # Children of this block
        self.children = dict()
        # Value
        self._value = None
        # XML version
        self.sdf_version = "1.6"
        # List of options for this element, if necessary
        self.options = None
        # For optinal child elements that can have multiple instances
        # this counter
        self._n_mult_child_counter = dict()
        # Some elements have modes depending of the input
        self._mode = None
        # String description
        self._description = ''
        # Flag to indicate this element can have children that
        # are not only in the children creator's list
        self._has_custom_elements = False

        self._n_optional_elems = 0
        for tag in self._CHILDREN_CREATORS:
            if 'optional' in self._CHILDREN_CREATORS[tag]:
                if self._CHILDREN_CREATORS[tag]['optional']:
                    self._n_optional_elems += 1

        # Store range limits for scalar value inputs
        if min_value is not None:
            if not self._is_scalar(min_value):
                self.log_error(
                    'Minimum value must be a scalar,'
                    ' received={}, type={}'.format(
                        min_value, type(min_value)),
                    raise_exception=True,
                    exception_type=AssertionError)
            self._min_value = min_value
        else:
            self._min_value = None

        if max_value is not None:
            if min_value is not None:
                if max_value <= min_value:
                    self.log_error(
                        'Maximum value is not greater '
                        'than minimum value, min={}, max={}'.format(
                            min_value, max_value),
                        raise_exception=True,
                        exception_type=AssertionError)
            if not self._is_scalar(max_value):
                self.log_error(
                    'Maximum value must be a scalar,'
                    ' received={}, type={}'.format(
                        max_value, type(max_value)),
                    raise_exception=True,
                    exception_type=AssertionError)
            self._max_value = max_value
        else:
            self._max_value = None

    def __str__(self):
        msg = self.to_xml_as_str(pretty_print=True)
        return msg

    def __ne__(self, other):
        result = self.__eq__(other)
        return not result

    def __eq__(self, other):
        if self._NAME != other._NAME:
            return False
        for tag in self._attributes:
            if tag not in other.attributes:
                return False
            v_this = convert_from_string(self._attributes[tag])
            v_other = convert_from_string(other.attributes[tag])
            if self._is_scalar(v_this):
                if not np.isclose(v_this, v_other):
                    return False
            elif self._is_numeric_vector(v_this):
                if not np.isclose(v_this, v_other).all():
                    return False
            elif self._attributes[tag] != other.attributes[tag]:
                return False

        if self._value is not None and other._value is not None:
            if self._is_numeric_vector(
                    self._value) and self._is_numeric_vector(
                    other._value):
                return np.isclose(self._value, other._value).all()
            elif self._is_scalar(self._value) and self._is_scalar(self._value):
                return np.isclose(self._value, other._value)
            else:
                return self._value == other._value

        for tag in self.children:
            if tag not in other.children:
                return False
            if self.children[tag] != other.children[tag]:
                return False
        return True

    @property
    def attributes(self):
        """`dict`: XML properties"""
        return self._attributes

    @property
    def xml_element_name(self):
        """`str`: Name of the SDF element"""
        return self._NAME

    @property
    def xml_format(self):
        """`str`: Name of the XML format"""
        return self._TYPE

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        self._set_value(value)

    def _is_scalar(self, value):
        return is_scalar(value)

    @property
    def modes(self):
        return self._MODES

    def _is_numeric_vector(self, vec, range=None):
        if not self._is_array(vec):
            return False
        try:
            if range is not None:
                for i in vec:
                    if i < range[0] or i > range[1]:
                        return False
            return True
        except (ValueError, TypeError):
            return False

    def _is_array(self, vec):
        return is_array(vec)

    def _is_boolean(self, value):
        return is_boolean(value)

    def _set_value(self, value):
        raise NotImplementedError()

    def _reset_counter(self, tag):
        self._n_mult_child_counter[tag] = 0

    def _get_counter_state(self, tag):
        assert tag in self._n_mult_child_counter
        return self._n_mult_child_counter[tag]

    def _add_to_counter(self, tag):
        assert tag in self._n_mult_child_counter
        self._n_mult_child_counter[tag] + 1

    def _get_child_element(self, tag, version=None):
        if tag not in self.children:
            return None
        if version is not None:
            if self._child_exists_in_version(tag, version):
                return self.children[tag]
            else:
                return None
        else:
            return self.children[tag]

    def _get_child_element_name(self, tag):
        if tag not in self._CHILDREN_CREATORS:
            return None
        elif self._CHILDREN_CREATORS[tag]['creator'] is not None:
            return self._CHILDREN_CREATORS[tag]['creator']._NAME
        else:
            # If child creator is given as None, it is interpreted
            # that this element is a nested instance of the same
            # class
            return self._NAME

    def _get_child_element_creator(self, tag):
        if tag not in self._CHILDREN_CREATORS:
            return None
        elif self._CHILDREN_CREATORS[tag]['creator'] is not None:
            return self._CHILDREN_CREATORS[tag]['creator']
        else:
            return type(self)

    def _add_child_element(self, tag, value, use_as_if_duplicated=None):
        if not self._has_custom_elements:
            assert tag in self._CHILDREN_CREATORS, \
                '<{}> child not found for <{}>, value=\n{}'.format(
                    tag, self._NAME, value)
        assert value is not None, \
            'Input value for element <{}> in element <{}>' \
            ' cannot be None'.format(
                tag, self._NAME)

        if self.has_value():
            if issubclass(value.__class__, XMLBase):
                assert value.has_value()
                setattr(self.children[tag], 'value', value.value)
            else:
                assert hasattr(self.children[tag], 'value')
                setattr(self.children[tag], 'value', value)
        else:
            creator = self._get_child_element_creator(tag)
            assert creator is not None, 'No creator for {} was found'.format(
                tag)
            if 'default' in self._CHILDREN_CREATORS[tag]:
                obj = creator(*self._CHILDREN_CREATORS[tag]['default'])
            else:
                obj = creator()

            if issubclass(value.__class__, XMLBase):
                if self._get_child_element_name(tag) != 'empty':
                    if value.has_value():
                        if len(value.attributes):
                            for att in value.attributes:
                                setattr(obj, att, value.attributes[att])
                        assert hasattr(
                            obj, 'value'), 'No element <{}> for <{}>'.format(
                            'value', tag)
                        setattr(obj, 'value', value.value)
                    else:
                        assert self._get_child_element_name(tag) == \
                            value._NAME, \
                            'Child creator has a different name from' \
                            ' input value, received={}, ' \
                            'expected={}'.format(
                                value._NAME,
                                self._get_child_element_name(tag))

                        obj = value
                        data = value.to_dict(False)
                        # Checking if the element's attributes are consistent
                        for elem in data:
                            # Setting attributes, if provided
                            if elem == 'attributes':
                                for att in data[elem]:
                                    assert att in obj.attributes or hasattr(
                                        obj, att), 'No attribute <{}>' \
                                        ' for <{}>'.format(elem, tag)
                            else:
                                if not obj.is_valid_element(elem):
                                    PCG_ROOT_LOGGER.warning(
                                        'No element <{}> for <{}>,'
                                        ' input={}'.format(
                                            elem, tag, value))
                                continue
                        # Copy element
                        obj = deepcopy(value)
            else:
                if isinstance(value, dict):
                    # The empty entity takes to arguments
                    if self._get_child_element_name(tag) != 'empty':
                        # Check if it is not an empty dictionary
                        assert len(
                            value) != 0, 'Dictionary for object {}' \
                            ' properties is empty'.format(tag)
                    for elem in value:
                        # Setting attributes, if provided
                        if elem == 'attributes':
                            for att in value[elem]:
                                assert att in obj.attributes or hasattr(
                                    obj, att), 'No attribute <{}> for' \
                                    ' <{}>, input={}'.format(
                                        att, tag, value[elem])
                        else:
                            if not obj.is_valid_element(elem):
                                PCG_ROOT_LOGGER.warning(
                                    'No element <{}> for <{}>,'
                                    ' input={}'.format(
                                        elem, tag, value[elem]))
                                continue

                    for elem in value:
                        if elem == 'attributes':
                            for att in value[elem]:
                                setattr(obj, att, value[elem][att])
                        elif obj.has_value() and elem == 'value':
                            setattr(obj, 'value', value[elem])
                        else:
                            if obj._NAME != 'empty':
                                has_mult = False
                                if not obj._has_custom_elements:
                                    if elem not in obj._CHILDREN_CREATORS:
                                        msg = '<{}> element is not a child' \
                                            ' element from <{}>, ' \
                                            'input={}'.format(
                                                elem, obj._NAME, value[elem])
                                        PCG_ROOT_LOGGER.error(msg)
                                        return
                                    if 'n_elems' in \
                                            obj._CHILDREN_CREATORS[elem]:
                                        if obj._CHILDREN_CREATORS[elem][
                                                'n_elems'] == '+':
                                            has_mult = True
                                else:
                                    if elem in obj._CHILDREN_CREATORS:
                                        if 'n_elems' in \
                                                obj._CHILDREN_CREATORS[elem]:
                                            if obj._CHILDREN_CREATORS[elem][
                                                    'n_elems'] == '+':
                                                has_mult = True
                                    else:
                                        has_mult = True

                                if isinstance(value[elem], list) and has_mult:
                                    for item in value[elem]:
                                        obj._add_child_element(elem, item)
                                else:
                                    obj._add_child_element(elem, value[elem])
                elif isinstance(value, collections.Iterable) and \
                        not obj.has_value():
                    for subelem in value:
                        if isinstance(subelem, collections.Iterable):
                            for elem in subelem:
                                has_mult = False
                                if elem in obj._CHILDREN_CREATORS:
                                    if 'n_elems' in \
                                            obj._CHILDREN_CREATORS[elem]:
                                        if obj._CHILDREN_CREATORS[elem][
                                                'n_elems'] == '+':
                                            has_mult = True
                                assert has_mult, 'No multiple' \
                                    ' elements for {}'.format(
                                        elem)
                                obj._add_child_element(elem, subelem[elem])
                elif obj.has_value():
                    tag_blacklist = ['friction']
                    if tag not in tag_blacklist:
                        assert len(obj._CHILDREN_CREATORS) == 0, \
                            'No value parameter found for <{}>, value={},' \
                            ' value type={}'.format(
                                tag, value, type(value))
                    if obj._NAME != 'empty':
                        setattr(obj, 'value', value)
                else:
                    return

            has_mult = False
            if not self._has_custom_elements:
                assert tag in self._CHILDREN_CREATORS, \
                    '{} element is not a child element from {}'.format(
                        tag, self._NAME)
                if 'n_elems' in self._CHILDREN_CREATORS[tag]:
                    if self._CHILDREN_CREATORS[tag]['n_elems'] == '+':
                        has_mult = True
            else:
                if tag in self._CHILDREN_CREATORS:
                    if 'n_elems' in self._CHILDREN_CREATORS[tag]:
                        if self._CHILDREN_CREATORS[tag]['n_elems'] == '+':
                            has_mult = True
                else:
                    has_mult = True

            if has_mult:
                if obj._NAME not in self.children:
                    self.children[obj._NAME] = list()
                self.children[obj._NAME].append(obj)
            else:
                self.children[obj._NAME] = obj

            if self.is_child_and_attribute(obj._NAME):
                self.attributes[obj._NAME] = obj.value

            mode = self._get_child_element_mode(obj._NAME)
            if mode is not None:
                self._mode = mode
                self._rm_child_elements_from_other_modes(mode)

    def _rm_child_elements_from_other_modes(self, mode):
        assert mode in self._MODES, \
            'Mode {} is invalid, modes={}'.format(mode, self._MODES)
        new_children = dict()
        for tag in self.children:
            if self._get_child_element_mode(tag) in [None, mode]:
                new_children[tag] = self.children[tag]
        self.children = new_children

    def _get_child_element_mode(self, tag):
        if tag not in self._CHILDREN_CREATORS:
            return None
        if 'mode' not in self._CHILDREN_CREATORS[tag]:
            return None
        return self._CHILDREN_CREATORS[tag]['mode']

    def _child_has_multiple_elements(self, child_name):
        for tag in self._CHILDREN_CREATORS:
            if 'n_elems' not in self._CHILDREN_CREATORS[tag]:
                continue
            if self._get_child_element_name(tag) == child_name and \
                    self._CHILDREN_CREATORS[tag]['n_elems'] == '+':
                return True
        return False

    def _child_sdf_versions(self, child_name):
        if child_name in self._CHILDREN_CREATORS:
            if 'sdf_versions' in self._CHILDREN_CREATORS[child_name]:
                return self._CHILDREN_CREATORS[child_name]['sdf_versions']
            else:
                return self._FORMAT_VERSIONS
        return self._FORMAT_VERSIONS

    def _child_exists_in_version(self, child_name, sdf_version):
        versions = self._child_sdf_versions(child_name)
        return sdf_version in versions

    def _get_num_mandatory_elems(self):
        n_elems = 0
        for tag in self._CHILDREN_CREATORS:
            if 'optional' in self._CHILDREN_CREATORS[tag]:
                if self._CHILDREN_CREATORS[tag]['optional']:
                    continue
            if 'mode' in self._CHILDREN_CREATORS[tag]:
                if self._CHILDREN_CREATORS[tag]['mode'] == self._mode:
                    n_elems += 1
            else:
                n_elems += 1
        return n_elems

    def rm_child(self, tag):
        if tag in self.children:
            self.children.pop(tag)
            return True
        else:
            return False

    def get_modes(self):
        return self._MODES

    def get_mode(self):
        return self._mode

    def get_value_options(self):
        return self._VALUE_OPTIONS

    def get_num_elements(self, tag):
        if tag not in self.children:
            return None
        if isinstance(self.children, list):
            return len(self.children[tag])
        else:
            return 1

    def is_valid(self):
        if not self.has_value():
            if len(self.children) < self._get_num_mandatory_elems():
                PCG_ROOT_LOGGER.error(
                    '{} must have at least {} child elements'.format(
                        self._NAME, len(
                            self._CHILDREN_CREATORS)))
                return False

            for tag in self._CHILDREN_CREATORS:
                child_name = self._get_child_element_name(tag)

                is_optional = False
                if 'optional' in self._CHILDREN_CREATORS[tag]:
                    is_optional = self._CHILDREN_CREATORS[tag]['optional']
                if 'mode' in self._CHILDREN_CREATORS[tag]:
                    is_optional = is_optional or \
                        self._CHILDREN_CREATORS[tag]['mode'] != self._mode

                if child_name not in self.children and not is_optional:
                    PCG_ROOT_LOGGER.error(
                        'Mandatory element {} not found'.format(child_name))
                    return False

                if child_name in self.children:
                    if not isinstance(self.children[child_name], list):
                        if not isinstance(
                                self.children[child_name],
                                self._get_child_element_creator(tag)):
                            PCG_ROOT_LOGGER.error(
                                'Element {} set as the wrong type,'
                                ' expected type={}, actual type={}'.format(
                                    child_name,
                                    self._get_child_element_creator(tag),
                                    type(
                                        self.children[child_name])))
                            return False
                    else:
                        for i in range(len(self.children[child_name])):
                            if not isinstance(
                                    self.children[child_name][i],
                                    self._get_child_element_creator(tag)):
                                PCG_ROOT_LOGGER.error(
                                    'Element {} #{} set as the wrong'
                                    ' type'.format(
                                        child_name, i))
                                return False

        is_valid = True
        for child in self.children.values():
            if not isinstance(child, list):
                is_valid = is_valid and child.is_valid()
            else:
                for subchild in child:
                    is_valid = is_valid and subchild.is_valid()
        return is_valid

    def get_formatted_value_as_str(self):
        raise NotImplementedError(
            '[{}] has not implemented this method'.format(
                self.xml_element_name))

    def has_value(self):
        return self._value is not None

    def get_number_of_elements(self, tag):
        if tag not in self._n_mult_child_counter:
            if tag in self.children:
                return 1
            else:
                return 0
        else:
            return self._n_mult_child_counter[tag]

    def has_element(self, tag):
        return tag in self.children

    def is_valid_element(self, name):
        if name == 'value' and self.has_value():
            return True
        elif self._has_custom_elements:
            return True
        else:
            for tag in self._CHILDREN_CREATORS:
                if name == self._get_child_element_name(tag):
                    return True
            return hasattr(self, name)

    def reset(self, mode=None, with_optional_elements=False):
        if mode is not None and len(self._MODES) > 0:
            assert mode in self._MODES, 'Mode {} does not' \
                ' belong to the list of valid modes={}'.format(
                    mode, self._MODES)
            self._mode = mode
        elif mode is None and len(self._MODES) > 0:
            pass
        else:
            self._mode = mode

        if len(self._ATTRIBUTES):
            self._attributes = dict()
            for k in self._ATTRIBUTES:
                self._attributes[k] = self._ATTRIBUTES[k]

        if len(self._CHILDREN_CREATORS) > 0:
            self.children = dict()
            for child in self._CHILDREN_CREATORS:
                if 'optional' in self._CHILDREN_CREATORS[child]:
                    if self._CHILDREN_CREATORS[child]['optional'] and \
                            not with_optional_elements:
                        continue
                # Test if element has a mode option and compare to the
                # mode provided
                if self._mode is not None:
                    if 'mode' in self._CHILDREN_CREATORS[child]:
                        if self._mode != \
                                self._CHILDREN_CREATORS[child]['mode']:
                            continue

                tag = self._get_child_element_name(child)

                if 'n_elems' in self._CHILDREN_CREATORS[child]:
                    if self._CHILDREN_CREATORS[child]['n_elems'] == '+':
                        self.children[tag] = list()

                creator = self._get_child_element_creator(child)
                if 'default' in self._CHILDREN_CREATORS[child]:
                    assert isinstance(
                        self._CHILDREN_CREATORS[child]['default'], list)
                    obj = creator(*self._CHILDREN_CREATORS[child]['default'])
                else:
                    obj = creator()

                if not obj.has_value():
                    obj.reset(with_optional_elements=with_optional_elements)

                if 'n_elems' in self._CHILDREN_CREATORS[child]:
                    if self._CHILDREN_CREATORS[child]['n_elems'] == '+':
                        self.children[tag].append(obj)
                    else:
                        self.children[tag] = obj
                else:
                    self.children[tag] = obj

    def get_attributes(self, version='1.6'):
        att = dict()
        for name in self.attributes:
            if name in self._ATTRIBUTES_VERSIONS:
                if version in self._ATTRIBUTES_VERSIONS[name]:
                    att[name] = self.attributes[name]
            else:
                att[name] = self.attributes[name]
        return att

    def to_xml(self, root=None, version='1.6'):
        assert version in self._FORMAT_VERSIONS, \
            'Invalid version, options={}'.format(self._FORMAT_VERSIONS)
        assert self.is_valid(), 'XML data is invalid'

        # Adding attributes
        att = self.get_attributes(version)
        for tag in att:
            # Test if the element has both the options to use
            # an input as attribute or as a child
            if self.is_child_and_attribute(tag):
                if hasattr(self, '_use_{}_as'.format(tag)):
                    if getattr(self, '_use_{}_as'.format(tag)) == 'child':
                        continue

            att[tag] = str(att[tag])

        if root is None:
            base = Element(self._NAME, attrib=att)
        else:
            base = SubElement(root, self._NAME, attrib=att)

        if self.has_value():
            base.text = self.get_formatted_value_as_str()
        else:
            if len(self.children) > 0:
                for child_name in self.children:
                    if isinstance(self.children[child_name], list):
                        if self._child_exists_in_version(child_name, version):
                            for elem in self.children[child_name]:
                                elem.to_xml(base, version)
                        else:
                            PCG_ROOT_LOGGER.info(
                                '<{}> child element not available'
                                ' for version {}'.format(
                                    child_name, version))
                    else:
                        # Test if the element has both the options to use
                        # an input as attribute or as a child
                        if self.is_child_and_attribute(child_name):
                            # If that is the case, check if it was
                            # explicitly defined as a child or an attribute
                            if hasattr(self, '_use_{}_as'.format(child_name)):
                                if getattr(self, '_use_{}_as'.format(
                                        child_name)) == 'child':
                                    self.children[child_name].to_xml(
                                        base, version)
                                    continue
                        elif self._child_exists_in_version(
                                child_name, version):
                            self.children[child_name].to_xml(base, version)
                        else:
                            PCG_ROOT_LOGGER.info(
                                '<{}> child element not available'
                                ' for version {}'.format(
                                    child_name, version))
        return base

    def to_dict(self, root=True):
        output = dict()

        if self.has_value():
            if root:
                output[self._NAME] = dict(value=self.value)
            else:
                output = dict(value=self.value)

            if len(self.attributes):
                output['attributes'] = self.attributes

            if root:
                root_output = dict()
                root_output[self._NAME] = output
                return root_output
            else:
                return output
        else:
            data = dict()
            for child in self.children:
                if self.children[child] is not None:
                    if not isinstance(self.children[child], list):
                        data[child] = self.children[child].to_dict(False)
                        if len(self.children[child].attributes):
                            data[child]['attributes'] = \
                                self.children[child].attributes
                    else:
                        if child not in data:
                            data[child] = list()
                        for elem in self.children[child]:
                            data[child].append(elem.to_dict(False))
                            if len(elem.attributes):
                                data[child][-1]['attributes'] = elem.attributes
            if root:
                output[self._NAME] = data
                if len(self._attributes):
                    output[self._NAME]['attributes'] = self.attributes
                return output
            else:
                return data

    def has_duplicated_child_and_attribute(self):
        for tag in self.attributes:
            if tag in self._CHILDREN_CREATORS:
                return True
        return False

    def is_child_and_attribute(self, tag):
        return tag in self.attributes and tag in self._CHILDREN_CREATORS

    def from_dict(self, sdf_data, ignore_tags=list()):
        for tag in sdf_data:
            if tag in ignore_tags:
                continue
            if isinstance(sdf_data[tag], list) and tag != 'value':
                for elem in sdf_data[tag]:
                    self._add_child_element(tag, elem)
            elif tag == 'attributes':
                for att in sdf_data[tag]:
                    if not hasattr(self, att):
                        PCG_ROOT_LOGGER.warning(
                            'WARNING: Attribute {} does'
                            ' not exist for {}'.format(
                                att, self._NAME))
                    else:
                        setattr(self, att, sdf_data[tag][att])
            else:
                if self._child_has_multiple_elements(tag):
                    self._add_child_element(tag, sdf_data[tag])
                else:
                    if hasattr(self, tag):
                        setattr(self, tag, sdf_data[tag])
                    else:
                        self._add_child_element(tag, sdf_data[tag])

    def to_xml_as_str(self, pretty_print=False, version='1.6'):
        elem = self.to_xml(version=version)
        return etree.tostring(elem, pretty_print=pretty_print).decode('utf-8')

    def export_xml(self, filename, version='1.6'):
        xml_root = self.to_xml(version=version)
        with open(filename, 'w+', encoding='utf-8') as output_xml:
            if sys.version_info[0] == 2:
                output_xml.write(u'<?xml version="1.0" ?>\n')
            else:
                output_xml.write('<?xml version="1.0" ?>\n')
            output_xml.write(
                etree.tostring(xml_root, pretty_print=True,
                               encoding='utf-8').decode('utf-8'))

    def to_urdf(self):
        raise NotImplementedError(
            '{} has no implementation of to_urdf method'.format(self._NAME))

    def to_sdf(self):
        raise NotImplementedError(
            '{} has no implementation of to_sdf method'.format(self._NAME))

    def random(self):
        if len(self.children) > 0:
            try:
                for tag in self.children:
                    if tag != 'empty':
                        self.children[tag].random()
            except Exception:
                pass

    def log_error(self, msg, ex=None, raise_exception=False,
                  exception_type=None):
        error_msg = '[{}] {}'.format(
            self.xml_element_name, msg)
        if ex is not None:
            error_msg += ', message={}'.format(str(ex))
        PCG_ROOT_LOGGER.error(error_msg)
        if raise_exception:
            if exception_type is None:
                raise Exception(error_msg)
            else:
                raise exception_type(error_msg)

    def log_warning(self, msg, ex=None):
        warning_msg = '[{}] {}'.format(
            self.xml_element_name, msg)
        if ex is not None:
            warning_msg += ', message={}'.format(str(ex))
        PCG_ROOT_LOGGER.warning(warning_msg)

    def log_info(self, msg):
        info_msg = '[{}] {}'.format(
            self.xml_element_name, msg)
        PCG_ROOT_LOGGER.info(info_msg)
