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
"""Parsing module to generated and convert SDF,
URDF and SDF Configuration formats.

> *Sources*

* [SDF format](http://sdformat.org/)
* [URDF format specifications](https://wiki.ros.org/urdf/XML)
"""


def parse_sdf(input_xml):
    """Parse an XML file in the SDF format and generates
    an `pcg_gazebo` SDF instance.

    > *Input arguments*

    * `input_xml` (*type:* `str`): Filename of the SDF file or
    SDF XML formatted text.

    > *Returns*

    `pcg_gazebo.parsers.types.XMLBase` object.
    """
    sdf = parse_xml(input_xml, type='sdf')
    return sdf


def parse_urdf(input_xml):
    """Parse an XML file in the URDF format and generates
    an `pcg_gazebo` URDF instance.

    > *Input arguments*

    * `input_xml` (*type:* `str`): Filename of the URDF file or
    URDF XML formatted text.

    > *Returns*

    `pcg_gazebo.parsers.types.XMLBase` object.
    """
    urdf = parse_xml(input_xml, type='urdf')
    if urdf.xml_element_name == 'robot':
        urdf.update_materials()
    return urdf


def parse_sdf_config(input_xml):
    """Parse an XML file in the SDF Configuration format and generates
    an `pcg_gazebo` SDF Configuration instance.

    > *Input arguments*

    * `input_xml` (*type:* `str`): Filename of the SDF Configuration file or
    SDF Configuration XML formatted text.

    > *Returns*

    `pcg_gazebo.parsers.types.XMLBase` object.
    """
    sdf_config = parse_xml(input_xml, type='sdf_config')
    return sdf_config


def parse_xacro(input_xml, output_type='urdf'):
    import os
    import subprocess

    assert os.path.isfile(input_xml), \
        'Input {} is not a valid xacro file'.format(input_xml)

    cmd = 'xacro {}'.format(input_xml)
    output = subprocess.check_output(cmd.split())
    xml = output.decode()

    if output_type == 'urdf':
        output_xml = parse_urdf(xml)
    elif output_type == 'sdf':
        output_xml = parse_sdf(xml)
    elif output_type == 'sdf_config':
        output_xml = parse_sdf_config(xml)
    else:
        output_xml = xml

    return output_xml


def parse_xml(input_xml, type='sdf'):
    """Parse an XML file into an `collections.OrderedDict`.

    > *Input arguments*

    * `input_xml` (*type:* `str`): Filename of the XML file or
    XML formatted text.
    * `type` (*type:* `str`): Type of XML format used in the input
    file, options are `sdf`, `urdf` or `sdf_config`.

    > *Returns*

    `collections.OrderedDict`: Dictionary where the XML tags are the keys.
    """
    import os
    from ..utils import is_string
    from ..log import PCG_ROOT_LOGGER

    if not is_string(input_xml):
        msg = 'Input XML must be either a' \
            ' filename or a string, received={}'.format(
                input_xml)
        PCG_ROOT_LOGGER.error(msg)
        raise Exception(msg)

    if os.path.isfile(input_xml):
        filename = input_xml
        assert os.path.isfile(filename), 'File does not exist'
        output = str()
        with open(filename, 'r') as xml_file:
            for line in xml_file:
                output += line
    else:
        output = input_xml
    return parse_xml_str(output, type)


def parse_xml_str(xml_str, type='sdf'):
    """Parse an XML formatted string into an
    `collections.OrderedDict`.

    > *Input arguments*

    * `input_xml` (*type:* `str`): XML formatted text.
    * `type` (*type:* `str`): Type of XML format used in the input
    file, options are `sdf`, `urdf` or `sdf_config`.

    > *Returns*

    `collections.OrderedDict`: Dictionary where the XML tags are the keys.
    """
    import xmltodict
    parsed_xml = xmltodict.parse(xml_str, encoding='utf-8')
    return parse_xml_dict(parsed_xml, type)


def parse_xml_dict(xml_dict, type='sdf'):
    """Converts an `collections.OrderedDict` created from a XML file
    and return an SDF, URDF or SDF Configuration `pcg_gazebo` element.

    > *Input arguments*

    * `xml_dict` (*type:* `collections.OrderedDict`): XML contents.
    * `type` (*type:* `str`): Type of XML format used in the input
    file, options are `sdf`, `urdf` or `sdf_config`.

    > *Returns*

    `pcg_gazebo.parsers.types.XMLBase` object.
    """
    from .sdf import create_sdf_element
    from .urdf import create_urdf_element
    from .sdf_config import create_sdf_config_element

    data = convert_to_dict(xml_dict)

    name = list(xml_dict.keys())[0]
    if type == 'sdf':
        obj = create_sdf_element(name)
    elif type == 'urdf':
        obj = create_urdf_element(name)
    elif type == 'sdf_config':
        obj = create_sdf_config_element(name)
    else:
        raise TypeError('File type {} is invalid'.format(type))
    assert obj is not None, 'Element {} does not exist'.format(name)

    obj.from_dict(data[name])
    return obj


def convert_custom(xml_dict):
    import collections

    if isinstance(xml_dict, list):
        output = list()
        for elem in xml_dict:
            output.append(convert_custom(elem))
    else:
        tags = list(xml_dict.keys())
        output = dict()
        for tag in tags:
            if '@' in tag:
                if 'attributes' not in output:
                    output['attributes'] = dict()
                output['attributes'][tag.replace(
                    '@', '')] = convert_from_string(xml_dict[tag])
            elif '#' in tag:
                if 'value' not in output:
                    output['value'] = dict()
                output['value'] = convert_from_string(xml_dict[tag])
            elif isinstance(xml_dict[tag], dict) or \
                    isinstance(xml_dict[tag], collections.OrderedDict):
                output[tag] = convert_custom(xml_dict[tag])
            else:
                output[tag] = convert_from_string(xml_dict[tag])
    return output


def convert_to_dict(xml_dict):
    """Convert the `xmltodict` output into a dictionary that can be
    parsed into a `pcg_gazebo.parsers.types.XMLBase`.

    > *Input arguments*

    * `xml_dict` (*type:* `collections.OrderedDict`): XML content in
    dictionary form.

    > *Returns*

    `dict`: Formatted XML dictionary.
    """
    import collections
    custom_elements = ['plugin']

    tags = list(xml_dict.keys())

    output = dict()

    for tag in tags:
        if tag in custom_elements:
            output[tag] = convert_custom(xml_dict[tag])
        elif '@' in tag:
            if 'attributes' not in output:
                output['attributes'] = dict()
            output['attributes'][tag.replace(
                '@', '')] = convert_from_string(xml_dict[tag])
        elif '#' in tag:
            if 'value' not in output:
                output['value'] = dict()
            output['value'] = convert_from_string(xml_dict[tag])
        elif isinstance(xml_dict[tag], list):
            if tag not in output:
                output[tag] = list()
            for elem in xml_dict[tag]:
                subelem_dict = dict()
                subelem_dict[tag] = elem
                if isinstance(subelem_dict[tag], dict) or \
                        isinstance(subelem_dict[tag], collections.OrderedDict):
                    output[tag].append(convert_to_dict(subelem_dict[tag]))
                else:
                    output[tag].append(convert_from_string(subelem_dict[tag]))
        elif isinstance(xml_dict[tag], dict):
            output[tag] = convert_to_dict(xml_dict[tag])
        else:
            output[tag] = dict(value=convert_from_string(xml_dict[tag]))

    return output


def convert_from_string(str_input_xml):
    """Convert a string into a Python data structure type.

    > *Input arguments*

    * `str_input_xml` (*type:* `str`): Input string

    > *Returns*

    `bool`, `int`, `float`, list of `float` or `str`.
    """
    import string
    if str_input_xml is None:
        return ''
    value = None

    def is_hex(s):

        if len(s) < 2:
            return False
        if '0x' != s[0:2]:
            return False
        for e in s:
            if e not in string.hexdigits and e != 'x':
                return False
        try:
            int(s, 16)
        except BaseException:
            return False
        return len(s) % 2 == 0

    def is_numeric(s):
        import sys
        if sys.version_info[0] > 2:
            return str(s).isdigit()
        else:
            return s.isdigit()

    if is_hex(str_input_xml):
        value = int(str_input_xml, 0)
    elif isinstance(str_input_xml, list):
        value = str_input_xml
    elif is_numeric(str_input_xml):
        value = int(str_input_xml)
    elif str_input_xml in ['true', 'false', 'True', 'False']:
        value = True if str_input_xml in ['true', 'True'] else False
    elif ' ' in str_input_xml:
        # Check if this a string with whitespaces
        is_numeric = True
        for c in str_input_xml.split():
            try:
                float(c)
            except BaseException:
                is_numeric = False
                break

        if is_numeric:
            value = list()
            for item in str_input_xml.split():
                value.append(float(item))
    else:
        try:
            value = float(str_input_xml)
        except ValueError:
            value = str(str_input_xml)

    return value if value is not None else str_input_xml


def expand_nested_models(sdf):
    from ..simulation import SimulationModel

    if sdf.xml_element_name == 'model':
        if sdf.includes is None and sdf.models is None:
            return sdf
    elif sdf.xml_element_name == 'sdf':
        if sdf.models is None:
            assert sdf.models is not None, \
                'No models found in included provided SDF'
        assert len(sdf.models) == 1, 'Only one model per SDF will be parsed'
        return expand_nested_models(sdf.models[0])

    if (sdf.models is not None and len(sdf.models) > 0) or \
            (sdf.includes is not None and len(sdf.includes) > 0):
        model = SimulationModel.from_sdf(sdf).merge_nested_models()
        return model.to_sdf(type='model')
    else:
        return None


def sdf2urdf(sdf):
    """Recursively convert a SDF `pcg_gazebo` element and its child elements
    into an URDF `pcg_gazebo` element.

    > *Input arguments*

    * `sdf` (*type:* `pcg_gazebo.parsers.types.XMLBase`): Valid SDF element

    > *Returns*

    `pcg_gazebo.parsers.types.XMLBase` as an URDF element.
    """
    from .urdf import create_urdf_element
    from ..simulation.properties import Pose
    from ..path import Path
    import numpy as np
    from ..log import PCG_ROOT_LOGGER

    assert sdf is not None, 'Input SDF is invalid'
    SDF2URDF_OPT = dict(
        model='robot',
        link='link',
        joint='joint',
        inertial='inertial',
        inertia='inertia',
        mass='mass',
        visual='visual',
        collision='collision',
        box='box',
        cylinder='cylinder',
        sphere='sphere',
        mesh='mesh',
        geometry='geometry',
        material='material',
        pose='origin',
        child='child',
        parent='parent',
        limit='limit',
        dynamics='dynamics')

    assert sdf._NAME in SDF2URDF_OPT.keys(), \
        'SDF element of type <{}> not ' \
        'available for conversion to URDF'.format(sdf._NAME)

    urdf = create_urdf_element(SDF2URDF_OPT[sdf._NAME])

    if sdf._NAME == 'model':
        from copy import deepcopy
        model_sdf = expand_nested_models(deepcopy(sdf))
        # For the URDF file the frames must be positioned in
        # the joint blocks instead of the links
        if model_sdf.joints:
            # TODO Check relative link position to previous link frame
            for i in range(len(model_sdf.joints)):
                joint = model_sdf.joints[i]
                if joint.parent.value != 'world':
                    parent_link = model_sdf.get_link_by_name(
                        joint.parent.value)
                    if parent_link is None:
                        msg = 'Parent link <{}> for joint' \
                            ' <{}> does not exist!'.format(
                                joint.parent.value, joint.name)
                        PCG_ROOT_LOGGER.error(msg)
                        raise ValueError(msg)
                    if parent_link.pose is not None:
                        parent_pose = Pose(
                            parent_link.pose.value[0:3],
                            parent_link.pose.value[3::])
                    else:
                        parent_pose = Pose()
                else:
                    parent_pose = Pose()

                child_link = model_sdf.get_link_by_name(joint.child.value)
                if child_link is None:
                    msg = 'Child link <{}> for joint ' \
                        '<{}> does not exist!'.format(
                            joint.child.value, joint.name)
                    PCG_ROOT_LOGGER.error(msg)
                    raise ValueError(msg)

                if child_link.pose is not None:
                    child_pose = Pose(
                        child_link.pose.value[0:3], child_link.pose.value[3::])
                else:
                    child_pose = Pose()

                # Calculate relative pose of the joint regarding the parent's
                # pose
                pose_diff = Pose(
                    pos=np.dot(
                        parent_pose.rotation_matrix[0:3, 0:3].T,
                        child_pose.position - parent_pose.position),
                    rot=Pose.get_transform(parent_pose.quat, child_pose.quat)
                )

                model_sdf.joints[i].pose = pose_diff.to_sdf()

        urdf.name = model_sdf.name

        if model_sdf.links is not None:
            for link in model_sdf.links:
                urdf_link = sdf2urdf(link)
                urdf.add_link(urdf_link.name, urdf_link)

        if model_sdf.joints is not None:
            for joint in model_sdf.joints:
                urdf_joint = sdf2urdf(joint)
                urdf.add_joint(urdf_joint.name, urdf_joint)

        if model_sdf.urdf is not None:
            if model_sdf.urdf.links is not None:
                for link in model_sdf.urdf.links:
                    urdf.add_link(link.name, link)

            if model_sdf.urdf.transmissions is not None:
                for tr in model_sdf.urdf.transmissions:
                    urdf.add_transmission(tr.name, tr)

    elif sdf._NAME == 'mass':
        urdf.value = sdf.value
    elif sdf._NAME == 'inertia':
        urdf.ixx = sdf.ixx.value
        urdf.iyy = sdf.iyy.value
        urdf.izz = sdf.izz.value
        urdf.ixy = sdf.ixy.value
        urdf.ixz = sdf.ixz.value
        urdf.iyz = sdf.iyz.value
    elif sdf._NAME == 'box':
        urdf.size = sdf.size.value
    elif sdf._NAME == 'cylinder':
        urdf.length = sdf.length.value
        urdf.radius = sdf.radius.value
    elif sdf._NAME == 'sphere':
        urdf.radius = sdf.radius.value
    elif sdf._NAME == 'mesh':
        uri = Path(sdf.uri.value)
        if uri.is_valid:
            if uri.package_uri is not None:
                urdf.filename = uri.package_uri
            else:
                urdf.filename = uri.file_uri
        else:
            urdf.filename = sdf.uri.value
        urdf.scale = sdf.scale.value
    elif sdf._NAME == 'pose':
        urdf.xyz = sdf.value[0:3]
        urdf.rpy = sdf.value[3::]
    elif sdf._NAME == 'geometry':
        if sdf.box:
            urdf.box = sdf2urdf(sdf.box)
        elif sdf.cylinder:
            urdf.cylinder = sdf2urdf(sdf.cylinder)
        elif sdf.sphere:
            urdf.sphere = sdf2urdf(sdf.sphere)
        elif sdf.mesh:
            urdf.mesh = sdf2urdf(sdf.mesh)
    elif sdf._NAME == 'visual':
        assert sdf.geometry is not None, \
            'Visual element has no geometry, name=' + sdf.name
        urdf.name = sdf.name
        urdf.geometry = sdf2urdf(sdf.geometry)
        if sdf.pose:
            urdf.origin = sdf2urdf(sdf.pose)
        if sdf.material:
            urdf.material = sdf2urdf(sdf.material)
    elif sdf._NAME == 'collision':
        assert sdf.geometry is not None, \
            'Collision element has no geometry, name=' + sdf.name
        urdf.name = sdf.name
        urdf.geometry = sdf2urdf(sdf.geometry)
        if sdf.pose:
            urdf.origin = sdf2urdf(sdf.pose)
    elif sdf._NAME == 'inertial':
        urdf.mass = sdf2urdf(sdf.mass)
        urdf.inertia = sdf2urdf(sdf.inertia)
        if sdf.pose:
            urdf.origin = sdf2urdf(sdf.pose)
    elif sdf._NAME == 'joint':
        urdf.name = sdf.name
        urdf.type = sdf.type
        urdf.parent = sdf2urdf(sdf.parent)
        urdf.child = sdf2urdf(sdf.child)
        if sdf.axis:
            if sdf.axis is not None:
                urdf.axis = create_urdf_element('axis')
                urdf.axis.xyz = sdf.axis.xyz.value
            if sdf.axis.limit:
                urdf.limit = sdf2urdf(sdf.axis.limit)
            if sdf.axis.dynamics:
                urdf.dynamics = sdf2urdf(sdf.axis.dynamics)
        if sdf.pose:
            urdf.origin = sdf2urdf(sdf.pose)

        if sdf.urdf is not None:
            if sdf.urdf.mimic is not None:
                urdf.mimic = sdf.urdf.mimic
            if sdf.urdf.safety_controller is not None:
                urdf.safety_controller = sdf.urdf.safety_controller
    elif sdf._NAME == 'parent':
        urdf.link = sdf.value
    elif sdf._NAME == 'child':
        urdf.link = sdf.value
    elif sdf._NAME == 'limit':
        if sdf.lower:
            urdf.lower = sdf.lower.value
        if sdf.upper:
            urdf.upper = sdf.upper.value
        if sdf.effort:
            urdf.effort = sdf.effort.value
        if sdf.velocity:
            urdf.velocity = sdf.velocity.value
    elif sdf._NAME == 'dynamics':
        if sdf.damping:
            urdf.damping = sdf.damping.value
        if sdf.friction:
            urdf.friction = sdf.friction.value
    elif sdf._NAME == 'material':
        if sdf.ambient:
            urdf.color = create_urdf_element('color')
            urdf.color.rgba = sdf.diffuse.value
        elif sdf.shader:
            urdf.texture.filename = sdf.shader.normal_map
    elif sdf._NAME == 'link':
        urdf.name = sdf.name
        if sdf.inertial:
            urdf.inertial = sdf2urdf(sdf.inertial)

        if sdf.visuals is not None:
            for visual in sdf.visuals:
                urdf_visual = sdf2urdf(visual)
                urdf.add_visual(urdf_visual.name, urdf_visual)

        if sdf.collisions is not None:
            for collision in sdf.collisions:
                urdf_collision = sdf2urdf(collision)
                urdf.add_collision(urdf_collision.name, urdf_collision)

    return urdf


def urdf2sdf(urdf):
    """Recursively convert an URDF `pcg_gazebo` element and its child elements
    into a SDF `pcg_gazebo` element.

    > *Input arguments*

    * `urdf` (*type:* `pcg_gazebo.parsers.types.XMLBase`): Valid URDF element

    > *Returns*

    `pcg_gazebo.parsers.types.XMLBase` as a SDF element.
    """
    from .sdf import create_sdf_element
    from ..simulation.properties import Pose
    from ..path import Path
    import collections
    from ..log import PCG_ROOT_LOGGER

    assert urdf is not None, 'Input URDF is invalid'
    URDF2SDF_OPT = dict(
        robot='model',
        link='link',
        joint='joint',
        visual='visual',
        collision='collision',
        inertia='inertia',
        inertial='inertial',
        mass='mass',
        origin='pose',
        box='box',
        cylinder='cylinder',
        sphere='sphere',
        mesh='mesh',
        geometry='geometry',
        material='material',
        pose='origin',
        child='child',
        parent='parent',
        limit='limit',
        dynamics='dynamics'
    )

    assert urdf._NAME in URDF2SDF_OPT.keys(), \
        'URDF element of type <{}> not available' \
        ' for conversion to SDF'.format(urdf._NAME)

    sdf = create_sdf_element(URDF2SDF_OPT[urdf._NAME])

    # Parse all gazebo elements
    if urdf._NAME == 'robot':
        if urdf.gazebos is not None:
            for gazebo in urdf.gazebos:
                if gazebo.reference is not None:
                    # In case the Gazebo block is
                    # referenced to a link or joint,
                    # copy child elements to the
                    # respective link or joint
                    if urdf.get_link_by_name(gazebo.reference):
                        for link in urdf.links:
                            if link.name == gazebo.reference:
                                link.gazebo = gazebo
                                break
                    if urdf.get_joint_by_name(gazebo.reference):
                        for joint in urdf.joints:
                            if joint.name == gazebo.reference:
                                joint.gazebo = gazebo
                                break
                else:
                    # Add all model specific Gazebo elements
                    for tag in gazebo.children:
                        if isinstance(
                                gazebo.children[tag],
                                collections.Iterable):
                            for elem in gazebo.children[tag]:
                                if elem._TYPE != 'sdf':
                                    continue
                                sdf._add_child_element(elem._NAME, elem)
                        else:
                            if gazebo.children[tag]._TYPE != 'sdf':
                                continue
                            sdf._add_child_element(tag, gazebo.children[tag])

    # Set all Gazebo specific elements for this SDF element
    # in case the element can parse Gazebo elements
    if hasattr(urdf, 'gazebo'):
        if urdf.gazebo is not None:
            for tag in urdf.gazebo.children:
                if isinstance(urdf.gazebo.children[tag], collections.Iterable):
                    for elem in urdf.gazebo.children[tag]:
                        if elem._TYPE != 'sdf':
                            continue
                        try:
                            sdf._add_child_element(elem._NAME, elem)
                        except AssertionError as ex:
                            PCG_ROOT_LOGGER.error(
                                '[urdf2sdf] Cannot add element <{}> of '
                                'format <{}> to <{}>, message={}'.format(
                                    elem.xml_element_name,
                                    elem.xml_format,
                                    sdf.xml_element_name,
                                    str(ex)))
                else:
                    if urdf.gazebo.children[tag]._TYPE != 'sdf':
                        continue
                    sdf._add_child_element(tag, urdf.gazebo.children[tag])

    if urdf._NAME == 'mass':
        sdf.value = urdf.value
    elif urdf._NAME == 'inertia':
        sdf.ixx.value = urdf.ixx
        sdf.iyy.value = urdf.iyy
        sdf.izz.value = urdf.izz
        sdf.ixy.value = urdf.ixy
        sdf.ixz.value = urdf.ixz
        sdf.iyz.value = urdf.iyz
    elif urdf._NAME == 'box':
        sdf.size.value = urdf.size
    elif urdf._NAME == 'cylinder':
        sdf.length.value = urdf.length
        sdf.radius.value = urdf.radius
    elif urdf._NAME == 'sphere':
        sdf.radius.value = urdf.radius
    elif urdf._NAME == 'mesh':
        uri = Path(urdf.filename)
        if uri.is_valid:
            if uri.model_uri is not None:
                sdf.uri = uri.model_uri
            else:
                sdf.uri = uri.file_uri
        else:
            sdf.uri = urdf.filename
        sdf.scale = urdf.scale
    elif urdf._NAME == 'origin':
        sdf.value = urdf.xyz + urdf.rpy
    elif urdf._NAME == 'geometry':
        if urdf.box is not None:
            sdf.box = urdf2sdf(urdf.box)
        elif urdf.cylinder is not None:
            sdf.cylinder = urdf2sdf(urdf.cylinder)
        elif urdf.sphere is not None:
            sdf.sphere = urdf2sdf(urdf.sphere)
        elif urdf.mesh is not None:
            sdf.mesh = urdf2sdf(urdf.mesh)
    elif urdf._NAME == 'visual':
        assert urdf.geometry is not None, \
            'Visual element has no geometry, name=' + urdf.name
        sdf.name = urdf.name
        sdf.geometry = urdf2sdf(urdf.geometry)
        if urdf.origin:
            sdf.pose = urdf2sdf(urdf.origin)
        if urdf.material:
            sdf.material = urdf2sdf(urdf.material)
    elif urdf._NAME == 'collision':
        assert urdf.geometry is not None, \
            'Collision element has no geometry, name=' + urdf.name
        sdf.name = urdf.name
        sdf.geometry = urdf2sdf(urdf.geometry)
        if urdf.origin:
            sdf.pose = urdf2sdf(urdf.origin)
    elif urdf._NAME == 'inertial':
        sdf.mass = urdf2sdf(urdf.mass)
        sdf.inertia = urdf2sdf(urdf.inertia)
        if urdf.origin is not None:
            sdf.pose = urdf2sdf(urdf.origin)
    elif urdf._NAME == 'joint':
        sdf.name = urdf.name

        if urdf.type == 'continuous':
            sdf.type = 'revolute'
            if not sdf.axis:
                sdf.axis = create_sdf_element('axis')
            sdf.axis.limit = create_sdf_element('limit')
            sdf.axis.limit.lower = -1e16
            sdf.axis.limit.upper = 1e16
        else:
            sdf.type = urdf.type
        sdf.parent = urdf2sdf(urdf.parent)
        sdf.child = urdf2sdf(urdf.child)
        if urdf.axis:
            if not sdf.axis:
                sdf.axis = create_sdf_element('axis')
            sdf.axis.xyz = urdf.axis.xyz
        if urdf.limit and urdf.type != 'continuous':
            if not sdf.axis:
                sdf.axis = create_sdf_element('axis')
            sdf.axis.limit = urdf2sdf(urdf.limit)
        if urdf.dynamics:
            if not sdf.axis:
                sdf.axis = create_sdf_element('axis')
            sdf.axis.dynamics = urdf2sdf(urdf.dynamics)
        if urdf.origin:
            sdf.pose = urdf2sdf(urdf.origin)
    elif urdf._NAME == 'parent':
        sdf.value = urdf.link
    elif urdf._NAME == 'child':
        sdf.value = urdf.link
    elif urdf._NAME == 'limit':
        sdf.lower = urdf.lower
        sdf.upper = urdf.upper
        sdf.velocity = urdf.velocity
        sdf.effort = urdf.effort
    elif urdf._NAME == 'dynamics':
        sdf.damping = urdf.damping
        sdf.friction = urdf.friction
    elif urdf._NAME == 'material':
        if urdf.color:
            sdf.ambient = urdf.color.rgba
            sdf.diffuse = urdf.color.rgba
        elif urdf.texture:
            sdf.shader = create_sdf_element('shader')
            sdf.shader.normal_map = urdf.texture.filename
            sdf.type = 'pixel'
    elif urdf._NAME == 'link':
        sdf.name = urdf.name
        if urdf.inertial:
            sdf.inertial = urdf2sdf(urdf.inertial)

        if urdf.gazebo is not None:
            if urdf.gazebo.selfCollide:
                sdf.self_collide = urdf.gazebo.selfCollide.value

        if urdf.visuals is not None:
            for visual in urdf.visuals:
                sdf_visual = urdf2sdf(visual)
                if urdf.gazebo is not None:
                    if urdf.gazebo.material:
                        sdf_visual.material = \
                            urdf.gazebo.material.to_sdf()
                sdf.add_visual(sdf_visual.name, sdf_visual)

        if urdf.collisions is not None:
            for collision in urdf.collisions:
                sdf_collision = urdf2sdf(collision)

                if urdf.gazebo is not None:
                    if urdf.gazebo.maxContacts:
                        sdf_collision.max_contacts = \
                            urdf.gazebo.maxContacts.value
                    if urdf.gazebo.mu1:
                        if not sdf_collision.surface:
                            sdf_collision.surface = create_sdf_element(
                                'surface')
                        if not sdf_collision.surface.friction:
                            sdf_collision.surface.friction = \
                                create_sdf_element(
                                    'friction')
                            sdf_collision.surface.friction.reset(
                                mode='surface',
                                with_optional_elements=True)

                        sdf_collision.surface.friction.ode.mu = \
                            urdf.gazebo.mu1.value
                        sdf_collision.surface.friction.bullet.friction = \
                            urdf.gazebo.mu1.value
                    if urdf.gazebo.mu2:
                        if not sdf_collision.surface:
                            sdf_collision.surface = create_sdf_element(
                                'surface')
                        if not sdf_collision.surface.friction:
                            sdf_collision.surface.friction = \
                                create_sdf_element(
                                    'friction')
                            sdf_collision.surface.friction.reset(
                                mode='surface',
                                with_optional_elements=True)

                        sdf_collision.surface.friction.ode.mu2 = \
                            urdf.gazebo.mu2.value
                        sdf_collision.surface.friction.bullet.friction2 = \
                            urdf.gazebo.mu2.value
                    if urdf.gazebo.kp:
                        if not sdf_collision.surface:
                            sdf_collision.surface = create_sdf_element(
                                'surface')
                        if not sdf_collision.surface.contact:
                            sdf_collision.surface.contact = create_sdf_element(
                                'contact')
                            sdf_collision.surface.contact.reset(
                                mode='collision', with_optional_elements=True)

                        sdf_collision.surface.contact.ode.kp = \
                            urdf.gazebo.kp.value
                        sdf_collision.surface.contact.bullet.kp = \
                            urdf.gazebo.kp.value
                    if urdf.gazebo.kd:
                        if not sdf_collision.surface:
                            sdf_collision.surface = create_sdf_element(
                                'surface')
                        if not sdf_collision.surface.contact:
                            sdf_collision.surface.contact = create_sdf_element(
                                'contact')
                            sdf_collision.surface.contact.reset(
                                mode='collision', with_optional_elements=True)

                        sdf_collision.surface.contact.ode.kd = \
                            urdf.gazebo.kd.value
                        sdf_collision.surface.contact.bullet.kd = \
                            urdf.gazebo.kd.value
                    if urdf.gazebo.minDepth:
                        if not sdf_collision.surface:
                            sdf_collision.surface = create_sdf_element(
                                'surface')
                        if not sdf_collision.surface.contact:
                            sdf_collision.surface.contact = create_sdf_element(
                                'contact')
                            sdf_collision.surface.contact.reset(
                                mode='collision', with_optional_elements=True)

                        sdf_collision.surface.contact.ode.min_depth = \
                            urdf.gazebo.minDepth.value
                    if urdf.gazebo.maxVel:
                        if not sdf_collision.surface:
                            sdf_collision.surface = create_sdf_element(
                                'surface')
                        if not sdf_collision.surface.contact:
                            sdf_collision.surface.contact = create_sdf_element(
                                'contact')
                            sdf_collision.surface.contact.reset(
                                mode='collision', with_optional_elements=True)

                        sdf_collision.surface.contact.ode.max_vel = \
                            urdf.gazebo.maxVel.value

                sdf.add_collision(sdf_collision.name, sdf_collision)
    elif urdf._NAME == 'robot':
        import networkx
        sdf.name = urdf.name

        # Create a graph to find the connections between
        # links and compute the relative pose between them
        robot_graph = networkx.DiGraph()

        if urdf.links is not None:
            for link in urdf.links:
                # Add link as a node
                robot_graph.add_node(link.name)

        if urdf.joints is not None:
            for joint in urdf.joints:
                # Add connection between links as an edge
                robot_graph.add_edge(
                    joint.parent.link, joint.child.link, label=joint.name)

        if urdf.links is not None and len(urdf.links) > 1:
            # Test if the robot graph is connected
            for n in robot_graph.nodes():
                if robot_graph.out_degree(n) + robot_graph.in_degree(n) == 0:
                    raise ValueError(
                        'Link <{}> is not connected by any joint'.format(n))

            start_nodes = list()
            end_nodes = list()
            for n in robot_graph.nodes():
                if robot_graph.out_degree(n) > 0 and \
                        robot_graph.in_degree(n) == 0:
                    start_nodes.append(n)
                if robot_graph.out_degree(n) == 0 and \
                        robot_graph.in_degree(n) > 0:
                    end_nodes.append(n)

            assert len(start_nodes) == 1, \
                'The URDF structure should have only one start node' \
                ', n_start_nodes={}'.format(len(start_nodes))

        if urdf.links is not None:
            for link in urdf.links:
                sdf_link = urdf2sdf(link)
                sdf.add_link(sdf_link.name, sdf_link)

        if urdf.joints is not None:
            for joint in urdf.joints:
                sdf_joint = urdf2sdf(joint)
                sdf_joint.rm_child('pose')
                sdf.add_joint(sdf_joint.name, sdf_joint)

            # Find the paths from start node to end nodes and
            # compute the link poses in between
            for e in end_nodes:
                for path in networkx.all_simple_paths(
                        robot_graph, source=start_nodes[0], target=e):
                    for i in range(len(path)):
                        if path[i] == start_nodes[0]:
                            continue
                        for joint in urdf.joints:
                            if joint.parent.link == path[i - 1] and \
                                    joint.child.link == path[i]:
                                parent_link = sdf.get_link_by_name(
                                    joint.parent.link)
                                child_link = sdf.get_link_by_name(
                                    joint.child.link)
                                if parent_link.pose is not None:
                                    pose = Pose.from_sdf(
                                        parent_link.pose) + \
                                        Pose.from_urdf(joint.origin)
                                else:
                                    pose = Pose.from_urdf(joint.origin)
                                child_link.pose = pose.to_sdf()

        sdf = merge_massless_links(sdf)
    return sdf


def merge_massless_links(sdf):
    from copy import deepcopy
    from ..log import PCG_ROOT_LOGGER
    if sdf.joints is None:
        return sdf

    output_sdf = deepcopy(sdf)
    while output_sdf.has_massless_links():
        if output_sdf.joints is not None:
            merged_link = None
            joint_name = None
            for joint in output_sdf.joints:
                if joint.type == 'fixed':
                    parent_link = output_sdf.get_link_by_name(
                        joint.parent.value)
                    child_link = output_sdf.get_link_by_name(joint.child.value)

                    if parent_link.inertial is None or \
                            child_link.inertial is None:
                        merged_link = merge_links(parent_link, child_link)
                        joint_name = joint.name
                        break
            if joint_name is None:
                PCG_ROOT_LOGGER.info(
                    'No fixed joints found for this model <{}>'.format(
                        sdf.name))
                break

            joint = output_sdf.get_joint_by_name(joint_name)

            parent_name = joint.parent.value
            child_name = joint.child.value

            # Remove joint
            output_sdf.remove_joint_by_name(joint.name)

            # Find all the links connected to the now merged link
            for joint in output_sdf.joints:
                if joint.parent.value == child_name:
                    joint.parent = parent_name

            # Remove link
            output_sdf.remove_link_by_name(child_name)

            # Replace old parent link with merged link
            for i in range(len(output_sdf.links)):
                if output_sdf.links[i].name == parent_name:
                    output_sdf.links[i] = merged_link
                    break

            # Refactor plugin inputs with the old link's name
            if output_sdf.plugins is not None:
                for i in range(len(output_sdf.plugins)):
                    output_sdf.plugins[i].replace_parameter_value(
                        child_name, parent_name)

    return output_sdf


def merge_links(parent, child):
    from copy import deepcopy
    from ..simulation.properties import Pose
    from ..log import PCG_ROOT_LOGGER

    if parent.inertial is not None and \
            child.inertial is not None:
        return None

    if parent.inertial is not None and \
            child.inertial is None:
        new_link = deepcopy(parent)
        merged_link = deepcopy(child)
    elif child.inertial is not None and \
            parent.inertial is None:
        new_link = deepcopy(child)
        merged_link = deepcopy(parent)
    else:
        PCG_ROOT_LOGGER.warning(
            'Links cannot be merged, parent={}, child={}'.format(
                parent.name, child.name))
        return None

    PCG_ROOT_LOGGER.info('Merging link <{}> into link <{}>'.format(
        merged_link.name, new_link.name))

    def compute_new_pose(ml, obj, nl):
        if ml.pose is None:
            ml_pose = parse_sdf('<pose>0 0 0 0 0 0</pose>')
        else:
            ml_pose = ml.pose
        if nl.pose is None:
            nl_pose = parse_sdf('<pose>0 0 0 0 0 0</pose>')
        else:
            nl_pose = nl.pose
        pose = Pose.from_sdf(ml_pose) - Pose.from_sdf(nl_pose)

        if obj.pose is not None:
            pose = pose + Pose.from_sdf(obj.pose)

        return pose

    visual_name_refactors = dict()
    if merged_link.visuals is not None:
        PCG_ROOT_LOGGER.info('Merged link <{}> has {} visual blocks'.format(
            merged_link.name, len(merged_link.visuals)))
        for visual in merged_link.visuals:
            pose = compute_new_pose(merged_link, visual, new_link)

            new_visual_name = '{}_{}'.format(merged_link.name, visual.name)
            visual_name_refactors[visual.name] = new_visual_name
            new_link.add_visual(
                name=new_visual_name,
                visual=visual)
            new_link.visuals[-1].pose = pose.to_sdf()
            PCG_ROOT_LOGGER.info(
                'Adding visual <{}> to parent link <{}>'.format(
                    new_link.visuals[-1].name, new_link.name))
    else:
        PCG_ROOT_LOGGER.info(
            'Merged link <{}> has no visual blocks'.format(
                merged_link.name))

    collision_name_refactors = dict()
    if merged_link.collisions is not None:
        PCG_ROOT_LOGGER.info('Merged link <{}> has {} collision blocks'.format(
            merged_link.name, len(merged_link.collisions)))
        for collision in merged_link.collisions:
            pose = compute_new_pose(merged_link, collision, new_link)

            new_collision_name = '{}_{}'.format(
                merged_link.name, collision.name)
            collision_name_refactors[collision.name] = new_collision_name
            new_link.add_collision(
                name=new_collision_name,
                collision=collision)
            new_link.collisions[-1].pose = pose.to_sdf()
            PCG_ROOT_LOGGER.info(
                'Adding collision <{}> to parent'
                ' link <{}> from merged link <{}>'.format(
                    new_link.collisions[-1].name,
                    parent.name,
                    merged_link.name))
    else:
        PCG_ROOT_LOGGER.info(
            'Merged link <{}> has no collision blocks'.format(
                merged_link.name))

    if merged_link.sensors is not None:
        PCG_ROOT_LOGGER.info('Merged link <{}> has {} sensor blocks'.format(
            merged_link.name, len(merged_link.sensors)))
        for sensor in merged_link.sensors:
            pose = compute_new_pose(merged_link, sensor, new_link)

            new_link.add_sensor(
                name='{}_{}'.format(merged_link.name, sensor.name),
                sensor=sensor)
            new_link.sensors[-1].pose = pose.to_sdf()

            # Refactor sensors for changes in visual and collision elements
            if new_link.sensors[-1].type == 'contact':
                old_collision_name = \
                    new_link.sensors[-1].contact.collision.value
                try:
                    # Replacing the name of the collision body in the sensor
                    # if the collision body name changed
                    if old_collision_name in collision_name_refactors:
                        new_link.sensors[-1].contact.collision.value = \
                            collision_name_refactors[old_collision_name]
                except AttributeError as ex:
                    PCG_ROOT_LOGGER.error(
                        'Could not refactor collision body name <{}>, '
                        'message={}'.format(old_collision_name, str(ex)))
            PCG_ROOT_LOGGER.info(
                'Adding sensor <{}> to parent'
                ' link <{}> from merged link <{}>'.format(
                    new_link.sensors[-1].name,
                    parent.name,
                    merged_link.name))
    else:
        PCG_ROOT_LOGGER.info(
            'Merged link <{}> has no sensor blocks'.format(
                merged_link.name))

    if merged_link.plugins is not None:
        PCG_ROOT_LOGGER.info(
            'Merged link <{}> has {}'
            ' plugin blocks'.format(
                merged_link.name, len(merged_link.plugins)))
        for plugin in merged_link.plugins:
            new_link.add_plugin(plugin.name, plugin)
            PCG_ROOT_LOGGER.info(
                'Adding plugin <{}> to parent link'
                ' <{}> from child link <{}>'.format(
                    plugin.name, new_link.name, merged_link.name))
    else:
        PCG_ROOT_LOGGER.info(
            'Merged link <{}> has no plugin blocks'.format(
                merged_link.name))

    # If the child link is the one with inertial tensor, handle the name
    # of the new link
    if child.inertial is not None and parent.inertial is None:
        new_link.name = parent.name
        if new_link.plugins is not None:
            for plugin in new_link.plugins:
                for tag in plugin.find_values(child.name):
                    plugin.value[tag] = parent.name

    return new_link
