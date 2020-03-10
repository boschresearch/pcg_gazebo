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
"""
Useful functions for Jinja file processing and YAML file parser extensions.
"""
from __future__ import print_function
import random
import string
import os
import re
import yaml
import rospkg
from jinja2 import FileSystemLoader, Environment, \
    BaseLoader, TemplateNotFound
from .log import PCG_ROOT_LOGGER

PCG_RESOURCES_ROOT_DIR = os.path.join(os.path.expanduser('~'), '.pcg')
PCG_ROOT_FOLDER = os.path.dirname(os.path.abspath(__file__))
PCG_TEMPLATE_FOLDER = os.path.join(
    PCG_ROOT_FOLDER,
    'templates')


class _PCGYAMLLoader(yaml.SafeLoader, object):
    # MIT License
    #
    # Copyright (c) 2018 Josh Bode
    #
    # Permission is hereby granted, free of charge, to any person
    # obtaining a copy of this software and associated documentation
    # files (the "Software"), to deal in the Software without restriction,
    # including without limitation the rights to use, copy, modify, merge,
    # publish, distribute, sublicense, and/or sell copies of the Software, and
    # to permit persons to whom the Software is furnished to do so, subject
    # to the following conditions:
    #
    # The above copyright notice and this permission notice shall be included
    # in all copies or substantial portions of the Software.

    # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
    # OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    # MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    # IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
    # CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
    # TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
    # SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
    """
    YAML Loader with `!include` parser to find and resolve
    absolute paths.
    """

    def __init__(self, stream):
        """Initialise Loader."""

        try:
            self._root = os.path.split(stream.name)[0]
        except AttributeError:
            self._root = os.path.curdir

        super(_PCGYAMLLoader, self).__init__(stream)


def yaml_include_constructor(loader, node):
    # MIT License
    #
    # Copyright (c) 2018 Josh Bode
    #
    # Permission is hereby granted, free of charge, to any person
    # obtaining a copy of this software and associated documentation
    # files (the "Software"), to deal in the Software without restriction,
    # including without limitation the rights to use, copy, modify, merge,
    # publish, distribute, sublicense, and/or sell copies of the Software, and
    # to permit persons to whom the Software is furnished to do so, subject
    # to the following conditions:
    #
    # The above copyright notice and this permission notice shall be included
    # in all copies or substantial portions of the Software.

    # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
    # OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    # MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    # IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
    # CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
    # TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
    # SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
    filename = os.path.abspath(
        os.path.join(
            loader._root,
            loader.construct_scalar(node)))

    with open(filename, 'r') as f:
        return yaml.load(f, _PCGYAMLLoader)


def yaml_find_ros_package(loader, node):
    """Parser for YAML processor to resolve ROS package paths
    using the `!find` function.
    """

    import rospkg
    input_str = loader.construct_scalar(node)
    assert '/' in input_str, \
        'ROS package to be searched must be provided' \
        ' as <ros_package/<path_to_file>'
    ros_package = input_str.split('/')[0]

    finder = rospkg.RosPack()
    assert ros_package in finder.list(), \
        'Could not find ROS package {}'.format(ros_package)

    ros_package_path = finder.get_path(ros_package)
    filename = input_str.replace(ros_package, ros_package_path)

    assert os.path.isfile(filename), 'Invalid filename={}'.format(filename)

    with open(filename, 'r') as f:
        return yaml.load(f, _PCGYAMLLoader)


def yaml_resolve_path(loader, node):
    filename = os.path.abspath(os.path.join(
        loader._root,
        loader.construct_scalar(node)))
    assert os.path.exists(filename), \
        'File does not exist, filename={}'.format(filename)
    return filename


yaml.add_constructor('!include', yaml_include_constructor, _PCGYAMLLoader)
yaml.add_constructor('!find', yaml_find_ros_package, _PCGYAMLLoader)
yaml.add_constructor('!file', yaml_resolve_path, _PCGYAMLLoader)


def load_yaml(input_yaml):
    """Load YAML file using internal path resolvers.

    > *Input arguments*

    * `input_yaml` (*type:* `str`): Filename or file content as string.

    > *Returns*

    `dict` parsed from the YAML file.
    """
    if os.path.isfile(input_yaml):
        extension = os.path.splitext(input_yaml)[1].lstrip('.')
        assert extension in ['yaml', 'yml'], \
            'Invalid YAML file extension, ' \
            'expected=yaml or yml, received={}'.format(
                extension)
        with open(input_yaml, 'r') as f:
            data = yaml.load(f, _PCGYAMLLoader)
        return data
    elif isinstance(input_yaml, str):
        return yaml.load(input_yaml, _PCGYAMLLoader)


class _AbsFileSystemLoader(BaseLoader):
    def __init__(self, path):
        self.path = path

    def get_source(self, environment, template):
        if os.path.isfile(template):
            path = template
        else:
            path = os.path.join(self.path, template)
            if not os.path.isfile(path):
                raise TemplateNotFound(template)
        mtime = os.path.getmtime(path)
        with open(path) as f:
            source = f.read()
            if not isinstance(source, str):
                source = source.decode('utf-8')
        return source, path, lambda: mtime == os.path.getmtime(path)


def _find_ros_package(pkg_name):
    try:
        pkg_path = rospkg.RosPack().get_path(pkg_name)
    except rospkg.ResourceNotFound as ex:
        PCG_ROOT_LOGGER.error(
            'Error finding package {}, message={}'.format(
                pkg_name, ex))
        return None
    return pkg_path


def _find_sdf_template(name):
    if '.sdf.jinja' not in name:
        filename = '{}.sdf.jinja'.format(name)
    else:
        filename = name
    return get_template_path(filename)


def _find_relative_path(name, root_dir='.'):
    full_path = os.path.abspath(os.path.join(root_dir, name))
    if os.path.exists(full_path):
        return os.path.abspath(full_path)
    else:
        PCG_ROOT_LOGGER.error(
            'Jinja processor [find_relative_file]: '
            'Cannot find file <{}>'.format(
                name))
        return None


def _pretty_print_xml(xml):
    import lxml.etree as etree
    parser = etree.XMLParser(remove_blank_text=True)
    root = etree.fromstring(xml, parser=parser)
    return etree.tostring(root, pretty_print=True).decode()


def _parse_package_paths(xml):
    # Finding patterns package://
    result = re.findall(r'package://\w+/', xml)
    output_xml = xml
    for item in result:
        pkg_name = item.replace('package://', '').replace('/', '')
        try:
            pkg_path = rospkg.RosPack().get_path(pkg_name)
        except rospkg.ResourceNotFound as ex:
            PCG_ROOT_LOGGER.error(
                'Error finding package {}, message={}'.format(
                    pkg_name, ex))
            return None

        output_xml = output_xml.replace(item, 'file://' + pkg_path + '/')

    # Finding patterns $(find pkg)
    result = re.findall(r'\$\(find \w+\)', output_xml)
    for item in result:
        pkg_name = item.split()[1].replace(')', '')
        try:
            pkg_path = rospkg.RosPack().get_path(pkg_name)
        except rospkg.ResourceNotFound as ex:
            PCG_ROOT_LOGGER.error(
                'Error finding package {}, message={}'.format(
                    pkg_name, ex))
            return None

        output_xml = output_xml.replace(item, 'file://' + pkg_path + '/')
    return output_xml


def process_jinja_template(template, parameters=None, include_dir=None):
    from .path import Path

    if not isinstance(parameters, dict):
        parameters = dict()
        PCG_ROOT_LOGGER.warning(
            'Input parameters to be replaced in the template'
            ' must be provided as a dictionary, received {}'
            ' instead'.format(type(parameters)))

    if not isinstance(template, str):
        PCG_ROOT_LOGGER.error(
            'The template input must a string, either with the'
            ' template text content or filename to the template')
        return None

    PCG_ROOT_LOGGER.info('Input template: {}'.format(template))
    base_loader = None
    if template.startswith('${PCG}/') or \
            template.startswith('$PCG/') or \
            template.startswith('$(PCG)/'):
        if template.startswith('${PCG}/'):
            template = template.replace('${PCG}/', '')
        elif template.startswith('$PCG/'):
            template = template.replace('$PCG/', '')
        else:
            template = template.replace('$(PCG)/', '')

        if os.path.exists(os.path.join(PCG_TEMPLATE_FOLDER, template)):
            template = os.path.join(PCG_TEMPLATE_FOLDER, template)
            base_loader = FileSystemLoader(PCG_TEMPLATE_FOLDER)
            templates_dir = PCG_TEMPLATE_FOLDER
        else:
            PCG_ROOT_LOGGER.error(
                'Input template {} not found in the default '
                'templates folder {}'.format(template, PCG_TEMPLATE_FOLDER))
            return None
    if base_loader is None:
        filename_path = Path(template)
        if filename_path.is_valid:
            templates_dir = os.path.dirname(filename_path.absolute_uri)
            PCG_ROOT_LOGGER.info('Input template is a file, {}'.format(
                filename_path.absolute_uri))
            base_loader = FileSystemLoader(templates_dir)
            template = filename_path.absolute_uri
        else:
            base_loader = FileSystemLoader('.')
            templates_dir = '.'

    if isinstance(include_dir, str):
        if not os.path.isdir(include_dir):
            PCG_ROOT_LOGGER.error(
                'Include directory in invalid, dir={}'.format(include_dir))
            return None
    else:
        include_dir = templates_dir

    includes_loader = _AbsFileSystemLoader(include_dir)

    base_env = Environment(loader=base_loader)
    # Add Jinja function similar to $(find <package>) in XACRO
    base_env.filters['find_ros_package'] = _find_ros_package
    base_env.filters['find_sdf_template'] = _find_sdf_template
    base_env.filters['find_relative_path'] = lambda p: _find_relative_path(
        p, templates_dir)

    if os.path.isfile(template):
        PCG_ROOT_LOGGER.info(
            'Retrieving template to be rendered from file {}'.format(template))
        model_template = base_env.get_template(os.path.basename(template))
    else:
        model_template = base_env.from_string(template)
    model_template.environment.loader = includes_loader

    output_xml = _parse_package_paths(model_template.render(**parameters))
    try:
        return _pretty_print_xml(output_xml)
    except BaseException:
        return output_xml


def generate_random_string(size=3):
    return ''.join(random.choice(string.ascii_letters) for i in range(size))


def get_template_path(filename):
    template_fullpath = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'templates',
        filename)
    if os.path.isfile(template_fullpath):
        return template_fullpath
    else:
        return None


def is_string(obj):
    """Test if input object is a string.

    > *Input arguments*

    * `obj`: Input variable.

    > *Returns*

    `True`, if `obj` is a string.
    """
    import sys
    if sys.version_info.major == 2:
        return isinstance(obj, (str, unicode))  # noqa: F821
    else:
        return isinstance(obj, str)


def is_scalar(obj):
    """Test if input object is a scalar.

    > *Input arguments*

    * `obj`: Input variable.

    > *Returns*

    `True`, if `obj` is a scalar.
    """
    import numpy as np
    return isinstance(obj, float) or \
        isinstance(obj, int) or \
        isinstance(obj, np.float64) or \
        isinstance(obj, np.int64)


def is_integer(obj):
    """Test if input object is an integer.

    > *Input arguments*

    * `obj`: Input variable.

    > *Returns*

    `True`, if `obj` is a integer.
    """
    import numpy as np
    return isinstance(obj, int) or \
        isinstance(obj, np.int64)


def is_boolean(obj):
    """Test if input object is a boolean.

    > *Input arguments*

    * `obj`: Input variable.

    > *Returns*

    `True`, if `obj` is a boolean.
    """
    return isinstance(obj, bool) or obj in [0, 1]


def is_array(obj):
    """Test if input object is a numerical vector.

    > *Input arguments*

    * `obj`: Input variable.

    > *Returns*

    `True`, if `obj` is a vector.
    """
    import collections
    return isinstance(
        obj, collections.Iterable) and not isinstance(
        obj, str)


def get_random_point_from_shape(geo):
    from shapely.geometry import Point
    min_x, min_y, max_x, max_y = geo.bounds
    pnt = [
        random.uniform(min_x, max_x),
        random.uniform(min_y, max_y)
    ]
    while not geo.contains(Point(pnt)):
        pnt = [
            random.uniform(min_x, max_x),
            random.uniform(min_y, max_y)
        ]
    return pnt


def has_string_pattern(input_str, pattern):
    if '*' not in pattern:
        if input_str == pattern:
            return True
    if pattern.startswith('*') and not pattern.endswith('*'):
        suffix = pattern.replace('*', '')
        if input_str.endswith(suffix):
            return True
    if pattern.endswith('*') and not pattern.startswith('*'):
        prefix = pattern.replace('*', '')
        if input_str.startswith(prefix):
            return True
    if pattern.startswith('*') and pattern.endswith('*'):
        pattern = pattern.replace('*', '')
        if pattern in input_str:
            return True
    return False
