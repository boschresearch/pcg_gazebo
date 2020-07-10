#!/usr/bin/env python
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
import sys
import json
import unittest
import subprocess
import matplotlib.pyplot as plt
from pcg_gazebo.utils import set_resources_root_dir


set_resources_root_dir(os.path.join('/tmp', '.pcg'))


def load_notebook(file_obj):
    # The MIT License (MIT)
    #
    # Copyright (c) 2019 Michael Dawson-Haggerty
    #
    # Permission is hereby granted, free of charge, to any person
    # obtaining a copy of this software and associated documentation
    # files (the "Software"), to deal in the Software without restriction,
    # including without limitation the rights to use, copy, modify, merge,
    # publish, distribute, sublicense, and/or sell copies of the Software,
    # and to permit persons to whom the Software is furnished to do so,
    # subject to the following conditions:
    #
    # The above copyright notice and this permission notice shall be
    # included in all copies or substantial portions of the Software.
    #
    # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    # EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    # OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    # IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
    # CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
    # TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
    # SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
    """
    Load an ipynb file into a cleaned and stripped string that can
    be ran with `exec`

    The motivation for this is to check ipynb examples with CI so
    they don't get silently broken and confusing.

    Arguments
    ----------
    file_obj :  open file object

    Returns
    ----------
    script : str
      Cleaned script which can be passed to exec
    """
    raw = json.load(file_obj)
    lines = list()
    for cell in raw['cells']:
        if cell['cell_type'] == 'code':
            for line in cell['source']:
                if '#' == line[0]:
                    continue
                lines.append(line)
    script = exclude_calls(lines)
    return script


def exclude_calls(
        lines,
        exclude=['%matplotlib',
                 '%pylab',
                 'show',
                 'plt',
                 'save_image',
                 'run_all_tasks',
                 'kill_all_tasks',
                 'spawn',
                 'spawn_model',
                 'wait',
                 'apply_body_wrench',
                 'rostopic',
                 'plot_workspace',
                 'plot_occupancy_grid',
                 'plot_shapely_geometry',
                 'plot_footprints',
                 'ax.',
                 '?']):
    # The MIT License (MIT)
    #
    # Copyright (c) 2019 Michael Dawson-Haggerty
    #
    # Permission is hereby granted, free of charge, to any person
    # obtaining a copy of this software and associated documentation
    # files (the "Software"), to deal in the Software without restriction,
    # including without limitation the rights to use, copy, modify, merge,
    # publish, distribute, sublicense, and/or sell copies of the Software,
    # and to permit persons to whom the Software is furnished to do so,
    # subject to the following conditions:
    #
    # The above copyright notice and this permission notice shall be
    # included in all copies or substantial portions of the Software.
    #
    # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    # EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    # OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    # IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
    # CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
    # TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
    # SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
    """
    Exclude certain calls based on substrings, replacing
    them with pass statements.

    Parameters
    -------------
    lines : (n, ) str
      Lines making up a Python script
    exclude (m, ) str
      Substrings to exclude lines based off of

    Returns
    -------------
    joined : str
      Lines combined with newline
    """
    result = []
    for line in lines:
        # skip lines that only have whitespace or comments
        strip = line.strip()
        if len(strip) == 0 or strip.startswith('#'):
            continue
        # if the line has a blacklisted phrase switch it with a pass statement
        # we don't want to exclude function definitions however
        if not strip.startswith('def ') and any(i in line for i in exclude):
            # switch statement with pass
            line_modified = to_pass(line)
        else:
            # remove trailing whitespace
            line_modified = line.rstrip()
        # skip duplicate lines
        if len(result) > 0 and line_modified == result[-1]:
            continue
        # append the modified line to the result
        result.append(line_modified)
    # recombine into string and add trailing newline
    result = '\n'.join(result) + '\n'
    return result


def to_pass(line):
    # The MIT License (MIT)
    #
    # Copyright (c) 2019 Michael Dawson-Haggerty
    #
    # Permission is hereby granted, free of charge, to any person
    # obtaining a copy of this software and associated documentation
    # files (the "Software"), to deal in the Software without restriction,
    # including without limitation the rights to use, copy, modify, merge,
    # publish, distribute, sublicense, and/or sell copies of the Software,
    # and to permit persons to whom the Software is furnished to do so,
    # subject to the following conditions:
    #
    # The above copyright notice and this permission notice shall be
    # included in all copies or substantial portions of the Software.
    #
    # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    # EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    # OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    # IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
    # CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
    # TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
    # SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
    """
    Replace a line of code with a pass statement, with
    the correct number of leading spaces

    Arguments
    ----------
    line : str, line of code

    Returns
    ----------
    passed : str, line of code with same leading spaces
                  but code replaced with pass statement
    """
    # the number of leading spaces on the line
    spaces = len(line) - len(line.lstrip(' '))
    # replace statement with pass and correct leading spaces
    passed = (' ' * spaces) + 'pass'
    return passed


EXAMPLES_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..',
    'examples'
)


def run_example(filename):
    if not os.path.exists(filename):
        return

    if filename.lower().endswith('.ipynb'):
        with open(filename, 'r') as file_obj:
            script = load_notebook(file_obj)
        print('-' for _ in range(20))
        print('Notebook: ', filename)
        exec(script, globals())
        plt.close('all')
        print('-' for _ in range(20))
    elif filename.lower().endswith('.py'):
        output = subprocess.check_output(['python', filename])
        assert output.returncode == 0


class TestExamples(unittest.TestCase):
    def test_generation_notebooks(self):
        if sys.version_info.major == 2:
            return

        notebooks = [
            'gen_fixed_pose_engine.ipynb',
            'gen_workspaces.ipynb',
            'gen_generate_heightmaps.ipynb',
            'gen_add_texture_to_heightmap.ipynb',
            # 'gen_grid_map.ipynb'
        ]

        for item in os.listdir(EXAMPLES_DIR):
            if 'gen_' in item and item in notebooks:
                filename = os.path.join(EXAMPLES_DIR, item)
                run_example(filename)

    def test_sdf_notebooks(self):
        if sys.version_info.major == 2:
            return

        for item in os.listdir(EXAMPLES_DIR):
            if 'sdf_' in item:
                filename = os.path.join(EXAMPLES_DIR, item)
                run_example(filename)

    def test_urdf_notebooks(self):
        if sys.version_info.major == 2:
            return

        for item in os.listdir(EXAMPLES_DIR):
            if 'urdf_' in item:
                filename = os.path.join(EXAMPLES_DIR, item)
                run_example(filename)

    def test_python_scripts(self):
        if sys.version_info.major == 2:
            return

        for item in os.listdir(EXAMPLES_DIR):
            if item.lower().endswith('.py'):
                filename = os.path.join(EXAMPLES_DIR, item)
                run_example(filename)


if __name__ == '__main__':
    unittest.main()
