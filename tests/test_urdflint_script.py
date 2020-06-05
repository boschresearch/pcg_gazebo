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
from __future__ import print_function
import unittest
import subprocess
import os


class TestURDFLintScript(unittest.TestCase):
    def test_run_linter_on_test_files(self):
        test_folder = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'urdf')
        for filename in os.listdir(test_folder):
            if '.urdf' not in filename:
                continue
            subprocess.check_output(
                ['pcg-urdflint', '--filename',
                 os.path.join(test_folder, filename)])


if __name__ == '__main__':
    unittest.main()
