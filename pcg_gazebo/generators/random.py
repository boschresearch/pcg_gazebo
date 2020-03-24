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
import numpy as np


PCG_RANDOM_STATE = None
PCG_RANDOM_SEED = None


def set_seed(value=None):
    global PCG_RANDOM_SEED
    PCG_RANDOM_SEED = value


def init_random_state(seed=None):
    global PCG_RANDOM_STATE
    set_seed(seed)
    PCG_RANDOM_STATE = np.random.RandomState(PCG_RANDOM_SEED)


def rand(*args):
    if PCG_RANDOM_STATE is None:
        init_random_state()
    return PCG_RANDOM_STATE.rand(*args)


def randint(*args, **kwargs):
    if PCG_RANDOM_STATE is None:
        init_random_state()
    return PCG_RANDOM_STATE.randint(*args, **kwargs)


def choice(*args, **kwargs):
    if PCG_RANDOM_STATE is None:
        init_random_state()
    return PCG_RANDOM_STATE.choice(*args, **kwargs)


def uniform(*args, **kwargs):
    if PCG_RANDOM_STATE is None:
        init_random_state()
    return PCG_RANDOM_STATE.uniform(*args, **kwargs)
