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
Global logging methods.
"""
import logging
import os
import sys
import datetime
import random
import string


# Output root directory for log file
PCG_LOG_DIR_ROOT = os.path.join(os.path.expanduser('~'), '.pcg', 'logs')

# Output directory for this instance of the PCG engine
PCG_LOG_DIR = os.path.join(
    PCG_LOG_DIR_ROOT,
    datetime.datetime.now().isoformat().replace(':', '_') + '_{}'.format(
        ''.join(random.choice(string.ascii_letters) for i in range(3))))


def update_log_dir(add_timestamp=True):
    """Update the output folder for log files.

    > *Input arguments*

    * `add_timestamp` (*type:* `bool`, *default:* `True`): Add timestamp to
    folder name.
    """
    global PCG_LOG_DIR
    if add_timestamp:
        PCG_LOG_DIR = os.path.join(
            PCG_LOG_DIR_ROOT,
            datetime.datetime.now().isoformat().replace(
                ':',
                '_') +
            '_{}'.format(
                ''.join(
                    random.choice(
                        string.ascii_letters) for i in range(3))))
    else:
        PCG_LOG_DIR = PCG_LOG_DIR_ROOT


def change_log_root_dir(root_dir, update_log_dir=False, add_timestamp=True):
    """Change the directory for the output log files.

    > *Input arguments*

    * `root_dir` (*type:* `str`): New output log directory.
    * `update_log_dir` (*type:* `bool`, *default:* `False`):
    Update current global path for log directory.
    * `add_timestamp` (*type:* `bool`, *default:* `True`): Add timestamp
    to the directory folder name.
    """
    global PCG_LOG_DIR_ROOT
    PCG_LOG_DIR_ROOT = root_dir
    if update_log_dir:
        update_log_dir(add_timestamp)


def get_log_dir():
    """Return the current output log directory."""
    return PCG_LOG_DIR


def create_logger(name, log_filename=None, output_dir=None,
                  log_level=logging.ERROR):
    """Create new instance of the logger.

    > *Input arguments*

    * `name` (*type:* `str`): Name of the instance.
    * `log_filename` (*type:* `str`, *default:* `None`): Name of
    the log filename, if `None` use `name` as filename.
    * `output_dir` (*type:* `str`, *default:* `data`): Output log directory.
    If `None` is provided, use the current log directory `PCG_LOG_DIR`.
    * `log_level` (*type:* `int`, *default:* `logging.ERROR`): Log level.

    > *Returns*

    Logger object.
    """
    logger = logging.getLogger(name)
    if len(logger.handlers) == 0:
        out_hdlr = logging.StreamHandler(sys.stdout)
        out_hdlr.setFormatter(logging.Formatter(
            '%(asctime)s | %(levelname)s | %(module)s | %(message)s'))
        out_hdlr.setLevel(log_level)
        logger.addHandler(out_hdlr)
        logger.setLevel(log_level)

        if log_filename is None:
            if output_dir is None:
                output_dir = PCG_LOG_DIR

            if not os.path.isdir(output_dir):
                os.makedirs(output_dir)
            log_filename = os.path.join(output_dir, '{}.log'.format(name))

        if log_filename is not None:
            file_hdlr = logging.FileHandler(log_filename)
            file_hdlr.setFormatter(
                logging.Formatter(
                    '%(asctime)s | %(levelname)s | %(module)s | %(message)s'))
            file_hdlr.setLevel(log_level)

            logger.addHandler(file_hdlr)
            logger.setLevel(logging.WARNING)
    return logger


PCG_ROOT_LOGGER = create_logger('pcg_gazebo')
