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
import collections
from time import time, sleep
from ..log import PCG_ROOT_LOGGER
from ..simulation.properties import Pose as PosePCG
from .ros_config import ROSConfig

try:
    import rospy
    from std_srvs.srv import Empty
    from geometry_msgs.msg import Pose, Point, Wrench
    from rosgraph_msgs.msg import Clock
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import GetWorldProperties, SpawnModel, DeleteModel, \
        GetModelProperties, SetModelState, GetLinkProperties, \
        GetPhysicsProperties, GetLinkState, GetModelState, ApplyBodyWrench
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class GazeboProxy(object):

    def __init__(self, ros_host='localhost', ros_port=11311,
                 gazebo_host='localhost', gazebo_port=11345, timeout=30,
                 ignore_services=None):
        assert ROS_AVAILABLE, 'ROS components could not be imported'
        assert timeout >= 0, 'Timeout should be equal or greater than zero'
        assert isinstance(ros_host, str), 'ROS host name must be a string'
        assert isinstance(ros_port, int), 'ROS port must be an integer'
        assert ros_port > 0, 'ROS port must be a positive integer'
        assert isinstance(gazebo_host, str), 'Gazebo host name must be a' \
            ' string'
        assert isinstance(gazebo_port, int), 'Gazebo port must be an integer'
        assert gazebo_port > 0, 'Gazebo port must be a positive integer'
        if ignore_services is not None:
            assert isinstance(ignore_services, list), 'Filter for ignored ' \
                'services should be a list'

        self._GAZEBO_SERVICES = dict(
            get_world_properties=GetWorldProperties,
            spawn_sdf_model=SpawnModel,
            pause_physics=Empty,
            unpause_physics=Empty,
            delete_model=DeleteModel,
            get_model_properties=GetModelProperties,
            set_model_state=SetModelState,
            get_link_properties=GetLinkProperties,
            get_physics_properties=GetPhysicsProperties,
            get_link_state=GetLinkState,
            get_model_state=GetModelState,
            apply_body_wrench=ApplyBodyWrench
        )

        self._logger = PCG_ROOT_LOGGER

        self._ros_config = ROSConfig(
            ros_host=ros_host,
            ros_port=ros_port,
            gazebo_port=gazebo_port,
            gazebo_host=gazebo_host)

        self._ignore_services = list()
        if ignore_services is not None:
            for item in ignore_services:
                if item not in self._GAZEBO_SERVICES:
                    self._logger.warning(
                        'Ignoring service {}: Service {} does '
                        'not exist'.format(
                            item, item))
                else:
                    self._ignore_services.append(item)

        self._ros_config.set_env_variables()

        self._logger.info('Loading Gazebo proxy')
        self._logger.info('Network properties')
        self._logger.info(self._ros_config)

        self._sim_time = 0
        self._sim_time_sub = None
        self._services = dict()

        from . import is_gazebo_running
        start_time = time()
        while not is_gazebo_running(self._ros_config.ros_master_uri) and \
                time() - start_time < timeout:
            self._logger.info('Waiting for Gazebo')
            sleep(0.5)

        if not is_gazebo_running(self._ros_config.ros_master_uri):
            raise RuntimeError('Gazebo did not start!')

        self._logger.info('Initializing Gazebo proxy')
        self.init()
        self._logger.info('Gazebo proxy initialized')

    @property
    def ros_config(self):
        return self._ros_config

    @property
    def sim_time(self):
        return self._sim_time

    def _clock_callback(self, msg):
        self._sim_time = rospy.Time(msg.clock.secs, msg.clock.nsecs).to_sec()

    def init(self):
        if self.is_init():
            return True
        try:
            self._services = dict()
            for name, srv_class in self._GAZEBO_SERVICES.items():
                if name in self._ignore_services:
                    self._logger.info('Ignoring service {}'.format(name))
                    continue
                srv_name = '/gazebo/{}'.format(name)
                rospy.wait_for_service(srv_name, timeout=60)
                self._services[name] = rospy.ServiceProxy(srv_name, srv_class)

                self._logger.info('Service {} found'.format(srv_name))
        except Exception as ex:
            self._logger.warning('Gazebo proxy was not initialized, '
                                 'message={}'.format(ex))
            return False

        try:
            self._sim_time_sub = rospy.Subscriber(
                'clock', Clock, self._clock_callback)

            rospy.init_node('gazebo_proxy', anonymous=True)
            return True
        except Exception as ex:
            self._logger.warning('Could not subscribe to Gazebo clock, '
                                 'message={}'.format(ex))
        return True

    def is_init(self):
        from . import is_roscore_running, is_gazebo_running
        instances_running = \
            is_roscore_running(self._ros_config.ros_master_uri) and \
            is_gazebo_running(self._ros_config.ros_master_uri)

        if not instances_running:
            return False

        for name in self._GAZEBO_SERVICES:
            if name not in self._services:
                return False

        return True

    def pause(self):
        assert 'pause_physics' in self._services
        self._services['pause_physics']()
        self._logger.info('Physics paused')

    def unpause(self):
        assert 'unpause_physics' in self._services
        self._services['unpause_physics']()
        self._logger.info('Physics unpaused')

    def get_model_properties(self, model_name):
        assert 'get_model_properties' in self._services
        assert isinstance(model_name, str)
        if model_name not in self.get_model_names():
            print('Model {} could not be found in the simulation'.format(
                model_name))
            return None
        output = self._services['get_model_properties'](model_name)
        if output.success:
            return output
        else:
            return None

    def get_model_state(self, model_name, reference_frame='world'):
        assert 'get_model_state' in self._services
        assert isinstance(model_name, str)
        if model_name not in self.get_model_names():
            print('Model {} could not be found in the simulation'.format(
                model_name))
            return None
        output = self._services['get_model_state'](model_name, reference_frame)
        if output.success:
            return output
        else:
            return None

    def get_world_properties(self):
        assert 'get_world_properties' in self._services
        return self._services['get_world_properties']()

    def get_link_properties(self, model_name, link_name):
        assert 'get_link_properties' in self._services
        output = self._services['get_link_properties'](
            '{}::{}'.format(model_name, link_name))
        if output.success:
            return output
        else:
            return None

    def get_link_state(self, model_name, link_name, reference_frame=''):
        assert 'get_link_state' in self._services
        output = self._services['get_link_state'](
            '{}::{}'.format(model_name, link_name),
            reference_frame)
        if output.success:
            return output
        else:
            return None

    def get_physics_properties(self):
        assert 'get_physics_properties' in self._services
        return self._services['get_physics_properties']()

    def get_model_names(self):
        assert 'get_world_properties' in self._services
        return self.get_world_properties().model_names

    def model_exists(self, model_name):
        return model_name in self.get_model_names()

    def get_link_names(self, model_name):
        assert 'get_model_properties' in self._services
        assert isinstance(model_name, str)
        if model_name not in self.get_model_names():
            print('Model {} could not be found in the simulation'.format(
                model_name))
            return None
        return self._services['get_model_properties'](model_name).body_names

    def has_link(self, model_name, link_name):
        link_names = self.get_link_names(model_name)
        if link_names is None:
            return False
        return link_name in link_names

    def is_rendering_enabled(self):
        assert 'get_world_properties' in self._services
        return self._services['get_world_properties']().rendering_enabled

    def delete_model(self, model_name):
        assert 'delete_model' in self._services
        status = self._services['delete_model'](model_name)
        return status.success

    def move_model(self, model_name, pos, rot=[0, 0, 0],
                   reference_frame='world'):
        assert 'set_model_state' in self._services
        assert isinstance(model_name, str), 'Input model_name must be a string'
        assert model_name in self.get_model_names(), \
            'Model {} does not exist'.format(model_name)

        assert isinstance(pos, collections.Iterable)
        assert len(list(pos)) == 3, 'Position vector must have three ' \
            'components, (x, y, z)'
        for elem in pos:
            assert isinstance(elem, float) or isinstance(elem, int), \
                '{} is not a valid number'.format(elem)

        assert isinstance(rot, collections.Iterable), \
            'Rotation vector must be iterable'
        assert len(list(rot)) == 3 or len(list(rot)) == 4
        for elem in rot:
            assert isinstance(elem, float) or isinstance(elem, int), \
                '{} is not a valid number'.format(elem)

        if len(list(rot)) == 3:
            rot = PosePCG.rpy2quat(*rot)

        state = ModelState()

        state.model_name = model_name
        state.pose.position.x = pos[0]
        state.pose.position.y = pos[1]
        state.pose.position.z = pos[2]

        state.pose.orientation.x = rot[0]
        state.pose.orientation.y = rot[1]
        state.pose.orientation.z = rot[2]
        state.pose.orientation.w = rot[3]

        state.reference_frame = reference_frame

        status = self._services['set_model_state'](state)
        return status.success

    def apply_body_wrench(self, model_name, link_name, force, torque,
                          start_time=0, duration=-1, reference_point=[0, 0, 0],
                          reference_frame=None):
        assert 'apply_body_wrench' in self._services
        assert model_name in self.get_model_names(
        ), 'Model {} does not exist'.format(model_name)

        if reference_frame is None:
            reference_frame = '{}::{}'.format(model_name, link_name)

        # Set reference point
        point = Point(*reference_point)
        # Set wrench
        wrench = Wrench()
        wrench.force.x = force[0]
        wrench.force.y = force[1]
        wrench.force.z = force[2]

        wrench.torque.x = torque[0]
        wrench.torque.y = torque[1]
        wrench.torque.z = torque[2]

        output = self._services['apply_body_wrench'](
            '{}::{}'.format(model_name, link_name),
            reference_frame,
            point,
            wrench,
            rospy.Time(start_time),
            rospy.Duration(duration)
        )
        return output.success

    def spawn_sdf_model(self, robot_namespace, xml, pos=[0, 0, 0],
                        rot=[0, 0, 0, 1], reference_frame='world'):
        assert 'spawn_sdf_model' in self._services
        assert isinstance(robot_namespace, str), \
            'Input robot_namespace must be a string'

        assert isinstance(pos, list)
        assert len(pos) == 3, 'Input position vector must have three ' \
            'components (x, y, z)'
        for elem in pos:
            assert isinstance(elem, float) or isinstance(elem, int), \
                '{} is not a valid number'.format(elem)

        assert len(list(rot)) == 3 or len(list(rot)) == 4
        for elem in rot:
            assert isinstance(elem, float) or isinstance(elem, int), \
                '{} is not a valid number'.format(elem)

        if len(list(rot)) == 3:
            rot = PosePCG.rpy2quat(*rot)

        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]

        pose.orientation.x = rot[0]
        pose.orientation.y = rot[1]
        pose.orientation.z = rot[2]
        pose.orientation.w = rot[3]

        return self._services['spawn_sdf_model'](
            robot_namespace,
            xml,
            robot_namespace,
            pose,
            reference_frame).success
