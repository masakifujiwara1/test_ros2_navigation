#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf.xacro'
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '2.urdf'
    # urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    urdf_path = '/home/fmasa/ros2_ws/src/test_ros2_navigation/robot_description'

    print('urdf_file_name : {}'.format(urdf_file_name))

    xacro_path = os.path.join(
        # get_package_share_directory('turtlebot3_description'),
        urdf_path,
        'urdf',
        xacro_file_name)

    urdf_path = os.path.join(
        # get_package_share_directory('turtlebot3_description'),
        urdf_path,
        'urdf',
        urdf_file_name)

    doc = xacro.process_file(xacro_path)
    robot_desc = doc.toprettyxml(indent='   ')
    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf_path]),
    ])
