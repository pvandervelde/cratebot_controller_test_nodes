# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions.launch_configuration import LaunchConfiguration

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='use_sim_time'
    ),
]

def generate_launch_description():

    steering_config = PathJoinSubstitution(
        [
            FindPackageShare("zinger_controller_test_nodes"),
            "config",
            "steering_controller.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="zinger_controller_test_nodes",
                executable="steering_controller",
                name="steering_controller",
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    steering_config
                ],
                arguments= [
                    "--ros-args",
                    "--log-level",
                    "node_test:=debug",
                ],
                output="both",
            )
        ]
    )
