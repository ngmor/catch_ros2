# Copyright 2023 Nick Morales.
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

from launch.actions import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from typing import Iterable
from typing import Optional

from launch.some_substitutions_type import SomeSubstitutionsType

from launch_ros.parameters_type import SomeParameters
from launch_ros.remap_rule_type import SomeRemapRules


class Catch2IntegrationTestNode(Node):
    def __init__(
        self, *,
        executable: SomeSubstitutionsType,
        package: Optional[SomeSubstitutionsType] = None,
        name: Optional[SomeSubstitutionsType] = None,
        namespace: Optional[SomeSubstitutionsType] = None,
        exec_name: Optional[SomeSubstitutionsType] = None,
        parameters: Optional[SomeParameters] = None,
        remappings: Optional[SomeRemapRules] = None,
        ros_arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        **kwargs
    ) -> None:
        
        # Add arguments for Catch
        arguments_appended = [
            '--reporter',
            ['JUnit::out=', LaunchConfiguration('result_file')],
            '--reporter',
            'console::out=-::colour-mode=ansi'
        ]

        if arguments:
            arguments_appended += arguments

        super().__init__(
            executable=executable,
            package=package,
            name=name,
            namespace=namespace,
            exec_name=exec_name,
            parameters=parameters,
            remappings=remappings,
            ros_arguments=ros_arguments,
            arguments=arguments_appended,
            on_exit=Shutdown(),
            output='screen',
            **kwargs
        )