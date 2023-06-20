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

from launch import LaunchDescription
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument

from typing import Iterable
from typing import Optional
from typing import Text


class Catch2LaunchDescription(LaunchDescription):
    """
    A wrapper around launch.LaunchDescription for Catch2 integration testing.

    Adds a required "result_file" argument for the launch file.
    This file will hold the results of the test.
    """

    def __init__(
        self,
        initial_entities: Optional[Iterable[LaunchDescriptionEntity]] = None,
        *,
        deprecated_reason: Optional[Text] = None
    ) -> None:

        # Add launch arguments for Catch
        initial_entities_appended = [
            DeclareLaunchArgument(
                name='result_file',
                description='Catch 2 test result file output location',
                default_value='/tmp/test_results.xml',
            ),
        ]

        if initial_entities:
            initial_entities_appended += initial_entities

        super().__init__(
            initial_entities=initial_entities_appended,
            deprecated_reason=deprecated_reason
        )
