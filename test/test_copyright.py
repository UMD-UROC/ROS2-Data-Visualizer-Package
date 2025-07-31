# Copyright 2015 Open Source Robotics Foundation, Inc.
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
Copyright header compliance test for UROC data visualizer package.

This test checks that all source files in the package contain proper
Apache 2.0 license headers as required by ROS2 package standards.
Currently skipped until copyright headers are added to source files.
"""

import pytest
from ament_copyright.main import main


# Remove the `skip` decorator once the source file(s) have a copyright header
@pytest.mark.skip(reason="No copyright header has been placed in the generated source file.")
@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    """
    Test for copyright header compliance in source files.
    
    Runs the ament_copyright tool to verify that all source files
    contain appropriate copyright headers. This helps ensure
    legal compliance and proper attribution.
    
    Returns
    -------
    None
        Asserts that no copyright violations are found
    """
    rc = main(argv=[".", "test"])
    assert rc == 0, "Found errors"
