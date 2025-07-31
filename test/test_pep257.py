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
PEP 257 docstring compliance test for UROC data visualizer package.

This test checks that all Python modules, classes, and functions in the
package have proper docstrings that comply with PEP 257 documentation
standards. This ensures code maintainability and API documentation quality.
"""

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """
    Test for Python docstring compliance using PEP 257 standards.
    
    Runs the pep257 tool to verify that all Python modules, classes,
    and functions have appropriate docstrings following the conventions
    outlined in PEP 257. This helps maintain code documentation quality
    and ensures consistent API documentation.
    
    Returns
    -------
    None
        Asserts that no docstring violations are found
    """
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
