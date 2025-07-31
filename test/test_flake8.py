# Copyright 2017 Open Source Robotics Foundation, Inc.
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
Flake8 code style compliance test for UROC data visualizer package.

This test runs flake8 static analysis on the package source code to
check for Python code style violations, syntax errors, and common
programming mistakes according to PEP 8 style guidelines.
"""

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """
    Test for Python code style compliance using flake8.
    
    Runs flake8 static analysis tool to check for:
    - PEP 8 style violations
    - Syntax errors
    - Undefined variable references  
    - Unused imports
    - Other common Python code issues
    
    Returns
    -------
    None
        Asserts that no code style violations are found
    """
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
