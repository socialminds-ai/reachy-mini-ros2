# Copyright 2026 SocialMinds Lab, IIIA-CSIC
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

# Defines the namespaced imported target reachy_mini_c::reachy_mini_c for
# downstream packages (find_package(reachy_mini_c REQUIRED)), plus the classic
# <pkg>_LIBRARIES / <pkg>_INCLUDE_DIRS variables for ament_target_dependencies().
get_filename_component(_rmc_prefix "${reachy_mini_c_DIR}/../../.." ABSOLUTE)
if(NOT TARGET reachy_mini_c::reachy_mini_c)
  add_library(reachy_mini_c::reachy_mini_c SHARED IMPORTED)
  set_target_properties(reachy_mini_c::reachy_mini_c PROPERTIES
    IMPORTED_LOCATION "${_rmc_prefix}/lib/libreachy_mini_c.so"
    INTERFACE_INCLUDE_DIRECTORIES "${_rmc_prefix}/include")
endif()
set(reachy_mini_c_LIBRARIES "${_rmc_prefix}/lib/libreachy_mini_c.so")
set(reachy_mini_c_INCLUDE_DIRS "${_rmc_prefix}/include")
