/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * ==========================================================================
 * This file was automatically generated by CMake; do not modify it directly.
 * To modify this file, make changes to ign-cmake/cmake/Export.hh.in
 * ==========================================================================
*/

#ifndef IGNITION_SDFORMAT_EXPORT_HH_
#define IGNITION_SDFORMAT_EXPORT_HH_

// The detail/Export.hh header is automatically generated by CMake, which only
// provides the system-dependent implementations of these macros, with no
// commentary or explanation, so we configure this public-facing header which
// leverages the auto-generated macros but provides commentary for them.
#include "sdf/detail/Export.hh"


#ifndef IGNITION_SDFORMAT_VISIBLE
/// For sdformat12 developers: Apply this macro to sdformat12
/// functions and classes which consumers of this library will need to be able
/// to call from their own programs or libraries.
#define IGNITION_SDFORMAT_VISIBLE \
  DETAIL_IGNITION_SDFORMAT_VISIBLE
#endif


#ifndef IGNITION_SDFORMAT_HIDDEN
/// For sdformat12 developers: Apply this macro to sdformat12
/// functions and classes which must not be used by consumers of this library.
/// By default, this property is applied to all classes and functions which are
/// not tagged with IGNITION_SDFORMAT_VISIBLE, so this does not
/// generally need to be used.
#define IGNITION_SDFORMAT_HIDDEN \
  DETAIL_IGNITION_SDFORMAT_HIDDEN
#endif


#ifndef IGN_DEPRECATED
/// For sdformat12 developers: Use this macro to indicate that a
/// function or class has been deprecated and should no longer be used. A
/// version should be specified to provide context to the user about when the
/// function became deprecated.
#define IGN_DEPRECATED(version) IGN_DEPRECATED_ALL_VERSIONS
#endif

#endif
