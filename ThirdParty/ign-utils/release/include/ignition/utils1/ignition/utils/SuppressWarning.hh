/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef IGNITION_UTILS__SUPPRESSWARNING_HH_
#define IGNITION_UTILS__SUPPRESSWARNING_HH_

#include <ignition/utils/detail/SuppressWarning.hh>

// This header contains cross-platform macros for suppressing warnings. Please
// only use these macros responsibly when you are certain that the compiler is
// producing a warning that is not applicable to the specific instance. Do not
// use these macros to ignore legitimate warnings, even if you may find them
// irritating.

/*
 * Usage example:
 *
 * SomeClass *ptr = CreatePtr();
 * IGN_UTILS_WARN_IGNORE__NON_VIRTUAL_DESTRUCTOR
 * delete ptr;
 * IGN_UTILS_WARN_RESUME__NON_VIRTUAL_DESTRUCTOR
 *
 */

// Be sure to call the IGN_UTILS_WARN_RESUME__XXXXX macro at the end of the
// block of code where the warning suppression is needed. Otherwise, you might
// inadvertently suppress legitimate warnings.

// ---- List of available suppressions ----

/// \brief Compilers might warn about deleting a pointer to a class that has
/// virtual functions without a virtual destructor or a `final` declaration,
/// because the pointer might secretly be pointing to a more derived class type.
/// We want to suppress this warning when we know for certain (via the design
/// of our implementation) that the pointer is definitely not pointing to a more
/// derived type.
#ifndef IGN_UTILS_WARN_IGNORE__NON_VIRTUAL_DESTRUCTOR
  #define IGN_UTILS_WARN_IGNORE__NON_VIRTUAL_DESTRUCTOR \
    DETAIL_IGN_UTILS_WARN_IGNORE__NON_VIRTUAL_DESTRUCTOR
#endif

#ifndef IGN_UTILS_WARN_RESUME__NON_VIRTUAL_DESTRUCTOR
  #define IGN_UTILS_WARN_RESUME__NON_VIRTUAL_DESTRUCTOR \
    DETAIL_IGN_UTILS_WARN_RESUME__NON_VIRTUAL_DESTRUCTOR
#endif

/// \brief Microsoft Visual Studio does not automatically export the interface
/// information for member variables that belong to interface classes of a DLL.
/// Instead it issues this warning. When the member variable is private, we
/// choose to suppress the warning instead of needlessly adding the class
/// information to the DLL interface.
#ifndef IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
  #define IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING \
    DETAIL_IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
#endif

#ifndef IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
  #define IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING \
    DETAIL_IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
#endif

/// \brief Use this to suppress deprecation warnings. This may be useful when
/// retaining tests for deprecated methods to preserve code coverage.
#ifndef IGN_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
  #define IGN_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION \
    DETAIL_IGN_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
#endif

#ifndef IGN_UTILS_WARN_RESUME__DEPRECATED_DECLARATION
  #define IGN_UTILS_WARN_RESUME__DEPRECATED_DECLARATION \
    DETAIL_IGN_UTILS_WARN_RESUME__DEPRECATED_DECLARATION
#endif

// TODO(anyone): Add more warning types as they become relevant.
// Do not add warning types to suppress unless they are genuinely necessary.

#endif  // IGNITION_UTILS__SUPPRESSWARNING_HH_
