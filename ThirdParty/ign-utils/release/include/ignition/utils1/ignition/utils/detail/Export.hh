
#ifndef DETAIL_IGNITION_UTILS_VISIBLE_H
#define DETAIL_IGNITION_UTILS_VISIBLE_H

#ifdef IGNITION_UTILS_STATIC_DEFINE
#  define DETAIL_IGNITION_UTILS_VISIBLE
#  define DETAIL_IGNITION_UTILS_HIDDEN
#else
#  ifndef DETAIL_IGNITION_UTILS_VISIBLE
#    ifdef ignition_utils1_EXPORTS
        /* We are building this library */
#      define DETAIL_IGNITION_UTILS_VISIBLE __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define DETAIL_IGNITION_UTILS_VISIBLE __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef DETAIL_IGNITION_UTILS_HIDDEN
#    define DETAIL_IGNITION_UTILS_HIDDEN __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef IGN_DEPRECATED_ALL_VERSIONS
#  define IGN_DEPRECATED_ALL_VERSIONS __attribute__ ((__deprecated__))
#endif

#ifndef IGN_DEPRECATED_ALL_VERSIONS_EXPORT
#  define IGN_DEPRECATED_ALL_VERSIONS_EXPORT DETAIL_IGNITION_UTILS_VISIBLE IGN_DEPRECATED_ALL_VERSIONS
#endif

#ifndef IGN_DEPRECATED_ALL_VERSIONS_NO_EXPORT
#  define IGN_DEPRECATED_ALL_VERSIONS_NO_EXPORT DETAIL_IGNITION_UTILS_HIDDEN IGN_DEPRECATED_ALL_VERSIONS
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef IGNITION_UTILS_NO_DEPRECATED
#    define IGNITION_UTILS_NO_DEPRECATED
#  endif
#endif

#endif
