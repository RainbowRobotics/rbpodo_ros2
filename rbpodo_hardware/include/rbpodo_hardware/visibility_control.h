#ifndef RBPODO_HARDWARE__VISIBILITY_CONTROL_H_
#define RBPODO_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RBPODO_HARDWARE_EXPORT __attribute__ ((dllexport))
    #define RBPODO_HARDWARE_IMPORT __attribute__ ((dllimport))
  #else
    #define RBPODO_HARDWARE_EXPORT __declspec(dllexport)
    #define RBPODO_HARDWARE_IMPORT __declspec(dllimport)
  #endif
  #ifdef RBPODO_HARDWARE_BUILDING_LIBRARY
    #define RBPODO_HARDWARE_PUBLIC RBPODO_HARDWARE_EXPORT
  #else
    #define RBPODO_HARDWARE_PUBLIC RBPODO_HARDWARE_IMPORT
  #endif
  #define RBPODO_HARDWARE_PUBLIC_TYPE RBPODO_HARDWARE_PUBLIC
  #define RBPODO_HARDWARE_LOCAL
#else
  #define RBPODO_HARDWARE_EXPORT __attribute__ ((visibility("default")))
  #define RBPODO_HARDWARE_IMPORT
  #if __GNUC__ >= 4
    #define RBPODO_HARDWARE_PUBLIC __attribute__ ((visibility("default")))
    #define RBPODO_HARDWARE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RBPODO_HARDWARE_PUBLIC
    #define RBPODO_HARDWARE_LOCAL
  #endif
  #define RBPODO_HARDWARE_PUBLIC_TYPE
#endif

#endif  // RBPODO_HARDWARE__VISIBILITY_CONTROL_H_
