cmake_minimum_required(VERSION 3.5)
project(hidapi)

set(CMAKE_CXX_STANDARD 14)

set(UDEV_ROOT_DIR
        "${UDEV_ROOT_DIR}"
        CACHE
        PATH
        "Directory to search for udev")

if(NOT ANDROID)
    find_package(PkgConfig QUIET)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(PC_LIBUDEV QUIET libudev)
    endif()
endif()

find_library(UDEV_LIBRARY
        NAMES
        udev
        PATHS
        ${PC_LIBUDEV_LIBRARY_DIRS}
        ${PC_LIBUDEV_LIBDIR}
        HINTS
        "${UDEV_ROOT_DIR}"
        PATH_SUFFIXES
        lib
)

get_filename_component(_libdir "${UDEV_LIBRARY}" PATH)

find_path(UDEV_INCLUDE_DIR
        NAMES
        libudev.h
        PATHS
        ${PC_LIBUDEV_INCLUDE_DIRS}
        ${PC_LIBUDEV_INCLUDEDIR}
        HINTS
        "${_libdir}"
        "${_libdir}/.."
        "${UDEV_ROOT_DIR}"
        PATH_SUFFIXES
        include
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(udev
        DEFAULT_MSG
        UDEV_LIBRARY
        UDEV_INCLUDE_DIR
)

if(UDEV_FOUND)
    list(APPEND UDEV_LIBRARIES ${UDEV_LIBRARY})
    list(APPEND UDEV_INCLUDE_DIRS ${UDEV_INCLUDE_DIR})
    mark_as_advanced(UDEV_ROOT_DIR)
endif()

mark_as_advanced(UDEV_INCLUDE_DIR
        UDEV_LIBRARY)

add_library(hidapi hid.c)
target_link_libraries(hidapi ${UDEV_LIBRARY})