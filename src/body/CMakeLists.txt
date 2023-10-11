cmake_minimum_required(VERSION 3.5)
project(body)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

add_subdirectory(include/thirdparty/hidapi)

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package (Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(libraries
        hidapi
        ${CMAKE_THREAD_LIBS_INIT}
)

set(dependencies
        rclcpp
        example_interfaces
        interfaces
        geometry_msgs
        cv_bridge
)

add_executable(body_node src/body_node.cpp)
target_link_libraries(body_node ${libraries})
ament_target_dependencies(body_node ${dependencies})
target_include_directories(body_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(TARGETS body_node
        DESTINATION lib/${PROJECT_NAME})

add_executable(test_usb_node src/test_usb.cpp)
target_link_libraries(test_usb_node ${libraries})
target_include_directories(test_usb_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(TARGETS test_usb_node
        DESTINATION lib/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
