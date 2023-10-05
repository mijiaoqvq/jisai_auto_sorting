cmake_minimum_required(VERSION 3.5)
project(body)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package (Threads REQUIRED)

include_directories(include/SerialUltra/include include/SerialUltra/thirdparty/serialib)
add_subdirectory(include/SerialUltra)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
        rclcpp
        example_interfaces
        tf2_msgs
        interfaces
        geometry_msgs
)

add_executable(arm_node src/arm_node.cpp)
target_link_libraries(arm_node serialib ${CMAKE_THREAD_LIBS_INIT})
ament_target_dependencies(arm_node ${dependencies})
target_include_directories(arm_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(TARGETS arm_node
        DESTINATION lib/${PROJECT_NAME})

add_executable(find_device_node src/find_device_node.cpp)
target_link_libraries(find_device_node serialib ${CMAKE_THREAD_LIBS_INIT})
ament_target_dependencies(find_device_node ${dependencies})
target_include_directories(find_device_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
 install(TARGETS find_device_node
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(qr_code_node src/qr_code_node.cpp)
target_link_libraries(qr_code_node serialib)
ament_target_dependencies(qr_code_node ${dependencies})
target_include_directories(qr_code_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS qr_code_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(chassis_node src/chassis_node.cpp)
target_link_libraries(chassis_node serialib ${OpenCV_LIBS})
ament_target_dependencies(chassis_node ${dependencies})
target_include_directories(chassis_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(TARGETS chassis_node
        DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
