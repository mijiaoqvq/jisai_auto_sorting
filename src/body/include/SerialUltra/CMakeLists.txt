cmake_minimum_required(VERSION 3.5)
project(SerialUltra)

set(CMAKE_CXX_STANDARD 14)

find_package (Threads)

include_directories(include thirdparty/serialib)

add_subdirectory(thirdparty/serialib)

#add_executable(t2 test2.cpp)
#target_link_libraries(t2 serialib ${CMAKE_THREAD_LIBS_INIT})
#
#add_executable(t1 test1.cpp)
#target_link_libraries(t1 serialib ${CMAKE_THREAD_LIBS_INIT})
#
#add_executable(t3 test3.cpp)
#target_link_libraries(t3 serialib ${CMAKE_THREAD_LIBS_INIT})