cmake_minimum_required(VERSION 3.12)

#set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake;/usr/local/share/ifopt/cmake" ${CMAKE_MODULE_PATH})
#foreach(path ${CMAKE_MODULE_PATH})
#  message("CMake module Path: " ${path})
#endforeach(path)


find_package(Eigen3 REQUIRED)
#include_directories(
#    ${EIGEN3_INCLUDE_DIR}
#)

#list(APPEND CMAKE_PREFIX_PATH
#    "/usr/local/lib;"
#)
#list(APPEND CMAKE_PREFIX_PATH
#    "/usr/local/include"
#)
#foreach(path ${CMAKE_PREFIX_PATH})
#  message("Cmake prefix Path: " ${path})
#endforeach(path)

#message(STATUS ${CMAKE_PREFIX_PATH})
#message(STATUS ${CMAKE_MODULE_PATH})
#message(STATUS ${CMAKE_INSTALL_PREFIX})

find_package(ifopt REQUIRED)
#message(STATUS ${ifopt_FOUND})
#message(STATUS ${ifopt_INCLUDE_DIRS})

#message (STATUS ${ifopt_LIB_IPOPT})
#message (STATUS ${ifopt_LIB_SNOPT})

set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(main_ipopt main_ipopt.cpp)
add_executable(main_snopt main_snopt.cpp)
# Pull in include directories, libraries, ... 
target_link_libraries(main_ipopt PUBLIC ifopt::ifopt_ipopt) 
target_link_libraries(main_snopt PUBLIC ifopt::ifopt_snopt) 

#target_compile_features(main_snopt
#  PUBLIC 
#    cxx_range_for 
#    cxx_auto_type
#)
