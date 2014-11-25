cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


project(test)

find_package(PEL REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(${PEL_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_definitions(-std=c++11)

add_executable (prova prova.cpp)
target_link_libraries (prova 
                       ${PEL_LIBRARIES}
                       ${PCL_LIBRARIES} 
                       )

