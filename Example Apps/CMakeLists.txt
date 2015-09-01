cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


project(example_apps)

find_package(PEL REQUIRED)
find_package(PCL 1.7.2 REQUIRED)
find_package(Boost)

include_directories(${PEL_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_definitions(-std=c++11)

add_executable (pel_estimator estimator_on_demand.cpp)
target_link_libraries (pel_estimator
                       ${PEL_LIBRARIES}
                       ${PCL_LIBRARIES}
                       ${Boost_LIBRARIES}
                       )

add_executable (db_creator prova2.cpp)
target_link_libraries (db_creator
                       ${PEL_LIBRARIES}
                       ${PCL_LIBRARIES}
                       ${Boost_LIBRRARIES}
                       )
