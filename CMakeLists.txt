cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PEL)
set (PEL_VERSION_MAJOR  0)
set (PEL_VERSION_MINOR  1)
configure_file (
  "${PROJECT_SOURCE_DIR}/cmake/version.h.in"
  "${PROJECT_SOURCE_DIR}/include/version.h"
  )

find_package(PCL 1.7 REQUIRED)
find_package(HDF5 REQUIRED)
#find_package(FLANN)

include_directories(${PCL_INCLUDE_DIRS})
include_directories("${PROJECT_SOURCE_DIR}/include")
#include_directories(${FLANN_INCLUDE_DIRS})
include_directories(${HDF5_INCLUDE_DIR})

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${PROJECT_BINARY_DIR})

add_definitions(${PCL_DEFINITIONS})
add_definitions(-std=c++11)

add_library (PEL SHARED "${PROJECT_SOURCE_DIR}/implementation/PoseEstimation_interface.cpp")

add_executable (prova Tests/prova.cpp )
#target_link_libraries (prova ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} PoseEstimation)
target_link_libraries (prova ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${HDF5_hdf5_LIBRARY} PEL)
