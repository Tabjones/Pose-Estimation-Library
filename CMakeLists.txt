cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#set(FIND_DIR "../cmake/") 

#list(APPEND CMAKE_MODULE_PATH ${FIND_DIR}) 

project(PoseEstimation)

find_package(PCL 1.8.0 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(-std=c++11)

find_package(HDF5)
if (HDF5_FOUND)
  find_package(FLANN)
  include_directories(${FLANN_INCLUDE_DIRS})
  include_directories(${HDF5_INCLUDE_DIR})
  add_executable (poseEst pose_estimator.cpp include/interface.hpp include/interface.h)
  target_link_libraries (poseEst ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})
  add_executable(createDB create_db.cpp include/interface.hpp include/interface.h)
  target_link_libraries(createDB ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})
endif (HDF5_FOUND)
