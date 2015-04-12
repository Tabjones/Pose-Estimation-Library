#find dependencies and set up include directories
find_package(PCL 1.7.2 REQUIRED)
find_package(HDF5 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${HDF5_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(-std=c++11)

#Library build
add_library ( pel SHARED 
  PoseEstimation_interface.cpp 
  pel.h
  version.h
  )
set_target_properties ( pel PROPERTIES
  PUBLIC_HEADER "pel.h;version.h" 
  )
target_link_libraries (pel ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${HDF5_LIBRARIES})

#Library install
install ( TARGETS pel 
  EXPORT pelTargets
  DESTINATION ${PEL_INSTALL_DIR} 
  PUBLIC_HEADER DESTINATION ${PEL_INCLUDE_DIR} 
  )

