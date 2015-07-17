cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(POLICY CMP0048)
  cmake_policy(SET CMP0048 OLD) #In case CMake version >= 3.0, retain old policy (Do not use VERSION in project() command)
endif()

project(PEL)
set(PEL_NAME ${PROJECT_NAME})
string(TOLOWER ${PROJECT_NAME} PEL_NAME_LOWER)
set(PEL_DESCRIPTION "Pose Estimation Library (PEL)")
set(PEL_URL "https://bitbucket.org/Tabjones/pose-estimation-library")
set(PEL_SOURCE_DIR ${PROJECT_SOURCE_DIR})
set(PEL_BIN_DIR ${PROJECT_BINARY_DIR})

## -------> Include pkgconfig (shipped with CMake)
include (FindPkgConfig)

## -------> Set Global Version of the Library, then dissect it to set various version variables
set (PEL_VERSION 0.4.1 CACHE STRING "PEL Version (maj.min.rev)")
string (REGEX REPLACE "^([0-9]+).*" "\\1"
    PEL_MAJOR_VERSION "${PEL_VERSION}")
string (REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1"
    PEL_MINOR_VERSION "${PEL_VERSION}")
string (REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1"
    PEL_REVISION_VERSION "${PEL_VERSION}")

## -------> Set Install dirs
set(PEL_LIB_INSTALL_DIR "lib" CACHE STRING "Relative path for library install")
set(INCLUDE_INSTALL_ROOT "include/${PEL_NAME_LOWER}-${PEL_MAJOR_VERSION}.${PEL_MINOR_VERSION}")
set(PEL_INCLUDE_INSTALL_DIR "${INCLUDE_INSTALL_ROOT}/${PEL_NAME_LOWER}" CACHE STRING "Relative path for headers install")
set(PEL_BIN_INSTALL_DIR "bin" CACHE STRING "Relative path for binaries install")
set(PEL_CONFIG_INSTALL_DIR "share/${PEL_NAME_LOWER}-${PEL_MAJOR_VERSION}.${PEL_MINOR_VERSION}" CACHE STRING "Relative path for PELConfig files install")
set(PEL_PKGCONFIG_INSTALL_DIR "${PEL_LIB_INSTALL_DIR}/pkgconfig" CACHE STRING "Relative path for pkgconfig files install")
set(PEL_OUTPUT_LIB_DIR "${PEL_BIN_DIR}/${PEL_LIB_INSTALL_DIR}" CACHE STRING "Output directory for Library")
set(PEL_OUTPUT_BIN_DIR "${PEL_BIN_DIR}/${PEL_BIN_INSTALL_DIR}" CACHE STRING "Output directory for Binaries")
make_directory("${PEL_OUTPUT_LIB_DIR}")
make_directory("${PEL_OUTPUT_BIN_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PEL_OUTPUT_LIB_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PEL_OUTPUT_LIB_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PEL_OUTPUT_BIN_DIR}")

##
# this is copy paste form http://www.itk.org/Wiki/CMake_RPATH_handling
# in order to always make a full statement RPATH
##
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${PEL_LIB_INSTALL_DIR}")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/${PEL_LIB_INSTALL_DIR}" is_system_dir)
IF("${is_system_dir}" STREQUAL "-1")
  SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${PEL_LIB_INSTALL_DIR}")
ENDIF("${is_system_dir}" STREQUAL "-1")

## -------> Add uninstall target  TODO
configure_file( "${PEL_SOURCE_DIR}/cmake/uninstall.cmake.in"
  "${PEL_BIN_DIR}/uninstall.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P
  "${PEL_BIN_DIR}/uninstakk.cmake")

## -------> Add support for c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") #will result in error during compilation if not supported

## -------> Set some user options (for now only 1!)
set(EIGEN_MPL2_ONLY ON CACHE BOOL "Force Eigen to use only MPL2(or more permissive) licensed code")

##################################################################
############## ------> Build Phase ###############################
##################################################################

## -------> Search for dependencies
#Pcl 1.7.2
find_package(PCL 1.7.2 REQUIRED)
if (PCL_FOUND)
  include_directories(BEFORE ${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  list(APPEND LINK_LIBS ${PCL_LIBRARIES})
endif (PCL_FOUND)
#hdf5
find_package(HDF5 REQUIRED)
if (HDF5_FOUND)
  include_directories(${HDF5_INCLUDE_DIRS})
  list(APPEND LINK_LIBS ${HDF5_LIBRARIES})
endif (HDF5_FOUND)
#boost
find_package(Boost REQUIRED)
if (Boost_FOUND)
  include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
  list(APPEND LINK_LIBS ${Boost_LIBRARIES})
endif (Boost_FOUND)

## -------> Library Build
include_directories(${PEL_SOURCE_DIR}/include)

configure_file("${PEL_SOURCE_DIR}/cmake/config_h.cmake.in"
  "${PEL_BIN_DIR}/include/pel/pel_config.h")

include_directories(${PEL_BIN_DIR}/include)

set(srcs
  src/pose_estimation.cpp
  src/storage/database.cpp
  )

set(incls
  "include/pel/pel_config.h"
  "include/pel/common.h"
  "include/pel/pose_estimation.h"
  "include/pel/storage/candidate.h"
  "include/pel/storage/database.h"
  )

add_library (${PEL_NAME_LOWER} SHARED ${srcs} ${incls})
target_link_libraries (${PEL_NAME_LOWER} ${LINK_LIBS})
set_target_properties(${PEL_NAME_LOWER} PROPERTIES
  VERSION "${PEL_VERSION}"
  SOVERSION "${PEL_MAJOR_VERSION}.${PEL_MINOR_VERSION}"
  LINK_FLAGS "-Wl, --as-needed"
  )

install(TARGETS ${PEL_NAME_LOWER}
  RUNTIME DESTINATION ${PEL_BIN_INSTALL_DIR}
  LIBRARY DESTINATION ${PEL_LIB_INSTALL_DIR}
  ARCHIVE DESTINATION ${PEL_LIB_INSTALL_DIR}
  )
install(FILES ${incls} DESTINATION ${PEL_INCLUDE_INSTALL_DIR})

## -------> Make a pkg-config file for the library
set(PKG_NAME ${PEL_NAME_LOWER})
set(PKG_DESC ${PEL_DESCRIPTION})
set(PKG_URL ${PEL_URL})
set(PKG_DEPS "pcl_common-1.7 pcl_features-1.7 pcl_filters-1.7 pcl_visualizaton-1.7 pcl_geometry-1.7
  pcl_io-1.7 pcl_registration-1.7 pcl_search-1.7 pcl_kdtree-1.7 pcl_sample_consensus-1.7
  pcl_segmentation-1.7 pcl_surface-1.7")
set(pc_file ${PEL_BIN_DIR}/${PEL_NAME_LOWER}-${PEL_MAJOR_VERSION}.${PEL_MINOR_VERSION}.pc)
configure_file(${PEL_SOURCE_DIR}/cmake/pkgconfig.cmake.in ${pc_file} @ONLY)
install(FILES
  "${pc_file}"
   DESTINATION ${PEL_PKGCONFIG_INSTALL_DIR})

## ------> Configure PELConfig.cmake
configure_file("${PEL_SOURCE_DIR}/cmake/PELConfig.cmake.in"
  "${PEL_BIN_DIR}/PELConfig.cmake" @ONLY)
configure_file("${PEL_SOURCE_DIR}/cmake/PELConfigVersion.cmake.in"
  "${PEL_BIN_DIR}/PELConfigVersion.cmake" @ONLY)
install(FILES
  "${PEL_BIN_DIR}/PELConfig.cmake"
  "${PEL_BIN_DIR}/PELConfigVersion.cmake"
    DESTINATION ${PEL_CONFIG_INSTALL_DIR})

