cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)

if(POLICY CMP0048)
  cmake_policy(SET CMP0048 OLD) #In case CMake version >= 3.0, retain old policy (Do not use VERSION in project() command)
endif()

project(pel) #lowercase name

# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

set(pel_NAME ${PROJECT_NAME})
set(pel_DESCRIPTION "Pose Estimation Library (PEL)")
set(pel_URL "https://github.com/Tabjones/Pose-Estimation-Library")
set(pel_SOURCE_DIR ${PROJECT_SOURCE_DIR})
set(pel_BIN_DIR ${PROJECT_BINARY_DIR})

## -------> Include pkgconfig (shipped with CMake)
include (FindPkgConfig)

## -------> Set Global Version of the Library, then dissect it to set various version variables
set (pel_VERSION 0.6.2 CACHE STRING "PEL Version (maj.min.rev)") ## <---- Change this to update library version globally
string (REGEX REPLACE "^([0-9]+).*" "\\1"
    pel_MAJOR_VERSION "${pel_VERSION}")
string (REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1"
    pel_MINOR_VERSION "${pel_VERSION}")
string (REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1"
    pel_REVISION_VERSION "${pel_VERSION}")

## -------> Set Install dirs
set(pel_LIB_INSTALL_DIR "lib" CACHE STRING "Relative path for library install")
set(INCLUDE_INSTALL_ROOT "include/${pel_NAME}-${pel_MAJOR_VERSION}.${pel_MINOR_VERSION}")
set(pel_INCLUDE_INSTALL_DIR "${INCLUDE_INSTALL_ROOT}/pel" CACHE STRING "Relative path for headers install")
set(pel_BIN_INSTALL_DIR "bin" CACHE STRING "Relative path for binaries install")
set(pel_CONFIG_INSTALL_DIR "share/${pel_NAME}-${pel_MAJOR_VERSION}.${pel_MINOR_VERSION}" CACHE STRING "Relative path for pelConfig files install")
set(pel_PKGCONFIG_INSTALL_DIR "${pel_LIB_INSTALL_DIR}/pkgconfig" CACHE STRING "Relative path for pkgconfig files install")
set(pel_OUTPUT_LIB_DIR "${pel_BIN_DIR}/${pel_LIB_INSTALL_DIR}" CACHE STRING "Output directory for Library")
set(pel_OUTPUT_BIN_DIR "${pel_BIN_DIR}/${pel_BIN_INSTALL_DIR}" CACHE STRING "Output directory for Binaries")
make_directory("${pel_OUTPUT_LIB_DIR}")
make_directory("${pel_OUTPUT_BIN_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${pel_OUTPUT_LIB_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${pel_OUTPUT_LIB_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${pel_OUTPUT_BIN_DIR}")

##
# this is copy paste form http://www.itk.org/Wiki/CMake_RPATH_handling
# in order to always make a full statement RPATH
##
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${pel_LIB_INSTALL_DIR}")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/${pel_LIB_INSTALL_DIR}" is_system_dir)
IF("${is_system_dir}" STREQUAL "-1")
  SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${pel_LIB_INSTALL_DIR}")
ENDIF("${is_system_dir}" STREQUAL "-1")

## -------> Add uninstall target
configure_file( "${pel_SOURCE_DIR}/cmake/uninstall.cmake.in"
  "${pel_BIN_DIR}/uninstall.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P
  "${pel_BIN_DIR}/uninstall.cmake")

## -------> Set Eigen to use only MPL2 licensed sources
set(EIGEN_MPL2_ONLY ON CACHE BOOL "Force Eigen to use only MPL2(or more permissive) licensed code")

## -------> Set External Apps variables
set(pel_EXAMPLE_APPS_BUILD ON CACHE BOOL "Build example applications")
set(pel_EXAMPLE_APPS_INSTALL ON CACHE BOOL "Install example applications on ${pel_BIN_INSTALL_DIR}")
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
include_directories(${pel_SOURCE_DIR}/include)

configure_file("${pel_SOURCE_DIR}/cmake/config_h.cmake.in"
  "${pel_BIN_DIR}/include/pel/pel_config.h")

set(pel_CONFIG_H_FILE "include/pel/pel_config.h")

include_directories(${pel_BIN_DIR}/include)

## ------> List Sources
set(srcs_base
  "src/common.cpp"
  "src/param_handler.cpp"
  "src/pose_estimation_base.cpp"
  "src/pe_brute_force.cpp"
  "src/pe_progressive_bisection.cpp"
  )
list(APPEND srcs ${srcs_base})
set(srcs_db
  "src/database/database.cpp"
  "src/database/database_io.cpp"
  "src/database/database_creator.cpp"
  )
list(APPEND srcs ${srcs_db})
set(srcs_cand
  "src/candidates/candidate_list.cpp"
  )
list(APPEND srcs ${srcs_cand})

## ------> List headers
list(APPEND incls ${pel_CONFIG_H_FILE})
set(incls_base
  "include/pel/common.h"
  "include/pel/param_handler.h"
  "include/pel/pose_estimation_base.h"
  "include/pel/pe_brute_force.h"
  "include/pel/pe_progressive_bisection.h"
  )
list(APPEND incls ${incls_base})
set(incls_cand
  "include/pel/candidates/candidate.h"
  "include/pel/candidates/candidate_list.h"
  "include/pel/candidates/target.h"
  )
list(APPEND incls ${incls_cand})
set(incls_db
  "include/pel/database/database.h"
  "include/pel/database/database_io.h"
  "include/pel/database/database_creator.h"
  )
list(APPEND incls ${incls_db})

add_library (${pel_NAME} SHARED ${srcs} ${incls})
target_link_libraries (${pel_NAME} ${LINK_LIBS})
set_target_properties(${pel_NAME} PROPERTIES
  VERSION "${pel_VERSION}"
  SOVERSION "${pel_MAJOR_VERSION}.${pel_MINOR_VERSION}"
  )
## -----> Install
install(TARGETS ${pel_NAME}
  RUNTIME DESTINATION ${pel_BIN_INSTALL_DIR}
  LIBRARY DESTINATION ${pel_LIB_INSTALL_DIR}
  ARCHIVE DESTINATION ${pel_LIB_INSTALL_DIR}
  )
install(FILES ${incls_base} DESTINATION ${pel_INCLUDE_INSTALL_DIR})
install(FILES ${incls_cand} DESTINATION "${pel_INCLUDE_INSTALL_DIR}/candidates")
install(FILES ${incls_db} DESTINATION "${pel_INCLUDE_INSTALL_DIR}/database")
install(FILES "${pel_BIN_DIR}/${pel_CONFIG_H_FILE}" DESTINATION ${pel_INCLUDE_INSTALL_DIR})

## -------> Make a pkg-config file for the library
set(PKG_NAME ${pel_NAME})
set(PKG_DESC ${pel_DESCRIPTION})
set(PKG_URL ${pel_URL})
set(PKG_DEPS "pcl_common-1.7 pcl_features-1.7 pcl_filters-1.7 pcl_visualizaton-1.7 pcl_geometry-1.7
  pcl_io-1.7 pcl_registration-1.7 pcl_search-1.7 pcl_kdtree-1.7 pcl_sample_consensus-1.7
  pcl_segmentation-1.7 pcl_surface-1.7")
set(pc_file ${pel_BIN_DIR}/${pel_NAME}-${pel_MAJOR_VERSION}.${pel_MINOR_VERSION}.pc)
configure_file(${pel_SOURCE_DIR}/cmake/pkgconfig.cmake.in ${pc_file} @ONLY)
install(FILES
  "${pc_file}"
   DESTINATION ${pel_PKGCONFIG_INSTALL_DIR})

## ------> Install catkin package.xml
install (FILES ${pel_SOURCE_DIR}/package.xml DESTINATION ${pel_CONFIG_INSTALL_DIR})

## ------> Configure pelConfig.cmake
configure_file("${pel_SOURCE_DIR}/cmake/pelConfig.cmake.in"
  "${pel_BIN_DIR}/pelConfig.cmake" @ONLY)
configure_file("${pel_SOURCE_DIR}/cmake/pelConfigVersion.cmake.in"
  "${pel_BIN_DIR}/pelConfigVersion.cmake" @ONLY)
install(FILES
  "${pel_BIN_DIR}/pelConfig.cmake"
  "${pel_BIN_DIR}/pelConfigVersion.cmake"
    DESTINATION ${pel_CONFIG_INSTALL_DIR})

## -------> Build example apps
if(pel_EXAMPLE_APPS_BUILD)
  ## estimator
  add_executable(pel_estimator ${pel_SOURCE_DIR}/ExampleApps/estimator_on_demand.cpp)
  target_link_libraries (pel_estimator ${pel_NAME} ${PCL_LIBRARIES})
  ## creator
  add_executable(pel_db_creator ${pel_SOURCE_DIR}/ExampleApps/database_builder.cpp)
  target_link_libraries (pel_db_creator ${pel_NAME} ${PCL_LIBRARIES})
  if(pel_EXAMPLE_APPS_INSTALL)
    install(TARGETS pel_estimator
      RUNTIME DESTINATION ${pel_BIN_INSTALL_DIR})
    install(TARGETS pel_db_creator
      RUNTIME DESTINATION ${pel_BIN_INSTALL_DIR})
  endif(pel_EXAMPLE_APPS_INSTALL)
endif(pel_EXAMPLE_APPS_BUILD)

