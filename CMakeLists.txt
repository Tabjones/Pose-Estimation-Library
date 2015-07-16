cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(POLICY CMP0048)
  cmake_policy(SET CMP0048 OLD) #In case CMake version >= 3.0, retain old policy (Do not use VERSION in project() command)
endif()

project(PEL)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

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
set(LIB_INSTALL_DIR "lib")
set(INCLUDE_INSTALL_ROOT "include/${PROJECT_NAME_LOWER}-${PEL_MAJOR_VERSION}.${PEL_MINOR_VERSION}")
set(INCLUDE_INSTALL_DIR "${INCLUDE_INSTALL_ROOT}/${PROJECT_NAME_LOWER}")
set(BIN_INSTALL_DIR "bin")
set(PKGCONFIG_INSTALL_DIR "${LIB_INSTALL_DIR}/pkgconfig")
set(PEL_OUTPUT_LIB_DIR "${PEL_BINARY_DIR}/${LIB_INSTALL_DIR}")
set(PEL_OUTPUT_BIN_DIR "${PEL_BINARY_DIR}/${BIN_INSTALL_DIR}")
make_directory("${PEL_OUTPUT_LIB_DIR}")
make_directory("${PEL_OUTPUT_BIN_DIR}")

##
# this is copy paste form http://www.itk.org/Wiki/CMake_RPATH_handling
# in order to always make a full statement RPATH
##
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}" is_system_dir)
IF("${is_system_dir}" STREQUAL "-1")
  SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}")
ENDIF("${is_system_dir}" STREQUAL "-1")

## -------> Add uninstall target
configure_file( "${PEL_SOURCE_DIR}/cmake/uninstall.cmake.in"
                "${PEL_BINARY_DIR}/uninstall.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P 
                  "${PEL_BINARY_DIR}/uninstakk.cmake")

#OLDcmakelist
set (PEL_INSTALL_DIR "lib/pel-${PEL_VERSION}" CACHE PATH "Installation directory for library" )
set (PEL_INCLUDE_DIR "include/pel-${PEL_VERSION}" CACHE PATH "Installation directory for header files" )
set (PEL_INSTALLCMAKE_DIR "lib/CMake/PEL" CACHE PATH "Installation directory for CMake files")

#configure version file
configure_file (
  "${PROJECT_SOURCE_DIR}/version.cmake.in"
  "${PROJECT_SOURCE_DIR}/pel/version.h"
  )

#make relative path absolute (needed later on...)
foreach(p INSTALL INCLUDE INSTALLCMAKE)
  set (var PEL_${p}_DIR)
  if (NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

include_directories(
  "${PROJECT_SOURCE_DIR}"
  "${PROJECT_BINARY_DIR}"
  )

add_subdirectory(pel)

#==============================================
#Add all targets to the build-tree export set
export (TARGETS pel
        FILE "${PROJECT_BINARY_DIR}/pelTargets.cmake"
       )
export (PACKAGE PEL)

#Create PELConfig.cmake and PELConfigVersion files
file (RELATIVE_PATH REL_INCLUDE_DIR "${PEL_INSTALLCMAKE_DIR}"
  "${PEL_INCLUDE_DIR}" )
#for install tree
set (CONF_INCLUDE_DIRS "${PEL_INSTALLCMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file (PELConfig.cmake.in
                "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/PELConfig.cmake" @ONLY )
#for build tree
#set (CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/pel/" "${PROJECT_BINARY_DIR}")
configure_file (PELConfig.cmake.in
                "${PROJECT_BINARY_DIR}/PELConfig.cmake" @ONLY)

message(STATUS "build= ${PROJECT_BINARY_DIR}/PELConfig.cmake")
message(STATUS "inst= ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/PELConfig.cmake")
message(STATUS "conf_include= ${CONF_INCLUDE_DIRS}")

#for both
configure_file (PELConfigVersion.cmake.in
                "${PROJECT_BINARY_DIR}/PELConfigVersion.cmake" @ONLY )

#install PELConfig set of cmake files
install(FILES
        "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/PELConfig.cmake"
        "${PROJECT_BINARY_DIR}/PELConfigVersion.cmake"
        DESTINATION "${PEL_INSTALLCMAKE_DIR}")
#install export set
install(EXPORT  pelTargets
  DESTINATION "${PEL_INSTALLCMAKE_DIR}")


#Add an uninstall target
configure_file (
    "${CMAKE_CURRENT_SOURCE_DIR}/uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake"
    IMMEDIATE @ONLY
    )
add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake)
