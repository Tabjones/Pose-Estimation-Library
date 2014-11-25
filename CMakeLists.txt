cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PEL)

set (PEL_VERSION_MAJOR  1)
set (PEL_VERSION_MINOR  0)
set (PEL_VERSION ${NPEL_VERSION_MAJOR}.${PEL_VERSION_MINOR} )
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
#for build tree
#set (CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/pel/" "${PROJECT_BINARY_DIR}")
#configure_file (PELConfig.cmake.in
#                "${PROJECT_BINARY_DIR}/PELConfig.cmake" @ONLY)
#              message(STATUS "build= ${PROJECT_BINARY_DIR}/PELConfig.cmake") 
#for install tree
set (CONF_INCLUDE_DIRS "${PEL_INSTALLCMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file (PELConfig.cmake.in
                "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/PELConfig.cmake" @ONLY )

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
