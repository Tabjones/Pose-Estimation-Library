POSE ESTIMATION LIBRARY
=======================

Library for pose estimation, code api is available [here.](http://federicocp.bitbucket.org/pel/index.html)

Set Up
------------
### Quick setup
       git clone https://Tabjones@bitbucket.org/Tabjones/pel.git
       cd pel
       mkdir build && cd build
       cmake ..
       make
       sudo make install
### Dependencies
+ pcl >= 1.8
+ boost
+ hdf5 (libhdf5-dev on ubuntu)
+ GCC  >4.7 (or equivalent compiler that supports -std=c++11)
### Build your own program and link it against PEL
To link a project against PEL, source code must include the library header:

     #include <pel.h>

Also CMakeLists.txt must contain the following lines:

      find_package (PEL REQUIRED)

      include_directories(${PEL_INCLUDE_DIRS})

      target_link_libraries (your_program ${PEL_LIBRARIES} )

An example program is available into /Tests folder.