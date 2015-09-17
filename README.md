POSE ESTIMATION LIBRARY
=======================

Library for pose estimation, code api is available [here.](http://federicocp.bitbucket.org/pel/index.html)

Set Up
------------
### Quick setup
```
git clone https://Tabjones@bitbucket.org/Tabjones/pose-estimation-library.git pel
cd pel
mkdir build && cd build
cmake ..
make
sudo make install
```
### Dependencies
  + pcl >= 1.7.2
  + Boost libraries
  + hdf5 (libhdf5-dev on Ubuntu)
  + GCC  > 4.7 (or equivalent compiler that supports -std=c++11)
  + CMake 2.8
### Build your own program and link it against PEL
To link a project against PEL, CMakeLists.txt must contain the following lines:
```
find_package (PEL REQUIRED)
include_directories(${PEL_INCLUDE_DIRS})
link_directoriers(${PEL_LIBRARY_DIRS})
add_definitions(${PEL_DEFINITIONS})
target_link_libraries (>your_program< ${PEL_LIBRARIES} )
```
Example programs are available into /ExampleApps folder and are built and installed by default.
