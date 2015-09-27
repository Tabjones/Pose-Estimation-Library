POSE ESTIMATION LIBRARY
=======================

Library for pose estimation of known objects, code api and user manual are available [here.](http://federicocp.bitbucket.org/pel/index.html)

# Set Up

You can build pel inside a catkin workspace with [catkin tools](http://catkin-tools.readthedocs.org/en/latest/index.html)
so that it is available to other catkin packages, or you can build and install it system wide with pure CMake.
Either path you choose you will need the following dependencies.

## Base Dependencies

+ pcl >= 1.7.2
+ Boost libraries
+ hdf5 (libhdf5-dev on Ubuntu)
+ GCC  > 4.7 (or equivalent compiler that supports -std=c++11)
+ CMake >= 2.8.3

## Install pel inside a catkin workspace

Navigate to your catkin source space (for most people it is just ~/catkin_ws/src/) then clone the project:
```
cd ~/catkin_ws/src/
git clone git@github.com:Tabjones/Pose-Estimation-Library.git
```
Then build the workspace with `catkin build`
```
cd ..
catkin build
```
Done.

## Install pel system wide

Clone the project wherever you want:
```
git clone git@github.com:Tabjones/Pose-Estimation-Library.git pel
cd pel
```
Make a build directory, for out-of-source build:
```
mkdir build
cd build
```
Configure CMake with defaults:
```
cmake ..
```
Or check and change variables:
```
ccmake ..
```
Then build and install
```
make
sudo make install
```
Done.

# Build your own program and link it against pel
To link a project against pel, its CMakeLists.txt must contain the following lines:
```
find_package (pel)
include_directories(${pel_INCLUDE_DIRS})
link_directoriers(${pel_LIBRARY_DIRS})
target_link_libraries (>your_program< ${pel_LIBRARIES} )
```
Example programs are available into /ExampleApps folder and are built and installed by default.

### Mirrors
This project is mirrored on:

* [Github](https://github.com/Tabjones/Pose-Estimation-Library).
* [Bitbucket](https://bitbucket.org/Tabjones/pose-estimation-library).
* [Gitlab](https://gitlab.com/fspinelli/Pose-Estimation-Library).
