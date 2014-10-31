#ifndef __INTERFACE_HPP_INCLUDED__
#define __INTERFACE_HPP_INCLUDED__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/norms.h>
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/vfh.h>
#include <pcl/features/esf.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <algorithm>
#include <string>
#include <fstream>
#include <cmath>
#include <pcl/common/centroid.h>

#include "interface.h"

#define D2R 0.017453293 //degrees to radians conversion

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace boost;
using namespace std;

/*Class Object */

Object::Object () 
{
  name = "UNSET";
}

Object::Object (string& str, PointCloud<PointXYZRGBA>& cl)
{
  name = str;
  copyPointCloud (cl, *cloud);
}

Object::Object (string& str, PointCloud<PointXYZRGBA>::Ptr clp)
{
  name = str;
  copyPointCloud (*clp, *cloud);
}

void Object::setName (string& str) 
{ 
  name = str; 
}

void Object::setCloud (PointCloud<PointXYZRGBA>& cl) 
{ 
  copyPointCloud (cl, *cloud);
}

void Object::setCloud (PointCloud<PointXYZRGBA>::Ptr clp) 
{ 
  copyPointCloud (*clp, *cloud);
}

void Object::getName (string& str) 
{ 
  str = name; 
}

void Object::getCloud (PointCloud<PointXYZRGBA>& cl) 
{ 
  copyPointCloud(*cloud, cl); 
}

void Object::getCloud (PointCloud<PointXYZRGBA>::Ptr clp) 
{ 
  copyPointCloud(*cloud, *clp); 
}

/* Class Candidate */

Candidate::Candidate ()
{
  name = "UNSET";
  rank = -1;
  distance = -1;
  rmse = -1;
  transformation.setIdentity();
}

Candidate::Candidate (string& str, PointCloud<PointXYZRGBA>& cl)
{
  name = str;
  copyPointCloud (cl, *cloud);
  rank = -1;
  distance = -1;
  rmse = -1;
  transformation.setIdentity();
}

Candidate::Candidate (string& str, PointCloud<PointXYZRGBA>::Ptr clp)
{
  name = str;
  copyPointCloud (*clp, *cloud);
  rank = 0;
  distance = 0;
  rmse = 0;
  transformation.setIdentity();
}

void Candidate::setRank (float r) { 
  rank = r; 
}
void Candidate::getRank (float& r) { 
  r = rank; 
}
void Candidate::setDistance (float d) { 
  distance = d; 
}
void Candidate::getDistance (float& d) { 
  d = distance; 
}
void Candidate::setRMSE (float r) { 
  rmse = r; 
}
void Candidate::getRMSE (float& r) { 
  r = rmse; 
}
void Candidate::getTransformation (Eigen::Matrix4f& t) { 
  t = transformation; 
}
void Candidate::setTransformation (Eigen::Matrix4f& t) { 
  transformation = t; 
}

/* Class PoseEstimation */

PoseEstimation::PoseEstimation ()
{
  use_VFH = use_ESF = use_CVFH = use_OURCVFH = downsampling = progressive = true; 
  upsampling = filter = false;
  k = 20;
  howmany = 4;
}

void PoseEstimation::setQuery(Object& q)
{
  string str;
  PointCloud<PointXYZRGBA>::Ptr pt(new PointCloud<PointXYZRGBA>);
  q.getName(str);
  q.getCloud(pt);
  query.setName(str);
  query.setCloud (pt);
}

#endif
