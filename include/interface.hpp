#ifndef __INTERFACE_HPP_INCLUDED__
#define __INTERFACE_HPP_INCLUDED__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/norms.h>
#include <pcl/console/print.h>
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
#include <stdexcept>
#include <pcl/common/centroid.h>

#include "interface.h"

#define D2R 0.017453293 //degrees to radians conversion

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace boost;
using namespace boost::filesystem;
using namespace std;

/*Class Object */

Object::Object () 
{
  name_ = "UNSET";
}

Object::Object (string& str, PointCloud<PointXYZRGBA>& cl)
{
  name_ = str;
  cloud_ = cl.makeShared();
}

Object::Object (string& str, PointCloud<PointXYZRGBA>::Ptr clp)
{
  name_ = str;
  cloud_ = clp;
}

void Object::setName (string& str) 
{ 
  name_ = str; 
}

void Object::setCloud (PointCloud<PointXYZRGBA>& cl) 
{ 
  cloud_ = cl.makeShared();
}

void Object::setCloud (PointCloud<PointXYZRGBA>::Ptr clp) 
{
  cloud_ = clp;
}

void Object::getName (string& str) 
{ 
  str = name_; 
}

void Object::getCloud (PointCloud<PointXYZRGBA>& cl) 
{ 
  copyPointCloud(*cloud_, cl); 
}

void Object::getCloud (PointCloud<PointXYZRGBA>::Ptr clp) 
{ 
  clp = cloud_;
}

Object& Object::operator= (const Object& obj)
{
  name_=obj.name_;
  cloud_=obj.cloud_;
  return *this;
}

/* Class Candidate */

Candidate::Candidate ()
{
  name_ = "UNSET";
  rank_ = -1;
  distance_ = -1;
  rmse_ = -1;
  transformation_.setIdentity();
}

Candidate::Candidate (string& str, PointCloud<PointXYZRGBA>& cl)
{
  name_ = str;
  cloud_ = cl.makeShared();
  rank_ = -1;
  distance_ = -1;
  rmse_ = -1;
  transformation_.setIdentity();
}

Candidate::Candidate (string& str, PointCloud<PointXYZRGBA>::Ptr clp)
{
  name_ = str;
  cloud_ = clp;
  rank_ = -1;
  distance_ = -1;
  rmse_ = -1;
  transformation_.setIdentity();
}

void Candidate::setRank (float r) 
{ 
  rank_ = r; 
}
void Candidate::getRank (float& r) 
{ 
  r = rank_; 
}
void Candidate::setDistance (float d) 
{ 
  distance_ = d; 
}
void Candidate::getDistance (float& d) 
{ 
  d = distance_; 
}
void Candidate::setRMSE (float r) 
{ 
  rmse_ = r; 
}
void Candidate::getRMSE (float& r) 
{ 
  r = rmse_; 
}
void Candidate::getTransformation (Eigen::Matrix4f& t) 
{ 
  t = transformation_; 
}
void Candidate::setTransformation (Eigen::Matrix4f t) 
{ 
  transformation_ = t; 
}

/* Class PoseEstimation */

PoseEstimation::PoseEstimation ()
{
  howmany_features_ = 4;
  params_["useVFH"]=params_["useESF"]=params_["useCVFH"]=params_["useOURCVFH"]=1;
  params_["progBisection"]=params_["downsamspling"]=1;
  params_["vgridLeafSize"]=0.003f;
  params_["upsampling"]=params_["filtering"]=0;
  params_["kNeighbors"]=20;
  params_["maxIterations"]=200;
  params_["progItera"]=5;
  params_["progFraction"]=0.5f;
  params_["rmseThreshold"]=0.003f;
  params_["mlsPolyOrder"]=2;
  params_["mlsPointDensity"]=200;
  params_["mlsPolyFit"]=1;
  params_["mlsSearchRadius"]=0.03f;
  params_["filterMeanK"]=50;
  params_["filterStdDevMulThresh"]=3;
}
void PoseEstimation::setParam(string& param, bool value)
{
  //USE <unordered_map> !! 
}

void PoseEstimation::initParams(string cfile)
{ //rewrite with maps !!
  ifstream file (cfile.c_str());
  string line;
  if (file.is_open())
  {
    while (getline (file, line))
    {
      trim(line); //remove white spaces from line
      if (line.empty())
      {
        //do nothing empty line ...
      }
      else if (line.compare(0,1,"%") == 0 )
      {
        //do nothing comment line ...
      }
      else
      {
        vector<string> vst;
        split(vst, line, boost::is_any_of("="), boost::token_compress_on);
        if (vst.size()!=2)
        {
          print_error("[PoseEstimation]\tInvalid configuration line (%s)!! Must be [Token]=[Value]\n", line.c_str());
          continue;
        }
        if (vst.at(0).compare("useVFH")==0)
        {
          if (vst.at(1).compare("false")==0)
            use_VFH_=false;
          else if (vst.at(1).compare("true")==0)
            use_VFH_=true;
          else
            print_error("[PoseEstimation]\tInvalid 'useVFH' value (%s)! (must be true/false)\n", vst.at(1).c_str());
          continue;
        }
        else if (vst.at(0).compare("useESF")==0)
        {
          if (vst.at(1).compare("false")==0)
            use_ESF_=false;
          else if (vst.at(1).compare("true")==0)
            use_ESF_=true;
          else 
            print_error("[PoseEstimation]\tInvalid 'useESF' value (%s)! (must be true/false)\n", vst.at(1).c_str());
          continue;
        }
        else if (vst.at(0).compare("useCVFH")==0)
        {
          if (vst.at(1).compare("false")==0)
            use_CVFH_=false;
          else if (vst.at(1).compare("true")==0)
            use_CVFH_=true;
          else 
            print_error("[PoseEstimation]\tInvalid 'useCVFH' value (%s)! (must be true/false)\n", vst.at(1).c_str());
          continue;
        }
        else if (vst.at(0).compare("useOURCVFH")==0)
        {
          if (vst.at(1).compare("false")==0)
            use_OURCVFH_=false;
          else if (vst.at(1).compare("true")==0)
            use_OURCVFH_=true;
          else 
            print_error("[PoseEstimation]\tInvalid 'useOURCVFH' value (%s)! (must be true/false)\n", vst.at(1).c_str());
          continue;
        }
        else if (vst.at(0).compare("downsampling")==0)
        {
          if (vst.at(1).compare("false")==0)
            downsampling_=false;
          else if (vst.at(1).compare("true")==0)
            downsampling_=true;
          else 
            print_error("[PoseEstimation]\tInvalid 'downsampling' value (%s)! (must be true/false)\n", vst.at(1).c_str());
          continue;
        }
        else if (vst.at(0).compare("vgridLeafSize")==0)
        {
          try
          {
            float f = stof(vst.at(1));
            if (f<=0)
            {
              print_warn("[PoseEstimation]\tInvalid 'vgridLeafSize' value: non positive number, resetting to default!\n");
              f = 0.003f;
            }
            vgrid_leafSize_ = f;
          }
          catch (const std::invalid_argument& ia)
          {
            print_error("[PoseEstimation]\tInvalid 'vgridLeafSize' value: %s \n", ia.what());
          }
          catch (const std::out_of_range& oor)
          {
            print_error("[PoseEstimation]\tInvalid 'vgridLeafSize' value: %s \n", oor.what());
          }
          continue;
        }
        else if (vst.at(0).compare("upsampling")==0)
        {
          if (vst.at(1).compare("false")==0)
            upsampling_=false;
          else if (vst.at(1).compare("true")==0)
            upsampling_=true;
          else 
            print_error("[PoseEstimation]\tInvalid 'upsampling' value (%s)! (must be true/false)\n", vst.at(1).c_str());
          continue;
        }
        else if (vst.at(0).compare("filtering")==0)
        {
          if (vst.at(1).compare("false")==0)
            filter_=false;
          else if (vst.at(1).compare("true")==0)
            filter_=true;
          else 
            print_error("[PoseEstimation]\tInvalid 'filtering' value (%s)! (must be true/false)\n", vst.at(1).c_str());
          continue;
        }
        else if (vst.at(0).compare("progBisection")==0)
        {
          if (vst.at(1).compare("false")==0)
            progressive_=false;
          else if (vst.at(1).compare("true")==0)
            progressive_=true;
          else 
            print_error("[PoseEstimation]\tInvalid 'progBisection' value (%s)! (must be true/false)\n", vst.at(1).c_str());
          continue;
        }
        else if (vst.at(0).compare("kNeighbors")==0)
        {
          try
          {
            int k = stoi(vst.at(1));
            if (k<1)
            {
              print_warn("[PoseEstimation]\tInvalid 'kNeighbors' value: non positive number, resetting to default!\n");
              k = 20;
            }
            k_ = k;
          }
          catch (const std::invalid_argument& ia)
          {
            print_error("[PoseEstimation]\tInvalid 'kNeighbors' value: %s \n", ia.what());
          }
          catch (const std::out_of_range& oor)
          {
            print_error("[PoseEstimation]\tInvalid 'kNeighbors' value: %s \n", oor.what());
          }
          continue;
        }
        else if (vst.at(0).compare("maxIterations")==0)
        {
          try
          {
            int k = stoi(vst.at(1));
            if (k<1)
            {
              print_warn("[PoseEstimation]\tInvalid 'maxIterations' value: non positive number, resetting to default!\n");
              k = 200;
            }
            max_itera_ = k;
          }
          catch (const std::invalid_argument& ia)
          {
            print_error("[PoseEstimation]\tInvalid 'maxIterations' value: %s \n", ia.what());
          }
          catch (const std::out_of_range& oor)
          {
            print_error("[PoseEstimation]\tInvalid 'maxIterations' value: %s \n", oor.what());
          }
          continue;
        }
        else if (vst.at(0).compare("progItera")==0)
        {
          try
          {
            int k = stoi(vst.at(1));
            if (k<1)
            {
              print_warn("[PoseEstimation]\tInvalid 'progItera' value: non positive number, resetting to default!\n");
              k = 5;
            }
            progressive_itera_ = k;
          }
          catch (const std::invalid_argument& ia)
          {
            print_error("[PoseEstimation]\tInvalid 'progItera' value: %s \n", ia.what());
          }
          catch (const std::out_of_range& oor)
          {
            print_error("[PoseEstimation]\tInvalid 'progItera' value: %s \n", oor.what());
          }
          continue;
        }
        else if (vst.at(0).compare("progFraction")==0)
        {
          try
          {
            float f = stof(vst.at(1));
            if (f<=0 || f>=1)
            {
              print_warn("[PoseEstimation]\tInvalid 'progFraction' value: must be fraction between 0 and 1, resetting to default!\n");
              f = 0.5f;
            }
            progressive_fraction_ = f;
          }
          catch (const std::invalid_argument& ia)
          {
            print_error("[PoseEstimation]\tInvalid 'progFraction' value: %s \n", ia.what());
          }
          catch (const std::out_of_range& oor)
          {
            print_error("[PoseEstimation]\tInvalid 'progFraction' value: %s \n", oor.what());
          }
          continue;
        }
        else if (vst.at(0).compare("rmseThreshold")==0)
        {
          try
          {
            float f = stof(vst.at(1));
            if (f<0)
            {
              print_warn("[PoseEstimation]\tInvalid 'rmseThreshold' value: negative number, resetting to default!\n");
              f = 0.003f;
            }
            rmse_threshold_ = f;
          }
          catch (const std::invalid_argument& ia)
          {
            print_error("[PoseEstimation]\tInvalid 'rmseThreshold' value: %s \n", ia.what());
          }
          catch (const std::out_of_range& oor)
          {
            print_error("[PoseEstimation]\tInvalid 'rmseThreshold' value: %s \n", oor.what());
          }
          continue;
        }
        else if (vst.at(0).compare("mlsPolyOrder")==0)
        {
          try
          {
            int k = stoi(vst.at(1));
            if (k<1)
            {
              print_warn("[PoseEstimation]\tInvalid 'mlsPolyOrder' value: non positive number, resetting to default!\n");
              k = 2;
            }
            mls_polyOrder_ = k;
          }
          catch (const std::invalid_argument& ia)
          {
            print_error("[PoseEstimation]\tInvalid 'mlsPolyOrder' value: %s \n", ia.what());
          }
          catch (const std::out_of_range& oor)
          {
            print_error("[PoseEstimation]\tInvalid 'mlsPolyOrder' value: %s \n", oor.what());
          }
          continue;
        }

      }//else
    }//while
  }//if_open
  else
    print_error("[PoseEstimation]\tCan not open config file!! (%s)\n", cfile.c_str());
}


PoseEstimation::PoseEstimation(path config_file)
{
  if ( exists(config_file) && is_regular_file(config_file))   
  {
    if (extension(config_file)==".conf") 
      this->initParams(config_file.string());
    else
      print_error("[PoseEstimation]\tConfig file provided (%s) has no valid extension! (must be .conf)\n", config_file.string().c_str());
  }
  else
    print_error("[PoseEstimation]\tPath to Config File is not valid !! (%s)\n", config_file.string().c_str());
}


void PoseEstimation::setQuery(Object q)
{
 query_ = q; 
}

void PoseEstimation::setQuery(string& str, PointCloud<PointXYZRGBA>& cl)
{
  Object tmp(str,cl);
  query_=tmp;
}
void PoseEstimation::setQuery(string& str, PointCloud<PointXYZRGBA>::Ptr clp)
{
  Object tmp(str,clp);
  query_=tmp;
}
#endif
