/* This file implementes the Pose Estimation interface, thus contains definitions,
 * for the interface declarations look in PoseEstimation_interface.h
 */
#ifndef __INTERFACE_HPP_INCLUDED__
#define __INTERFACE_HPP_INCLUDED__

#include <pcl/io/pcd_io.h>
#include <pcl/common/norms.h>
#include <pcl/common/time.h>
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
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <stdexcept>
#include <pcl/common/centroid.h>
//Definition header
#include "PoseEstimation_interface.h"

#define D2R 0.017453293 //degrees to radians conversion

using namespace pcl::console;
using namespace boost;
using namespace boost::filesystem;

/* Class Candidate Implementation */

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

/* Class PoseEstimation Implementation */

PoseEstimation::PoseEstimation ()
{
  vp_supplied_ = query_set_ = candidates_found_ = refinement_done_ = false;
  feature_count_ = 4;
  params_["verbosity"]=1;
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
  params_["mlsPointDensity"]=250;
  params_["mlsPolyFit"]=1;
  params_["mlsSearchRadius"]=0.03f;
  params_["filterMeanK"]=50;
  params_["filterStdDevMulThresh"]=3;
  params_["neRadiusSearch"]=0.015;
  params_["useSOasViewpoint"]=1;
  params_["computeViewpointFromName"]=0;
}

void PoseEstimation::setParam(string& key, float value)
{
  if (value < 0)
  {
    if (params_["verbosity"]>0)
      print_warn("[setParam]\tParameter '%s' has a negative value (%f), ignoring...\n", key.c_str(), value);
    exit;
  }
  params_[key]=value;
  //Check if key was a valid one, since the class has 24 parameters, if one was mispelled now we have 25 
  if (params_.size() != 24)
  {
    if (params_["verbosity"]>0)
      print_warn("[setParam]\tInvalid key parameter '%s', ignoring...\n", key.c_str());
    params_.erase(key);
  }
  else if (params_["verbosity"]>1)
    print_info("[setParam]\tSetting parameter: %s=%f\n",key.c_str(),value);
  //Recheck how many features we want
  int count(0);
  if (params_["useVFH"]>=1)
    count++;
  if (params_["useESF"]>=1)
    count++;
  if (params_["useCVFH"]>=1)
    count++;
  if (params_["useOURCVFH"]>=1)
    count++;
  feature_count_ = count;
}
void PoseEstimation::setParam_ (string& key, string& value)
{
  float f;
  try
  {
    f = stof(value);
  }
  catch (const std::invalid_argument& ia)
  {
    if (params_["verbosity"] > 0)
      print_warn("[setParam_]\tInvalid %s=%s : %s \n", key.c_str(), value.c_str(), ia.what());
  }
  catch (const std::out_of_range& oor)
  {
    if (params_["verbosity"] > 0)
      print_warn("[setParam]\tInvalid %s=%s : %s \n", key.c_str(), value.c_str(), oor.what());
  }
  setParam(key, f); 
}

void PoseEstimation::initParams(path config_file)
{ 
  if ( exists(config_file) && is_regular_file(config_file))   
  {
    if (extension(config_file)==".conf") 
    {
      ifstream file (config_file.string().c_str());
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
            //split the line to get a key and a token
            split(vst, line, boost::is_any_of("="), boost::token_compress_on);
            if (vst.size()!=2)
            {
              if (params_["verbosity"]>0)
                print_warn("[initParams]\tInvalid configuration line (%s), ignoring... Must be [Token]=[Value]\n", line.c_str());
              continue;
            }
            setParam_(vst.at(0), vst.at(1));
          }
        }//end of config file
      }
      else
        print_error("[initParams]\tCannot open config file! (%s)\n", config_file.string().c_str());
    }  
    else
      print_error("[initParams]\tConfig file provided (%s) has no valid extension! (must be .conf)\n", config_file.string().c_str());
  }
  else
    print_error("[initParams]\tPath to Config File is not valid ! (%s)\n", config_file.string().c_str());
}

void PoseEstimation::filtering_()
{
  StopWatch timer;
  if (params_["verbosity"] >1)
  {
    print_info("[filtering]\t Setting Statistical Outlier Filter to preprocess query cloud...\n");
    print_info("[filtering]\t Setting mean K to %f\n", params_["filterMeanK"]);
    print_info("[filtering]\t Setting Standard Deviation multiplier to %f\n", params_["filterStdDevMulThresh"]);
    timer.reset();
  }
  PointCloud<PointXYZRGBA>::Ptr filtered (new PointCloud<PointXYZRGBA>);
  StatisticalOutlierRemoval<PointXYZRGBA> fil;
  fil.setMeanK (params_["filterMeanK"]);
  fil.setStddevMulThresh (params_["filterStdDevMulThresh"]);
  fil.setInputCloud(query_cloud_);
  fil.filter(*filtered);
  query_cloud_processed_ = filtered;
  if (params_["verbosity"]>1)
  {
    print_highlight("[filtering]\t Total time elapsed during filter: ");
    print_value("%g", timer.getTime());
    print_info(" ms\n");
  }
}

void PoseEstimation::upsampling_()
{
  StopWatch timer;
  if (params_["verbosity"] >1)
  {
    print_info("[upsampling]\t Setting MLS with Random Uniform Density to preprocess query cloud...\n");
    print_info("[upsampling]\t Setting polynomial order to %f\n", params_["mlsPolyOrder"]);
    string t = params_["mlsPolyFit"] ? "true" : "false";
    print_info("[upsampling]\t Setting polynomial fit to %s\n", t.c_str());
    print_info("[upsampling]\t Setting desired point density to %f\n", params_["mlsPointDensity"]);
    print_info("[upsampling]\t Setting search radius to %f\n", params_["mlsSearchRadius"]);
    timer.reset();
  }
  PointCloud<PointXYZRGBA>::Ptr upsampled (new PointCloud<PointXYZRGBA>);
  search::KdTree<PointXYZRGBA>::Ptr tree (new search::KdTree<PointXYZRGBA>);
  MovingLeastSquares<PointXYZRGBA, PointXYZRGBA> mls;
  if (query_cloud_processed_)
    mls.setInputCloud(query_cloud_processed_);
  else
    mls.setInputCloud(query_cloud_);
  mls.setSearchMethod(tree);
  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZRGBA, PointXYZRGBA>::RANDOM_UNIFORM_DENSITY);
  mls.setComputeNormals (false);
  mls.setPolynomialOrder(params_["mlsPolyOrder"]);
  mls.setPolynomialFit(params_["mlsPolyFit"]);
  mls.setSearchRadius(params_["mlsSearchRadius"]);
  mls.setPointDensity(params_["mlsPointDensity"]);
  mls.process(*upsampled);
  copyPointCloud(*upsampled, *query_cloud_processed_);
  if (params_["verbosity"]>1)
  {
    print_highlight("[upsampling]\t Total time elapsed during upsampling: ");
    print_value("%g", timer.getTime());
    print_info(" ms\n");
  }
}

void PoseEstimation::downsampling_()
{
  StopWatch timer;
  if (params_["verbosity"] >1)
  {
    print_info("[downsampling]\t Setting Voxel Grid to preprocess query cloud...\n");
    print_info("[downsampling]\t Setting Leaf Size to %f\n", params_["vgridLeafSize"]);
    timer.reset();
  }
  PointCloud<PointXYZRGBA>::Ptr downsampled (new PointCloud<PointXYZRGBA>);
  VoxelGrid<PointXYZRGBA> vg;
  if (query_cloud_processed_)
    vg.setInputCloud(query_cloud_processed_);
  else
    vg.setInputCloud(query_cloud_);
  vg.setLeafSize (params_["vgridLeafSize"], params_["vgridLeafSize"], params_["vgridLeafSize"]);
  vg.setDownsampleAllData (true);
  vg.filter(*downsampled);
  copyPointCloud(*downsampled, *query_cloud_processed_);
  if (params_["verbosity"]>1)
  {
    print_highlight("[downsampling]\t Total time elapsed during downsampling: ");
    print_value("%g", timer.getTime());
    print_info(" ms\n");
  }
}

bool PoseEstimation::initQuery_()
{
  //Check if a filter is needed
  if (params_["filtering"] >= 1)
    filtering_();
  //Check if upsampling is needed
  if (params_["upsampling"] >= 1)
    upsampling_();
  //Check if downsampling is needed
  if (params_["downsampling"] >= 1)
    downsampling_();
  if (! query_cloud_processed_ )
    query_cloud_processed_ = query_cloud_;

  //Check if we need ESF descriptor
  if (params_["useESF"] >= 1)
    computeESF_();
  //Check if we need Normals
  if (params_["useVFH"] >=1 || params_["useCVFH"] >=1 || params_["useOURCVFH"] >=1)
  {
    computeNormals_();
    //And consequently other descriptors
    if (params_["useVFH"] >=1)
      computeVFH_();
    if (params_["useCVFH"] >=1)
      computeCVFH_();
    if (params_["useOURCVFH"] >1)
      computeOURCVFH_();
  }
}

void PoseEstimation::setQuery(string& str, PointCloud<PointXYZRGBA>& cl)
{
  query_name_ = str;
  if (query_cloud_)
    copyPointCloud(cl, *query_cloud_);
  else
    query_cloud_ = cl.makeShared();
  if (initQuery_())
    query_set_ = true;
}
void PoseEstimation::setQuery(string& str, PointCloud<PointXYZRGBA>::Ptr clp)
{
  query_name_ = str;
  if (query_cloud_)
    copyPointCloud(*clp, *query_cloud_);
  else
    query_cloud_ = clp;
  if (initQuery_())
    query_set_ = true;
}
#endif
