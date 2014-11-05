/* This file describes the PoseEstimation interface, for relative implementation
 * look in PoseEstimation_interface.hpp
 */
#ifndef __INTERFACE_H_INCLUDED__
#define __INTERFACE_H_INCLUDED__

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/filesystem.hpp>
#include <string>
#include <unordered_map>

using namespace pcl;
using namespace std;

/* Class Candidate describes a single candidate object to the query */
class Candidate{
  
    string name_;
    PointCloud<PointXYZRGBA>::Ptr cloud_;
    float rank_;
    float distance_;
    float rmse_;
    Eigen::Matrix4f transformation_;

  public:
    //Default empty Constructor
    Candidate ();
    //Constructor with name and cloud
    Candidate (string& , PointCloud<PointXYZRGBA>& );
    //Constructor with name and cloud pointer
    Candidate (string& , PointCloud<PointXYZRGBA>::Ptr );
    //Set Rank of Candidate
    void setRank (float);
    //Get Candidate Rank
    void getRank (float&);
    //Set Distance of Candidate from Query
    void setDistance (float);
    //Get Distance of Candidate from Query
    void getDistance (float&);
    //Set RMSE of Candidate
    void setRMSE (float);
    //Get Candidate RMSE 
    void getRMSE (float&);
    //Get Transformation that brings the Candidate over the Query into Eigen Matrix
    void getTransformation (Eigen::Matrix4f&);
    //Set Candidate Transformation 
    void setTransformation (Eigen::Matrix4f);
};

/*
 * Class PoseEstimation implements the procedure to achieve pose estimation of a given query object.
 * The ideal procedure can be summarized as follows:
 * 1) Initialize the class parameters either with the constructors or with initParams
 * 2) Set the query object to be identified with member setQuery
 * 2b) Make sure the query viewpoint is correctly set with parameters or set it explicitly 
 *     with setQueryViewpoint before calling setQuery
 * 3) Generate the list(s) of candidates to the query with member generateLists
 * 4) Obtain the final candidate with member refineCandidates
 * 5) Print, Get or Save the final pose estimation with the corresponding member functions
 */
class PoseEstimation {
  //Map to store all parameters as key=value pairs
  unordered_map<string,float> params_;
  
  //internal counter used to count how many feature the class uses
  int feature_count_;
  
  //The name of the query to estimate
  string query_name_;
  
  //The cloud pointer that represent the query to estimate as supplied before any computation
  PointCloud<PointXYZRGBA>::Ptr query_cloud_;
  
  //Cloud pointer to query pre-processed cloud (if preprocessing is made, otherwise it's the same as query_cloud
  PointCloud<PointXYZRGBA>::Ptr query_cloud_processed_;
  
  //List of candidates to the query
  vector<Candidate> VFH_list_, ESF_list_, CVFH_list_, OURCVFH_list_, composite_list_;
  
  //Viewpoint coordinates, used in computations like VFH and Normal estimation
  float vpx_, vpy_, vpz_;
  
  //Internal parameters to check if various stages of pose estimation are completed correctly
  bool vp_supplied_, query_set_, candidates_found_, refinement_done_;
  
  //Containers that hold the query features
  PointCloud<VFHSignature308> vfh_, cvfh_, ourcvfh_;
  PointCloud<ESFSignature640> esf_;
  PointCloud<Normal> normals_;
  
  //Set a parameter of Pose Estimation from a string representing its value,
  //used internally when reading parameters from a file
  void setParam_ (string&, string&);
  
  //Initialize the Query by computing preprocessing and features, returns true if success, internal use
  bool initQuery_();
  
  //Internal member to filter the query with Statistical Outlier Removal, internal use
  void filtering_();
  
  //Internal member to upsample the query with MLS Random Uniform Density, internal use
  void upsampling_();
  
  //Internal member to downsample the query with VoxelGrid, internal use
  void downsampling_();
  
  //Internal member to compute Surface Normals of the query, internal use, return true if success
  bool computeNormals_();
  
  //Internal member to compute VFH feature of the query, internal use
  void computeVFH_();
  
  //Internal member to compute ESF feature of the query, internal use
  void computeESF_();
  
  //Internal member to compute CVFH feature of the query, internal use
  void computeCVFH_();
  
  //Internal member to compute OURCVFH feature of the query, internal use
  void computeOURCVFH_();

  public:
  //Default Empty Constructor that sets default parameters (see them in config file)
  PoseEstimation(); 
  
  //Set a parameter of the Class directly, knowing its name
  // 1) string& the parameter name to change ([key] in config file)
  // 2) float the value that key should assume
  // Example Usage:
  // PoseEstimation pe; //construct and sets default parameters
  // pe.setParam("useVFH", 0); //pe will now skip VFH computation
  // See Config File for a complete description of parameters
  void setParam (string&, float);
  
  //Initialize the class with parameters found in config file
  //  1) path the location of config file to use (i.e "config/params.config")
  //Config file must have .config extension or will be refused by the procedure
  void initParams (boost::filesystem::path);
  
  //Constructor with path to a configuration file containing parameters to set, 
  //configuration file must have extension .conf and follow certain naming convections, 
  //look at example .conf file provided for more details
  //C++11 functionality: delegate construction to empty contructor then call initParams()
  //Same exact functionality as calling empty constructor and then initParams
  PoseEstimation(boost::filesystem::path config_file) : PoseEstimation() {this->initParams(config_file);}

  //Explicitly set the query viewpoint, THIS METHOD OVERRIDES ANY VIEWPOINT PARAMETERS SET, thus the interface
  //will ignore "useSOasViewpoint" and "computeViewpointFromName" parameters regardless of their value, and will use
  //the viewpoint set this way for the computations where a viewpoint is needed (normals, VFH, CVFH ...)
  //
  //This method should be used only if the the viewpoint cannot be obtained from sensor_origin of query cloud
  //or if it cannot be computed from query name (i.e. the above cited parameters failed to set it correctly)
  //The recommended and preferred way is to use the sensor_origin field, so prepare your query cloud accordingly
  //and use the "useSOasViewpoint" parameter instead of this method
  //
  // 1)float coordinate x of the viewpoint
  // 2)float coordinate y of the viewpoint
  // 3)float coordinate z ot the viewpoint
  void setQueryViewpoint(float, float, float);
  
  //Set the poseEstimation query to be an object of point cloud and name passed
  // 1) string& the name the query should assume
  // 2) PointCloud& a point cloud containing ONLY the object to be estimated (i.e. already segmented)
  void setQuery (string&, PointCloud<PointXYZRGBA>& );
  
  //Set the poseEstimation query to be an object of pointcloud shared pointer and name passed
  // 1) string& the name the query should assume
  // 2) PointCloud::Ptr Shared pointer containing ONLY the pointcloud of the object to be estimated (i.e. already segmented)
  void setQuery (string& str, PointCloud<PointXYZRGBA>::Ptr clp);

  //Print parameters value on screen
  void printParams();
};
#endif
