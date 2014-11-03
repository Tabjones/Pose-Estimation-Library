#ifndef __INTERFACE_H_INCLUDED__
#define __INTERFACE_H_INCLUDED__

#include <iostream>
//#include <pcl/console/parse.h>
//#include <pcl/console/print.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/common/time.h>
//#include <pcl/common/norms.h>
//#include <pcl/surface/mls.h>
//#include <pcl/search/kdtree.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/filters/approximate_voxel_grid.h>
//#include <pcl/features/vfh.h>
//#include <pcl/features/esf.h>
//#include <pcl/features/fpfh_omp.h>
//#include <pcl/features/cvfh.h>
//#include <pcl/features/gfpfh.h>
//#include <pcl/features/our_cvfh.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/registration/icp.h>
//#include <pcl/registration/ndt.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/histogram_visualizer.h>
//#include <pcl/segmentation/conditional_euclidean_clustering.h>
//#include <flann/flann.h>
//#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>
//#include <boost/algorithm/string/split.hpp>
//#include <boost/algorithm/string/trim.hpp>
//#include <algorithm>
#include <string>
#include <unordered_map>
//#include <fstream>
//#include <cmath>
//#include <pcl/common/centroid.h>


using namespace pcl;
using namespace pcl::io;
//using namespace pcl::console;
//using namespace boost;
using namespace std;

/* Class Object describes an object pointcloud with a given name*/
class Object {
  
  protected:
    string name_;
    PointCloud<PointXYZRGBA>::Ptr cloud_;
  
  public:
    //Default empty Constructor
    Object ();
    //Constructor with name and cloud
    Object (string& , PointCloud<PointXYZRGBA>& ); 
    //Constructor with name and cloud pointer
    Object (string& , PointCloud<PointXYZRGBA>::Ptr ); 
    //Set name of Object
    void setName (string& );
    //make a shared pointer of passed cloud and save it into cloud_ member of class
    void setCloud (PointCloud<PointXYZRGBA>& );
    //let class cloud_ member point at what is pointed by passed argument 
    void setCloud (PointCloud<PointXYZRGBA>::Ptr);
    //Get Object name
    void getName (string& );
    //Copy what is pointed by class cloud_ member into passed cloud (no checks)
    void getCloud (PointCloud<PointXYZRGBA>& cl);
    //Let passed pointer point at what is pointed by class cloud_ (passed pointer must not be initialized with new)
    void getCloud (PointCloud<PointXYZRGBA>::Ptr);
    //Overloaded assignment operator
    Object& operator= (const Object&);
};

/* Class Candidate describes a single candidate object to the query */
class Candidate: public Object {
  
  protected:
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
 * 1) Initialize the class parameters either with the constructors or with initParams()
 * 2) Set the query object to be identified with member setQuery()
 * 3) Generate the list(s) of candidates to the query with member generateLists()
 * 4) Obtain the final candidate with member refineCandidates()
 * 5) Print, Get or Save the final pose estimation with the corresponding member functions
 */
class PoseEstimation {
  //Map to store all parameters as key=value pairs
  unordered_map<string,float> params_;
  //Object that represent the query to estimate
  Object query_;
  //List of candidates to the query
  vector<Candidate> VFH_list_, ESF_list_, CVFH_list_, OURCVFH_list_, composite_list_;
  //Viewpoint coordinates, used in computations like VFH and Normal estimation
  float vpx_, vpy_, vpz_;
  //counter that tracks how many features where used in matching
  int how_many_features_;
  
  //Set a parameter of the Class from a string representing its value,
  //used when reading from a file
  void setParam_ (string&, string&);

  public:
  //Default Empty Constructor with default parameters
  PoseEstimation(); 
  //Constructor with path to a configuration file containing parameters to set, 
  //configuration file must have extension .conf and follow certain naming convections, 
  //look at example .conf file provided for more details
  PoseEstimation(boost::filesystem::path);
  //Set a parameter of the Class directly, knowing its name
  void setParam (string&, float);
  //Initialize the class with parameters found in config file (path provided as argument)
  void initParams (boost::filesystem::path);
  //Set the poseEstimation query to be Object q
  void setQuery (Object); 
  //Set the poseEstimation query to be an object of point cloud and name passed
  void setQuery (string&, PointCloud<PointXYZRGBA>& );
  //Set the poseEstimation query to be and object of pointcloud shared pointer and name passed
  void setQuery (string& str, PointCloud<PointXYZRGBA>::Ptr clp);
};
#endif
