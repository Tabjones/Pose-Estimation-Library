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
//#include <boost/filesystem.hpp>
//#include <boost/algorithm/string/split.hpp>
//#include <boost/algorithm/string/trim.hpp>
//#include <algorithm>
#include <string>
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
    string name;
    PointCloud<PointXYZRGBA>::Ptr cloud;
  
  public:
    //Default Constructor
    Object ();
    //Constructor with name and cloud
    Object (string& str, PointCloud<PointXYZRGBA>& cl); 
    //Constructor with name and cloud pointer
    Object (string& str, PointCloud<PointXYZRGBA>::Ptr clp); 
    //Set name of Object to str
    void setName (string& str);
    //Copy cl cloud into cloud pointer of Object
    void setCloud (PointCloud<PointXYZRGBA>& cl);
    //Copy what is pointed by clp into cloud pointer of Object
    void setCloud (PointCloud<PointXYZRGBA>::Ptr clp);
    //Get Object name into str
    void getName (string& str);
    //Copy Object cloud into cl
    void getCloud (PointCloud<PointXYZRGBA>& cl);
    //Copy Object cloud into what is pointed by clp
    void getCloud (PointCloud<PointXYZRGBA>::Ptr clp);
};

/* Class Candidate describes a single candidate object to the query */
class Candidate: public Object {
  
  protected:
    float rank;
    float distance;
    float rmse;
    Eigen::Matrix4f transformation;

  public:
    //Default Constructor
    Candidate ();
    //Constructor with name and cloud
    Candidate (string& str, PointCloud<PointXYZRGBA>& cl);
    //Constructor with name and cloud pointer
    Candidate (string& str, PointCloud<PointXYZRGBA>::Ptr clp);
    //Set Rank of Candidate to r
    void setRank (float r);
    //Get Candidate Rank into r
    void getRank (float& r);
    //Set Distance of Candidate from Query to d
    void setDistance (float d);
    //Get Distance of Candidate from Query into d
    void getDistance (float& d);
    //Set RMSE of Candidate to be r
    void setRMSE (float r);
    //Get Candidate RMSE into r
    void getRMSE (float& r);
    //Get Transformation that brings the Candidate over the Query into Eigen Matrix t
    void getTransformation (Eigen::Matrix4f& t);
    //Set Candidate Transformation to be t
    void setTransformation (Eigen::Matrix4f& t);
};

/*
 * Class PoseEstimation describes the procedure to achieve pose estimation of a given query object.
 * The ideal procedure can be summarized as follows:
 * 1) Initialize the class parameters either with the default constructor or by calling member initParams()
 * 2) Set the query object to be identified with member setQuery()
 * 3) Generate the list(s) of candidates to the query with member generateLists()
 * 4) Obtain the final candidate with member refineCandidates()
 * 5) Print, Get or Save the final pose estimation with the corresponding member functions
 */
class PoseEstimation {
  
  bool use_VFH, use_ESF, use_CVFH, use_OURCVFH;
  Object query;
  vector<Candidate> VFH_list, ESF_list, CVFH_list, OURCVFH_list, composite_list;
  int k, howmany;
  bool upsampling, downsampling, filter, progressive;

  public:
  PoseEstimation(); 
  void setQuery (Object& q); 
};
#endif

    



