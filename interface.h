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

//#define D2R 0.017453293 //degrees to radians conversion

using namespace pcl;
using namespace pcl::io;
//using namespace pcl::console;
//using namespace boost;
using namespace std;

class Object {
  
  protected:
    string name;
    PointCloud<PointXYZRGBA> cloud;
  
  public:
    Object ();
    Object (string& str, PointCloud<PointXYZRGBA>& cl) : name(str) { copyPointCloud(cl, cloud); } 
    void setName (string& str);
    void setCloud (PointCloud<PointXYZRGBA>& cl);
    void getName (string& str);
    void getCloud (PointCloud<PointXYZRGBA>& cl);
};

class Candidate: public Object {
  
  protected:
    float rank;
    float distance;
    float rmse;
    Eigne::Matrix4f transformation;

  public:
    Candidate ();
    Candidate (string& str, PointCloud<PointXYZRGBA>& cl) : name (str) { 
      copyPointCloud(cl, cloud);
      rank = 0;
      distance =0;
      rmse = 0;
      transformation.setIdentity();
    }
    void setRank (float r);
    void getRank (float& r);
    void setDistance (float d);
    void getDistance (float& d);
    void setRMSE (float r);
    void getRMSE (float& r);
    void getTransformation (Eigen::Matrix4f& t);
    void setTransformation (Eigen::Matrix4f& t);
};

class PoseEstimation {
  
  bool use_VFH, use_ESF, use_CVFH, use_OURCVFH;
  Object query;
  vector<Candidate> VFH_list, ESF_list, CVFH_list OURCVFH_list, composite_list;
  int k, howmany;
  bool upsampling, downsampling, filter, progressive;

  public:
  PoseEstimation() { 
    use_VFH = use_ESF = use_CVFH = use_OURCVFH = downsampling = progressive = true; 
    upsampling = filter = false;
    k = 20;
    howmany = 4;
  }
  void setQuery (Object& q) {
    query.setName( q.name );
    query.setCloud (q.cloud);
  }
  
};

    



struct candidate 
{
  string name;
  float rank;
  float dist;
  float rmse;
  PointCloud<PointXYZRGBA>::Ptr cloud;
  Eigen::Matrix4f transformation;
};

vector<candidate> candidates_vfh;
vector<candidate> candidates_esf;
vector<candidate> candidates_cvfh;
vector<candidate> candidates_ourcvfh;
vector<candidate> comp_list;

typedef vector<candidate>::iterator cand_iter;

//Program global params
float rmse_thresh (0.003f);
//float inliers_thresh (0.01f);
//float outliers_weight (2.0f);
string query_filename;
string query_name;
filesystem::path db_path;
bool mls_upsampling (false);
int k (20);
int itera (300);
bool scene (false);
bool test_f (false);
bool test_l (false);
bool test_u (false);
bool use_NDT (false);
bool visualize (false);
bool filter (false);
bool changed_reference(false);
bool no_ESF(false);
bool iterative_alignment(false);
vector<float> base_offset;
vector<float> base_orientation;
//tmp
bool show_model(false);
bool vis_scene(false);
bool clutter(false);

///////////////////////////////////////////
/////////// Show Help Function ////////////
///////////////////////////////////////////
void
showHelp (char *progName)
{ 
  cout<<std::endl;
  print_value( "***********************************************************************************\n"
               "*****************           Pose Estimation - Help             ********************\n"
               "***********************************************************************************\n");
  print_highlight( "Usage %s [Database Dir] [Query.pcd] <Options>\n", progName);
  print_info(  "[Database Dir]: as first mandatory argument must contain the path\n"
               "                to the Database directory created with createDB\n"
               "[Query.pcd]   : specifies the pointcloud to be tested against the \n"
               "                database with the pose estimation process. (mandatory)\n");
  print_info(  "Options:\n"
               "\t  -h      : Show this help.\n"
               "\t  -k <n>  : Specifies how many nearest neighbors to search during the\n" 
               "\t            matching phase. (default 20)\n"
               "\t  -rmse_t <t> : Specify the RMSE threshold to consider a candidate the good one, and stop the algorithmn. (default 0.003)\n"
               "\t  -u      : Apply an Upsampling with MLS random uniform density to the query\n"
               "\t            before processing the downsampling. Make sure the database\n"
               "\t            has been built with the same upsampling.\n"
               "\t  -f      : Apply a Statistical Outliers Removal filter with (k=50) and (std=1.6) before the\n"
               "\t            Up/Downsampling process. Make sure the database was also built with the -f option.\n"
               "\t  -s      : Specify if the query is a scene and needs to be\n" 
               "\t            presegmented and clustered.\n"
               "\t  -tf     : Use test accuracy of features and save results in txt .test files appending at bottom.\n"
               "\t            Saves the rank in with the ground truth is matched or 0 if no match.\n"
               "\t  -tu     : Use test unknown objects mode. write result of pose estimation in .test files appending at bottom.\n"
               "\t            Saves  names and under thereshold params. Not implemented in brute force alignment.\n"
               "\t  -tl     : test the accuracy of final candidate, save result in txt .test file appending at bottom.\n"
               "\t            With following syntax: <query>_<num>_<time>, where num is 0 if no candidate found, 1 if found\n"
               "\t            and matched ground truth, 2 if found and is a direct neighbourn of ground truth whitin 10 degress,\n"
               "\t            3 if found but it's a difference object/pose from ground truth. Time is the time elapsed in ms.\n"
               "\t  -g <str>: Use <str> as ground truth reference insted of file name. Only relevant if using test feature mode.\n"
               "\t  -i <n>  : Use at most <n> ICP/NDT iterations during alignment. (default 300)\n"
               "\t  -n      : Use NDT as alignment algorithmn insted of ICP, which is default.\n"
               "\t  -bt <t1,t2,t3> : Give the pose with respect to a reference frame translated by t1,t2,t3 from the origin.\n"
               "\t  -br <qw,q1,q2,q3> : Give the pose with respect to a reference frame rotated by this quaternion from the origin.\n"
               "\t  -v      : Show results in a visualization window.\n"
               "\t  -vm     : Same as -v, but replace the aligned candidate with a full object model.\n"
               "\t  -vs     : Same as -v, but replace the query with its scene. vm and vs can be both specified.\n"
               "\t  -no_esf : Ignore ESF descriptors for the composite list.\n"
               "\t  -iter_ali: Use iterative subdivision list in alignment phase, instead of trying to align all the elements.\n"
               "\t            Make small steps in alignment and take the first k/2 candidates to align for the next step, until one converges\n"
               "\t            or it remains only one candidate. (-i parameter gets set to 5, ignoring user input)\n");
  cout<<std::endl;
}

//////////////////////////////////////////////////////////////
///////// Distance function used for CVFH and OURCVFH ////////       1 + Sum_i[ min(a_i,b_i) ]
//////// as described in the related papers //////////////////  1 - ____________________________
//////////////////////////////////////////////////////////////       1 + Sum_i[ max(a_i,b_i) ]
float
MinMaxNorm (vector<float> & a, vector<float> & b)
{
  if (a.size() != b.size())
  {
    print_error("[MinMaxNorm]\t Vectors size mismatch !\n");
    return (-1);
  }
  else
  {
    int size = a.size();
    float num(1.0f), den(1.0f);
    //Process 11 items with each loop for efficency (since it is mostly applied to vectors of 308 elements)
    int i=0;
    for (; i<(size-10); i+=11)
    { 
      num += min(a[i],b[i]) + min(a[i+1],b[i+1]) + min(a[i+2],b[i+2]) + min(a[i+3],b[i+3]) + min(a[i+4],b[i+4]) + min(a[i+5],b[i+5]);
      num += min(a[i+6],b[i+6]) + min(a[i+7],b[i+7]) + min(a[i+8],b[i+8]) + min(a[i+9],b[i+9]) + min(a[i+10],b[i+10]);
      den += max(a[i],b[i]) + max(a[i+1],b[i+1]) + max(a[i+2],b[i+2]) + max(a[i+3],b[i+3]) + max(a[i+4],b[i+4]) + max(a[i+5],b[i+5]);
      den += max(a[i+6],b[i+6]) + max(a[i+7],b[i+7]) + max(a[i+8],b[i+8]) + max(a[i+9],b[i+9]) + max(a[i+10],b[i+10]);
    }
    //process last 0-10 elements
    while ( i < size)
    {
      num += min(a[i],b[i]);
      den += max(a[i],b[i]);
      ++i;
    }
    return (1 - (num/den));
  }
};

//////////////////////////////////////
//// Parse Command Line Function /////
//////////////////////////////////////
void
parseCommandLine (int argc, char *argv[])
{
  //Show help
  if (find_switch (argc, argv, "-h"))
  {
    showHelp (argv[0]);
    exit (0);
  }

  //query filename
  std::vector<int> filenames;
  filenames = parse_file_extension_argument (argc, argv, ".pcd");
  if (filenames.size () != 1)
  {
    if (filenames.size() > 1)
      print_error("[Query]\tToo many files, must specify only one!\n");
    else
      print_error ("[Query]\tFile missing or incorrect.\n");
    showHelp (argv[0]);
    exit (0);
  }
  query_filename = argv[filenames[0]];
  print_highlight("[Query]\t%s -> ", query_filename.c_str());
  vector<string> vs;
  split (vs, query_filename, boost::is_any_of("/\\.."), boost::token_compress_on);
  query_name = vs[vs.size()-2];
  parse_argument (argc, argv, "-g", query_name);
  print_value("%s\n", query_name.c_str());
  
  //database dir
  if ( !filesystem::exists (argv[1]) && !filesystem::is_directory (argv[1]) )
  {
    print_error ("[Database]\t%s Invalid directory! Need database dir as first argument!\n", argv[1]);
    showHelp (argv[0]);
    exit (0);
  }
  db_path = argv[1];
  print_highlight("[Database]\t");
  print_value("%s\n", db_path.string().c_str());

  //Program behavior
  if (find_switch (argc, argv, "-no_ESF"))
  {
    print_highlight("[Query]\tDisabling ESF descriptors ...\n");
    no_ESF = true;
  }
  
  if (find_switch (argc, argv, "-u"))
  {
    print_highlight("[Query]\tEnabling MLS Upsampling ...\n");
    mls_upsampling = true;
  }
  if (find_switch (argc, argv, "-f"))
  {
    print_highlight("[Query]\tEnabling Statistical Outliers Filtering ...\n");
    filter = true;
  }
  if (find_switch (argc, argv, "-s"))
  {
    print_highlight("[Query]\tEnabling Scene preprocessing ...\n");
    scene = true;
  }
  if (find_switch (argc, argv, "-c"))  //TMP
  {
    print_highlight("[Query]\tEnabling Cluttering Scenes ...\n");
    clutter = true;
  }
  if (find_switch (argc, argv, "-tf"))
  {
    print_highlight("[Database]\tEnabling test features mode ...\n");
    test_f = true;
  }
  if (find_switch (argc, argv, "-tu"))
  {
    print_highlight("[Database]\tEnabling test unknown objects mode ...\n");
    test_u = true;
  }
  if (find_switch (argc, argv, "-tl"))
  {
    print_highlight("[Database]\tEnabling test final candidate mode ...\n");
    test_l = true;
  }
  if (find_switch (argc, argv, "-n"))
  {
    print_highlight("[Database]\tUsing NDT instead of ICP ...\n");
    use_NDT = true;
  }
  if (find_switch (argc, argv, "-v"))
  {
    print_highlight("[Database]\tEnabling visualization mode ...\n");
    visualize = true;
  }
  if (find_switch (argc, argv, "-vm"))
  {
    print_highlight("[Database]\tEnabling visualization mode with object model ...\n");
    visualize = true;
    show_model = true;
  }
  if (find_switch (argc, argv, "-vs"))
  {
    print_highlight("[Database]\tEnabling visualization query scene ...\n");
    visualize = true;
    vis_scene = true;
  }
  if (find_switch (argc, argv, "-iter_ali"))
  {
    print_highlight("[Database]\tEnabling iterative subdivision list mode ...\n");
    iterative_alignment = true;
  }
  //General Parameters
  parse_argument (argc, argv, "-k", k);
  if ( k <= 0)
  {
    print_warn("[Database]\tInvalid 'k' parameter, must be positive! Defaulting to 20.\n");
    k = 20; //leave default
  }
  else
    print_highlight("[Database]\tUsing %d nearest neighbours ...\n", k);
  parse_argument (argc, argv, "-rmse_t", rmse_thresh );
  if ( rmse_thresh <= 0)
  {
    print_warn("[Database]\tInvalid 'rmse_t' parameter, must be positive! Defaulting to 0.003\n");
    rmse_thresh = 0.003f; //leave default
  }
  else
    print_highlight("[Database]\tUsing RMSE threshold of %g ...\n", rmse_thresh);
  parse_argument (argc, argv, "-i", itera);
  if ( itera <= 0)
  {
    print_warn("[Database]\tInvalid 'i' parameter, must be positive! Defaulting to 300.\n");
    itera = 300; //leave default
  }
  else
    print_highlight("[Database]\tUsing %d iteration step in ICP or NDT ...\n", itera);
  if (parse_x_arguments (argc, argv, "-bt", base_offset) > 0)
  {
    changed_reference = true;
    if (base_offset.size() > 3)
    {
      print_warn("[Database]\tInvalid 'bt' parameter, more than 3 arguments found, using the first 3 provided.\n");
      base_offset.resize(3);
    }
    else if (base_offset.size() < 3)
    {
      print_warn("[Database]\tInvalid 'bt' parameter, less than 3 arguments found, appending zeros to the one provided.\n");
      base_offset.resize(3, 0.0f);  
    }
    print_highlight("[Database]\tUsing Base reference offset of [%g, %g, %g]\n", base_offset[0], base_offset[1], base_offset[2]);
  }
  else
    base_offset.resize(3, 0.0f);
  if (parse_x_arguments (argc, argv, "-br", base_orientation) > 0)
  {
    changed_reference = true;
    if (base_orientation.size() > 4)
    {
      print_warn("[Database]\tInvalid 'br' parameter, more than 4 arguments found, using the first 4 provided.\n");
      base_orientation.resize(4);
    }
    else if (base_orientation.size() < 4)
    {
      print_warn("[Database]\tInvalid 'br' parameter, less than 4 arguments found, defaulting to identity.\n");
      base_orientation.resize(4, 0.0f);
      base_orientation[0] = 1.0f;
    }
    print_highlight("[Database]\tUsing Base reference quaternion of orientation [%g, %g, %g, %g]\n", base_orientation[0], base_orientation[1], base_orientation[2], base_orientation[3]);
  }
  else
  {
    base_orientation.resize(4, 0.0f);
    base_orientation[0] = 1.0f;
  }
}
/*
/////////////////////////////////////////////////////////////
///////// Conditional Euclidean Clustering Function /////////
////////// for classification during the GFPFH //////////////
/////////////////////////////////////////////////////////////
bool
enforceFPFHSimilarity (const PointXYZL& a, const PointXYZL& b, float squared_distance)
{
  int i = static_cast<int>(a.label);
  int j = static_cast<int>(b.label);
  if (L2_Norm(fpfh->points[i].histogram, fpfh->points[j].histogram, 33) <= 200.0f)
    return (true);
  else
    return (false);
}
*/
/////////////////////////////////////////////////////////////////////
///// findName function, searches a file .list for the i-th /////////
///// row and outputs the string it finds in "name", returns ////////
//////// true if it finds a name, false otherwise ///////////////////
/////////////////////////////////////////////////////////////////////
inline bool
findName (const char* file, int i, std::string &name)
{
  int n=0;
  std::string line;
  ifstream file_list (file);
  if (file_list.is_open() )
  {
    while (getline (file_list, line) )
    {
      if ( n == i ) //found the i-th row
      {  
        trim(line);
        name = line;
        return (true);
      }
      ++n;
    }
    file_list.close();
    return (false);
  }
  else
    print_error ("[findName]\tUnable to open file %s ...\n", file);
}
/////////////////////////////////////////////////////////////////////////////
///// findCandidate function, searches and array, for a candidate name //////
///// then removes that element from the array and assigns its rank in rank//
///// and its distance in dist as references, return true if a candidate ////
///// is found, false otherwise ///////////////////////////////////////////// 
/////////////////////////////////////////////////////////////////////////////
bool
findCandidate (vector<candidate> & array, string & name, float & rank, float & dist)
{
  for (cand_iter it=array.begin(); it!=array.end(); ++it)
  {
    if (it->name.compare(name) == 0)
    {
      rank = it->rank;
      dist = it->dist;
      array.erase(it);
      return (true);
    }
  }
  return (false);
}

///////////////////////////////////////////////////////////////////////
///// calculateObjective function, calculates the goal member of //////
///// a candidate, by estimating its objective function: //////////////
/////// Obj_func = #inliers - (w)*#outliers     w is a weight /////////
///////////////////////////////////////////////////////////////////////
/*
void
calculateObjective (PointCloud<PointXYZRGBA>::Ptr query, candidate & c, float thresh, float weight)
{
  search::KdTree<PointXYZRGBA>::Ptr tree (new search::KdTree<PointXYZRGBA>);
  if (!c.cloud)
  {
    PointCloud<PointXYZRGBA> cl;
    c.cloud = cl.makeShared();
    c.transformation.setIdentity();
    if (loadPCDFile ((db_path.string() + "Clouds/" + c.name + ".pcd"), *c.cloud) <0)
    {
      print_error ("[Database]\tError loading candidate cloud.\n");
      exit(0);
    }
  }
  tree->setInputCloud(c.cloud);
  int inliers(0), outliers(0);
  vector<int> k_ind;
  vector<float> k_dist;
  k_ind.resize(1);
  k_dist.resize(1);
  for (size_t i=0; i < query->points.size(); ++i)
  {
    tree->nearestKSearch(query->points[i], 1, k_ind, k_dist);
    if (sqrt(k_dist[0]) <= thresh)
      ++inliers;
    else
      ++outliers;
  }
  c.goal = (inliers - (weight*outliers))*(101-c.rank)/100;
}
*/
/////////////////////////////////////////////////////////////
////// Clustered Features match function for matching ///////
////// of objects with CVFH or OURCVFH features /////////////
/////////////////////////////////////////////////////////////
void
clusteredMatch (vector<vector<float> > & query_hist, flann::Matrix<float> & db_hist, const char* ind_fname, vector<candidate> & list, int k)
{
  int clusters = query_hist.size(); 
  int tot_db_clusters = db_hist.rows;
  ifstream file_list (ind_fname);
  if (file_list.is_open() )
  {
    int num;
    for(size_t i=0; i<tot_db_clusters; i+=num)
    {
      string line;
      getline (file_list, line); 
      vector<string> vst;
      split (vst, line, boost::is_any_of("_"), boost::token_compress_on);
      num = stoi(vst.at(vst.size()-1));
      candidate cand;
      cand.name = vst.at(vst.size()-3) + "_" + vst.at(vst.size()-2);
      cand.rmse = 0;
      cand.transformation.setIdentity();
      float dist = 0;
      for (size_t j=0; j < clusters; ++j)
      {
        vector<float> tmp_dist;
        for (size_t n=0; n<num; ++n)
        {
          vector<float> hist;
          for (size_t m=0; m<308; ++m)
            hist.push_back(db_hist[i+n][m]);
          tmp_dist.push_back( MinMaxNorm (query_hist[j], hist) );
        }
        dist += *min_element(tmp_dist.begin(), tmp_dist.end());
      }
      cand.dist = dist;
      list.push_back(cand);
    }
    if ( list.size() >= k )
    {
      sort(list.begin(), list.end(),
        [](candidate const & a, candidate const & b)
        {
          return (a.dist < b.dist);
        });
      list.resize(k);
      for (size_t i=0; i<k; ++i)
      {
        list[i].rank = i+1;
        list[i].dist = (list[i].dist - list[0].dist)/(list[k-1].dist - list[0].dist); //normalize distances to 0-1
      }
    }
    else
    {
      print_error ("[clusteredMatch]\tNot enought candidates to select in list (k is too big) ...\n");
      exit (0);
    }
  }
  else
  {
    print_error ("[clusteredMatch]\tUnable to open file %s ...\n", ind_fname);
    exit (0);
  }
}
//////////////////////////////////////////////////////////////////////
//// ICPiteration function, do <n> ICP iterations and returns ////////
//// the final transformation and the fitness score as reference /////
//////  in the candidate, returns true if converged   ////////////////
//////////////////////////////////////////////////////////////////////
bool
ICPiteration (candidate & c, PointCloud<PointXYZRGBA>::Ptr target, int n, Eigen::Matrix4f & guess)
{
  IterativeClosestPoint<PointXYZRGBA, PointXYZRGBA> icp;
  PointCloud<PointXYZRGBA>::Ptr cloud_aligned (new PointCloud<PointXYZRGBA>);
  icp.setInputSource (c.cloud);
  icp.setInputTarget (target);
  //icp.setUseReciprocalCorrespondences (true);
  // First termination criteria
  icp.setMaximumIterations (n); //num iterations has reached this value
  // Second termination criteria
  icp.setTransformationEpsilon (1e-6); //minimum difference between two consecutive transformations step is smaller than this
  // Third termination criteria
  icp.setEuclideanFitnessEpsilon (1e-8); //sum of square euclidean distances between points is smaller than this
  // Perform alignment
  icp.align (*cloud_aligned, guess);
  //copyPointCloud(*cloud_aligned, *c.cloud);
  c.transformation = icp.getFinalTransformation();
  c.rmse = sqrt(icp.getFitnessScore());
  return icp.hasConverged();
}
//////////////////////////////////////////////////////////////////////
//// NDTiteration function, do <n> NDT iterations and returns ////////
//// the final transformation and the fitness score as reference /////
//////  in the candidate, returns true if converged   ////////////////
//////////////////////////////////////////////////////////////////////
bool
NDTiteration (candidate & c, PointCloud<PointXYZRGBA>::Ptr target, int n, Eigen::Matrix4f & guess)
{
  NormalDistributionsTransform<PointXYZRGBA, PointXYZRGBA> ndt;
  PointCloud<PointXYZRGBA>::Ptr cloud_aligned (new PointCloud<PointXYZRGBA>);
  ndt.setInputSource (c.cloud);
  ndt.setInputTarget (target);
  ndt.setResolution(0.015);
  ndt.setStepSize(0.2);
  // First termination criteria
  ndt.setMaximumIterations (n); //num iterations has reached this value
  // Second termination criteria
  ndt.setTransformationEpsilon (1e-6); //minimum difference between two consecutive transformations step is smaller than this
  // Third termination criteria
  ndt.setEuclideanFitnessEpsilon (1e-8); //sum of euclidean distances between points is smaller than this
  // Perform alignment
  ndt.align (*cloud_aligned, guess);
  //copyPointCloud(*cloud_aligned, *c.cloud);
  c.transformation = ndt.getFinalTransformation();
  c.rmse = sqrt(ndt.getFitnessScore());
  return ndt.hasConverged();
}


////////////////////////////////////////////////////
//////////////////  Main  //////////////////////////
////////////////////////////////////////////////////
int
main (int argc, char *argv[])
{
  StopWatch global_timer;
  global_timer.reset();
  parseCommandLine (argc, argv);

  vector<PointCloud<PointXYZRGBA> > queries;
  PointCloud<PointXYZRGBA> scene_cloud;
  vector<PointCloud<Normal> > queries_normals;
  StopWatch timer;
  //
  //  Load cloud
  //
  if (loadPCDFile (query_filename, scene_cloud) < 0)
  {
    print_error ("[Query]\tError loading query cloud.\n" );
    showHelp (argv[0]);
    return (-1);
  }
  //
  // Scene preprocessing
  //
  if (scene)
  {
    //TODO Segmentation
  }
  else
  {
    queries.resize(1);
    queries_normals.resize(1);
    copyPointCloud(scene_cloud, queries[0]);
  }
  search::KdTree<PointXYZRGBA>::Ptr tree (new search::KdTree<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr output (new PointCloud<PointXYZRGBA>);
  //
  // Outliers Filtering
  //
  if (filter)
  {
    for (size_t i=0; i< queries.size(); ++i)
    {
      timer.reset();
      print_highlight ("[Query %d]\tFiltering Outliers ...\t\t\t", i+1);
      StatisticalOutlierRemoval<PointXYZRGBA> filter;
      filter.setMeanK (50);
      filter.setStddevMulThresh (3);
      filter.setInputCloud(queries[i].makeShared());
      filter.filter(*output);
      copyPointCloud(*output, queries[i]);
      print_value ("%g", timer.getTime());
      print_info (" ms\n");
    }
  }
  //
  // Downsample the scene 
  //
  for (size_t i=0; i < queries.size(); ++i)
  {
    timer.reset();
    if (mls_upsampling)
    {
      print_highlight ("[Query %d]\tUpsampling and downsampling ...\t\t", i+1);
      MovingLeastSquares<PointXYZRGBA, PointXYZRGBA> mls;
      mls.setInputCloud (queries[i].makeShared());
      mls.setSearchMethod (tree);
      mls.setUpsamplingMethod (MovingLeastSquares<PointXYZRGBA, PointXYZRGBA>::RANDOM_UNIFORM_DENSITY);
      mls.setComputeNormals (false);
      mls.setPolynomialOrder (2);
      mls.setPolynomialFit (true);
      mls.setSearchRadius (0.03); //3cm
      mls.setPointDensity(250);
      mls.process (*output); //Process Upsampling
      copyPointCloud (*output, queries[i]);  
      StatisticalOutlierRemoval<PointXYZRGBA> filter;
      filter.setMeanK (50);
      filter.setStddevMulThresh (4);
      filter.setInputCloud(queries[i].makeShared());
      filter.filter(*output);
      copyPointCloud(*output, queries[i]);

    }
    VoxelGrid<PointXYZRGBA> vg;
    vg.setInputCloud (queries[i].makeShared());
    vg.setLeafSize (0.003, 0.003, 0.003); //downsample to 3 mm
    vg.setDownsampleAllData (true);
    vg.filter (*output);
    copyPointCloud (*output, queries[i]);
    if (!mls_upsampling)
      print_highlight ("[Query %d]\tDownsampling ...\t\t\t", i+1);
    print_value ("%g", timer.getTime());
    print_info (" ms\n");
  }
  //
  //  Compute Normals
  //
  vector<string> vs;
  split (vs, query_filename, boost::is_any_of("/\\.._"), boost::token_compress_on);
  int longitude = stoi(vs[vs.size()-2]);
  float vx,vy,vz; //viewpoint coordinates
  vx = cos(22*D2R)*sin(longitude*D2R);
  vy = sin(22*D2R);
  vz = cos(22*D2R)*cos(longitude*D2R);
  for (size_t i=0; i < queries.size(); ++i)
  {

    timer.reset();
    print_highlight ("[Query %d]\tComputing Normals ...\t\t\t", i+1);
    NormalEstimationOMP<PointXYZRGBA, Normal> ne;
    ne.setSearchMethod(tree);
    ne.setRadiusSearch (0.015); //1.5cm
    ne.setInputCloud (queries[i].makeShared() );
    ne.setViewPoint (vx, vy, vz);
    ne.setNumberOfThreads (0); //autoallocation
    ne.compute (queries_normals[i]);
    print_value ("%g", timer.getTime());
    print_info (" ms\n");
  }
  //
  // Load the Database from disk
  //
  timer.reset();
  print_highlight("[Database]\tLoading histograms ...\t\t\t");
  cout<<std::flush;
  flann::Matrix<float> vfh, esf, cvfh, ourcvfh; //,gfpfh;
  if (filesystem::is_regular_file(db_path.string() + "/vfh.h5") && filesystem::extension(db_path.string() + "/vfh.h5") == ".h5")
    load_from_file (vfh, db_path.string() + "/vfh.h5", "VFH Histograms");
  else
  { 
    print_error("\nThe vfh.h5 file is incorrect. Please run createDB program and make sure\n"
                "the files are created correctly in the Database folder.\n");
    return(-1);
  }
  if (filesystem::is_regular_file(db_path.string() + "/esf.h5") && filesystem::extension(db_path.string() + "/esf.h5") == ".h5")
    load_from_file (esf, db_path.string() + "/esf.h5", "ESF Histograms");
  else
  { 
    print_error("\nThe esf.h5 file is incorrect. Please run createDB program and make sure\n"
                "the files are created correctly in the Database folder.\n");
    return(-1);
  }
  if (filesystem::is_regular_file(db_path.string() + "/cvfh.h5") && filesystem::extension(db_path.string() + "/cvfh.h5") == ".h5")
    load_from_file (cvfh, db_path.string() + "/cvfh.h5", "CVFH Histograms");
  else
  { 
    print_error("\nThe cvfh.h5 file is incorrect. Please run createDB program and make sure\n"
                "the files are created correctly in the Database folder.\n");
    return(-1);
  }
  if (filesystem::is_regular_file(db_path.string() + "/ourcvfh.h5") && filesystem::extension(db_path.string() + "/ourcvfh.h5") == ".h5")
    load_from_file (ourcvfh, db_path.string() + "/ourcvfh.h5", "OURCVFH Histograms");
  else
  { 
    print_error("\nThe ourcvfh.h5 file is incorrect. Please run createDB program and make sure\n"
                "the files are created correctly in the Database folder.\n");
    return(-1);
  }
  /*
  if (filesystem::is_regular_file(db_path.string() + "/gfpfh.h5") && filesystem::extension(db_path.string() + "/gfpfh.h5") == ".h5")
    load_from_file (gfpfh, db_path.string() + "/gfpfh.h5", "GFPFH Histograms");
  else
  { 
    print_error("\nThe gfpfh.h5 file is incorrect. Please run createDB program and make sure\n"
                "the files are created correctly in the Database folder.\n");
    return(-1);
  }
  */
  print_value("%g", timer.getTime());
  print_info(" ms\n");
  timer.reset();
  print_highlight("[Database]\tLoading indexes ...\t\t\t");
  cout<<std::flush;
  if (!filesystem::is_regular_file(db_path.string() + "/vfh.idx") || !(filesystem::extension (db_path.string() + "/vfh.idx") == ".idx"))
  { 
    print_error("\nThe vfh.idx file is incorrect. Please run createDB program and make sure\n"
                "the files are created correctly in the Database folder.\n");
    return(-1);
  }
  Index<ChiSquareDistance<float> > vfh_index (vfh, SavedIndexParams(db_path.string() + "/vfh.idx"));
  vfh_index.buildIndex();
  if (!filesystem::is_regular_file(db_path.string() + "/esf.idx") || !(filesystem::extension (db_path.string() + "/esf.idx") == ".idx"))
  { 
    print_error("\nThe esf.idx file is incorrect. Please run createDB program and make sure\n"
                "the files are created correctly in the Database folder.\n");
    return(-1);
  }
  Index<flann::L2<float> > esf_index (esf, SavedIndexParams(db_path.string() + "/esf.idx"));
  esf_index.buildIndex();
/*
  if (!filesystem::is_regular_file(db_path.string() + "/cvfh.idx") || !(filesystem::extension (db_path.string() + "/cvfh.idx") == ".idx"))
  { 
    print_error("\nThe cvfh.idx file is incorrect. Please run createDB program and make sure\n"
                "the files are created correctly in the Database folder.\n");
    return(-1);
  }
  Index<CustomDistance<float> > cvfh_index (cvfh, SavedIndexParams(db_path.string() + "/cvfh.idx"));
  cvfh_index.buildIndex();
  if (!filesystem::is_regular_file(db_path.string() + "/ourcvfh.idx") || !(filesystem::extension (db_path.string() + "/ourcvfh.idx") == ".idx"))
  { 
    print_error("\nThe ourcvfh.idx file is incorrect. Please run createDB program and make sure\n"
                "the files are created correctly in the Database folder.\n");
    return(-1);
  }
  Index<CustomDistance<float> > ourcvfh_index (ourcvfh, SavedIndexParams(db_path.string() + "/ourcvfh.idx"));
  ourcvfh_index.buildIndex();
  if (!filesystem::is_regular_file(db_path.string() + "/gfpfh.idx") || !(filesystem::extension (db_path.string() + "/gfpfh.idx") == ".idx"))
  { 
    print_error("\nThe gfpfh.idx file is incorrect. Please run createDB program and make sure\n"
                "the files are created correctly in the Database folder.\n");
    return(-1);
  }
  Index<ChiSquareDistance<float> > gfpfh_index (gfpfh, SavedIndexParams(db_path.string() + "/gfpfh.idx"));
  gfpfh_index.buildIndex();
  */
  print_value ("%g", timer.getTime());
  print_info (" ms\n");
  //
  // Compute all the Query histograms and save them in flann Matrix
  //
  flann::Matrix<float> vfh_query (new float[queries.size()*308], queries.size(), 308);
  flann::Matrix<float> esf_query (new float[queries.size()*640], queries.size(), 640);
  vector<vector<vector<float> > > cvfh_query, ourcvfh_query;
  for (size_t i=0; i < queries.size(); ++i)
  {
    //VFH
    PointCloud<VFHSignature308>::Ptr vfh_hist (new PointCloud<VFHSignature308>);
    timer.reset();
    print_highlight ("[Query %d]\tComputing VFH ...\t\t\t", i+1);
    VFHEstimation<PointXYZRGBA, Normal, VFHSignature308> vfhE;
    vfhE.setSearchMethod(tree);
    vfhE.setInputCloud (queries[i].makeShared() );
    vfhE.setViewPoint (vx, vy, vz);
    vfhE.setInputNormals (queries_normals[i].makeShared() );
    vfhE.compute (*vfh_hist);
    for (size_t j=0; j < vfh_query.cols; ++j)
      vfh_query[i][j]= vfh_hist->points[0].histogram[j];
    print_value ("%g", timer.getTime());
    print_info (" ms\n");
    //ESF
    if (!no_ESF)
    {
      PointCloud<ESFSignature640>::Ptr esf_hist (new PointCloud<ESFSignature640>);
      timer.reset();
      print_highlight ("[Query %d]\tComputing ESF ...\t\t\t", i+1);
      ESFEstimation<PointXYZRGBA, ESFSignature640> esfE;
      esfE.setSearchMethod(tree);
      esfE.setInputCloud (queries[i].makeShared() );
      esfE.compute (*esf_hist);
      for (size_t j=0; j < esf_query.cols; ++j)
        esf_query[i][j]= esf_hist->points[0].histogram[j];
      print_value ("%g", timer.getTime());
      print_info (" ms\n");
    }
    //CVFH
    timer.reset();
    print_highlight ("[Query %d]\tComputing CVFH ...\t\t\t", i+1);
    CVFHEstimation<PointXYZRGBA, Normal, VFHSignature308> cvfhE;
    PointCloud<VFHSignature308>::Ptr cvfh_hist (new PointCloud<VFHSignature308>);
    cvfhE.setInputCloud (queries[i].makeShared() );
    cvfhE.setSearchMethod (tree);
    cvfhE.setViewPoint (vx, vy, vz);
    cvfhE.setInputNormals (queries_normals[i].makeShared() );
    //Set maximum allowable deviation of the normals, in the region segmentation step
    cvfhE.setEPSAngleThreshold(7.5*D2R); // 7.5 degrees according to paper
    //Set maximum disparity between curvatures, in the region segmentation step
    cvfhE.setCurvatureThreshold(0.025f); //this value is from related paper
    //Set to true to normalize the bins of the histogram using the total number of points.
    //Doing this will render cvfh invariant to scale, but the authors encourage the opposite.
    cvfhE.setNormalizeBins(false); //leaving it false for now
    //Clustering parameters
    cvfhE.setClusterTolerance (0.01f); //1 cm all points should have a density of 3 mm
    cvfhE.setMinPoints (50); //at least 50 points to consider a cluster to be valid (as stated in the paper)
    //Compute method
    cvfhE.compute (*cvfh_hist);
    vector<vector<float> > q_hists;
    for (size_t n=0; n < cvfh_hist->points.size(); ++n)
    {
      vector<float> hist;
      for (size_t m=0;  m < 308 ; ++m)
        hist.push_back(cvfh_hist->points[n].histogram[m]);
      q_hists.push_back(hist);
    }
    cvfh_query.push_back(q_hists);
    print_value ("%g", timer.getTime());
    print_info (" ms\n");
    //OURCVFH
    timer.reset();
    print_highlight ("[Query %d]\tComputing OURCVFH ...\t\t\t", i+1);
    OURCVFHEstimation<PointXYZ, Normal, VFHSignature308> ourcvfhE;
    search::KdTree<PointXYZ>::Ptr tree_our (new search::KdTree<PointXYZ>);
    PointCloud<VFHSignature308>::Ptr ourcvfh_hist (new PointCloud<VFHSignature308>);
    PointCloud<PointXYZ>::Ptr input (new PointCloud<PointXYZ>);
    copyPointCloud(queries[i], *input);
    ourcvfhE.setInputCloud (input);
    ourcvfhE.setInputNormals (queries_normals[i].makeShared() );
    ourcvfhE.setSearchMethod (tree_our);
    ourcvfhE.setViewPoint (vx, vy, vz);
    //Set to true to normalize the bins of the histogram using the total number of points.
    //Doing this will render ourcvfh invariant to scale, but the authors encourage the opposite.
    ourcvfhE.setNormalizeBins(false); //leaving it false for now
    //ourcvfhE.setRadiusNormals (0.015); //1.5cm used in normal computation step
    //Set maximum allowable deviation of the normals, in the region segmentation step
    ourcvfhE.setEPSAngleThreshold(7.5f*D2R);  
    //Set maximum disparity between curvatures, in the region segmentation step
    ourcvfhE.setCurvatureThreshold(0.025f); //according to paper
    //Clustering parameters
    ourcvfhE.setClusterTolerance (0.01f); //1 cm all points should have a density of 3 mm
    ourcvfhE.setMinPoints (50); //at least 30 points to consider a cluster to be valid
    // Set the minimum axis ratio between the SGURF axes. At the disambiguation phase,
	  // this will decide if additional Reference Frames need to be created, if ambiguous.
  	ourcvfhE.setAxisRatio(0.95);
    //Set the min disambiguation axis value to generate several SGURFs for the cluster when disambiguition is difficult.
    //ourcvfhE.setMinAxisValue(0.01);
    //Set the refinement factor for the clusters
    ourcvfhE.setRefineClusters (1.0); //dont refine clusters
    //Compute method
    ourcvfhE.compute ( *ourcvfh_hist );
    vector<vector<float> > qo_hists;
    for (size_t n=0; n < ourcvfh_hist->points.size(); ++n)
    {
      vector<float> hist;
      for (size_t m=0;  m < 308 ; ++m)
        hist.push_back(ourcvfh_hist->points[n].histogram[m]);
      qo_hists.push_back(hist);
    }
    ourcvfh_query.push_back(qo_hists);
    print_value ("%g", timer.getTime());
    print_info (" ms\n");
  }

  //
  // Matching Phase
  //
  //VFH
  timer.reset();
  print_highlight("[Database]\tMatching VFH queries ...\n");
  flann::Matrix<int> vfh_match_ind (new int[vfh_query.rows * k], vfh_query.rows, k);
  flann::Matrix<float> vfh_match_dist (new float[vfh_query.rows * k], vfh_query.rows, k);
  vfh_index.knnSearch (vfh_query, vfh_match_ind, vfh_match_dist, k, SearchParams(256) );
  for (size_t i=0; i < vfh_query.rows; ++i)
  {
    print_highlight("[Query %d]\tVFH matched:\n", i+1);
    bool gt_match (false);
    for (size_t j=0; j < k; ++j)
    {
      string match;
      if (!findName( (db_path.string() + "/names.list").c_str(), vfh_match_ind[i][j], match))
        print_error("[Database]\tCant find matches in list file ...\n");
      candidate c;
      c.name = match;
      c.rank = j+1;
      c.dist = (vfh_match_dist[i][j] - vfh_match_dist[i][0])/(vfh_match_dist[i][k-1] - vfh_match_dist[i][0]);
      c.rmse = 0;
      c.transformation.setIdentity();
      print_value("%20s", c.name.c_str());
      print_info("\tdistance:");
      print_value("%12g", c.dist);
      print_info("\trank:");
      print_value("%3g ", c.rank);
      if (query_name.compare(c.name) == 0)
      {
        gt_match = true;
        print_highlight(" Ground Truth Match (%s)\n",query_name.c_str());
        if(test_f)
        {
          ofstream vfh_result;
          vfh_result.open ("vfh.test", fstream::out | fstream::app);
          vfh_result<<c.rank<<"_";
          vfh_result.close();
        }
      }
      else
        cout<<endl;
      candidates_vfh.push_back(c);
    }
    if (test_f && !gt_match)
    {
      ofstream vfh_result;
      vfh_result.open ("vfh.test", fstream::out | fstream::app);
      vfh_result<<"0_";
      vfh_result.close();
    }
  }
  cout<<endl;
  print_info ("\tTotal match time: ");
  print_value ("%g", timer.getTime());
  print_info (" ms\n");
  //ESF
  if (!no_ESF)
  {
    timer.reset();
    print_highlight("[Database]\tMatching ESF queries ...\n");
    flann::Matrix<int> esf_match_ind (new int[esf_query.rows * k], esf_query.rows, k);
    flann::Matrix<float> esf_match_dist (new float[esf_query.rows * k], esf_query.rows, k);
    esf_index.knnSearch (esf_query, esf_match_ind, esf_match_dist, k, SearchParams(256) );
    for (size_t i=0; i < esf_query.rows; ++i)
    {
      print_highlight("[Query %d]\tESF matched:\n", i+1);
      bool gt_match (false);
      for (size_t j=0; j < k; ++j)
      {
        string match;
        if (!findName( (db_path.string() + "/names.list").c_str(), esf_match_ind[i][j], match))
          print_error("[Database]\tCant find matches in list file ...\n");
        candidate c;
        c.name = match;
        c.rank = j+1;
        c.dist = (esf_match_dist[i][j] - esf_match_dist[i][0])/(esf_match_dist[i][k-1] - esf_match_dist[i][0]); //normalizing distances to 0-1
        c.rmse = 0;
        c.transformation.setIdentity();
        print_value("%20s", match.c_str());
        print_info("\tdistance:");
        print_value("%12g", c.dist);
        print_info("\trank:");
        print_value("%3g ", c.rank);
        if (query_name.compare(c.name) == 0)
        {
          gt_match = true;
          print_highlight(" Ground Truth Match (%s)\n",query_name.c_str());
          if(test_f)
          {
            ofstream esf_result;
            esf_result.open ("esf.test", fstream::out | fstream::app);
            esf_result<<c.rank<<"_";
            esf_result.close();
          }
        }
        else
          cout<<endl;
        candidates_esf.push_back(c);
      }
      if (test_f && !gt_match)
      {
        ofstream esf_result;
        esf_result.open ("esf.test", fstream::out | fstream::app);
        esf_result<<"0_";
        esf_result.close();
      }
    }
    cout<<endl;
    print_info ("\tTotal match time: ");
    print_value ("%g", timer.getTime());
    print_info (" ms\n");
  }
  //CVFH
  timer.reset();
  print_highlight("[Database]\tMatching CVFH queries ...\n");
  for (size_t n=0; n < cvfh_query.size(); ++n)
  {
    vector<candidate> c_cvfh;
    clusteredMatch (cvfh_query[n], cvfh, (db_path.string()+"/cvfh.list").c_str(), c_cvfh, k);
    print_highlight("[Query %d]\tCVFH matched:\n", n+1);
    bool gt_match (false);
    for (size_t i=0; i<k; ++i)
    { 
      print_value("%20s", c_cvfh[i].name.c_str());
      print_info("\tdistance:");
      print_value("%12g", c_cvfh[i].dist);
      print_info("\trank:");
      print_value("%3g ", c_cvfh[i].rank);
      if (query_name.compare(c_cvfh[i].name) == 0)
      {
        gt_match = true;
        print_highlight(" Ground Truth Match (%s)\n",query_name.c_str());
        if (test_f)
        {
          ofstream cvfh_result;
          cvfh_result.open("cvfh.test", fstream::out | fstream::app);
          cvfh_result<<c_cvfh[i].rank<<"_";
          cvfh_result.close();
        }
      }
      else
        cout<<endl;
      candidates_cvfh.push_back(c_cvfh[i]);
    }
    if (test_f && !gt_match)
    {
      ofstream cvfh_result;
      cvfh_result.open("cvfh.test", fstream::out | fstream::app);
      cvfh_result<<"0_";
      cvfh_result.close();
    }
  }
  cout<<endl;
  print_info ("\tTotal match time: ");
  print_value ("%g", timer.getTime());
  print_info (" ms\n");
  //OURCVFH
  timer.reset();
  print_highlight("[Database]\tMatching OURCVFH queries ...\n");
  for (size_t n=0; n < ourcvfh_query.size(); ++n)
  {
    vector<candidate> c_ourcvfh;
    clusteredMatch (ourcvfh_query[n], ourcvfh, (db_path.string()+"/ourcvfh.list").c_str(), c_ourcvfh, k);
    print_highlight("[Query %d]\tOURCVFH matched:\n", n+1);
    bool gt_match(false);
    for (size_t i=0; i<k; ++i)
    { 
      print_value("%20s", c_ourcvfh[i].name.c_str());
      print_info("\tdistance:");
      print_value("%12g", c_ourcvfh[i].dist);
      print_info("\trank:");
      print_value("%3g ", c_ourcvfh[i].rank);
      if (query_name.compare(c_ourcvfh[i].name) == 0)
      {
        gt_match = true;
        print_highlight(" Ground Truth Match (%s)\n",query_name.c_str());
        if (test_f)
        {
          ofstream ourcvfh_result;
          ourcvfh_result.open("ourcvfh.test", fstream::out | fstream::app);
          ourcvfh_result<<c_ourcvfh[i].rank<<"_";
          ourcvfh_result.close();
        }
      }
      else
        cout<<endl;
      candidates_ourcvfh.push_back(c_ourcvfh[i]);
    }
    if (test_f && !gt_match)
    {
      ofstream ourcvfh_result;
      ourcvfh_result.open("ourcvfh.test", fstream::out | fstream::app);
      ourcvfh_result<<"0_";
      ourcvfh_result.close();
    }
  }
  cout<<endl;
  print_info ("\tTotal match time: ");
  print_value ("%g", timer.getTime());
  print_info (" ms\n");
  // computing composite list

  print_highlight("[Database]\tProducing Composite list...\n");
  timer.reset();
  for (size_t n=0; n<queries.size(); ++n)
  {
    print_highlight("[Query %d]\t Composite list:\n", n+1);
    vector<candidate> tmp_esf, tmp_cvfh, tmp_ourcvfh;
    for (size_t i=0; i<k;++i)
    {
      if(!no_ESF)
        tmp_esf.push_back(candidates_esf.at(n*k + i));
      tmp_cvfh.push_back(candidates_cvfh.at(n*k + i));
      tmp_ourcvfh.push_back(candidates_ourcvfh.at(n*k + i));
    }
    bool gt_match (false);
    for (cand_iter it=(candidates_vfh.begin()+ k*n); it!=(candidates_vfh.begin() + k*n +k); ++it)
    {
      float dist(it->dist);
      float d,r;
      if(!no_ESF)
      {
        if (findCandidate(tmp_esf, it->name, r, d))
          dist += d;
        else
          dist += 1;
      }
      if (findCandidate(tmp_cvfh, it->name, r, d))
        dist += d;
      else
        dist += 1;
      if (findCandidate(tmp_ourcvfh, it->name, r, d))
        dist += d;
      else
        dist += 1;
      candidate cand;
      cand.name = it->name;
      cand.rank = 0;
      if (!no_ESF)
        cand.dist = dist/4; 
      else
        cand.dist = dist/3;
      cand.rmse = 0;
      cand.transformation.setIdentity();
      comp_list.push_back(cand);
    }
    if (!tmp_esf.empty() && !no_ESF) //still some candidates in esf to check
      for (cand_iter it=tmp_esf.begin(); it!= tmp_esf.end(); ++it)
      {
        float dist(it->dist + 1);
        float r,d;
        if (findCandidate(tmp_cvfh, it->name, r, d))
          dist += d;
        else
          dist += 1;
        if (findCandidate(tmp_ourcvfh, it->name, r, d))
          dist += d;
        else
          dist += 1;
        candidate cand;
        cand.name = it->name;
        cand.rank = 0;
        if (!no_ESF)
          cand.dist = dist/4; 
        else
          cand.dist = dist/3;
        cand.rmse = 0;
        cand.transformation.setIdentity();
        comp_list.push_back(cand);
      }
    if (!tmp_cvfh.empty())
      for (cand_iter it=tmp_cvfh.begin(); it!= tmp_cvfh.end(); ++it)
      {
        float dist(it->dist + 2);
        float r,d;
        if (findCandidate(tmp_ourcvfh, it->name, r, d))
          dist += d;
        else
          dist += 1;
        candidate cand;
        cand.name = it->name;
        cand.rank = 0;
        if (!no_ESF)
          cand.dist = dist/4;
        else
          cand.dist = dist/3;
        cand.rmse = 0;
        cand.transformation.setIdentity();
        comp_list.push_back(cand);
      }
    if (!tmp_ourcvfh.empty()) //last check
      for (cand_iter it=tmp_ourcvfh.begin(); it!= tmp_ourcvfh.end(); ++it)
      {
        float dist(it->dist);
        candidate cand;
        cand.name = it->name;
        cand.rank = 0;
        if (!no_ESF)
          cand.dist = (dist + 3)/4; 
        else
          cand.dist = (dist + 2)/3;
        cand.rmse = 0;
        cand.transformation.setIdentity();
        comp_list.push_back(cand);
      }
    sort(comp_list.begin()+ k*n, comp_list.end(),
        [](candidate const & a, candidate const & b)
        {
          return (a.dist < b.dist);
        });  //sort them by min rank
    comp_list.resize((n+1)*k);
    for (cand_iter it=(comp_list.begin()+ n*k); it!=(comp_list.begin()+ n*k +k) ; ++it)
    {
      //store in dist the median rank calculated, and rerank the list from 1 to k for each query
      it->rank = it - comp_list.begin() +1 -n*k; //get the rank of the candidate in the list
      print_value("%20s",it->name.c_str());
      print_info("\trank:");
      print_value("%3g", it->rank);
      print_info("\tmedian distance:");
      print_value("%3g ", it->dist);
      if (query_name.compare(it->name) == 0) //match gt
      { 
        print_highlight(" Ground Truth Match (%s)\n",query_name.c_str());
        gt_match = true;
        if (test_f)
        {
          ofstream composite_result;
          composite_result.open("composite.test", fstream::out | fstream::app);
          composite_result<<it->rank<<"_";
          composite_result.close();
        }
      }
      else
        cout<<endl;
    }
    if (test_f && !gt_match)
    {
      ofstream composite_result;
      composite_result.open("composite.test", fstream::out | fstream::app);
      composite_result<<"0_";
      composite_result.close();
    }
  }
  cout<<endl;
  print_info ("\tTotal composition time: ");
  print_value ("%g", timer.getTime());
  print_info (" ms\n");
  //Alignment phase with ICP or NDT
  Eigen::Matrix<float,3,1> offset (base_offset[0], base_offset[1], base_offset[2]);
  Eigen::Quaternion<float> orientation (base_orientation[0], base_orientation[1], base_orientation[2], base_orientation[3]);
  if (!test_f) //stop the program here if we are doing the features test to save time
  {
    CentroidPoint<PointXYZ> centroid;
    for(size_t i=0; i<queries[0].points.size(); ++i)
    {
      PointXYZ pt;
      pt.x = queries[0].points[i].x;
      pt.y = queries[0].points[i].y;
      pt.z = queries[0].points[i].z;
      centroid.add(pt);
    }
    PointXYZ cen;
    centroid.get(cen);

    Eigen::Vector3f t_cen (cen.x, cen.y, cen.z);
    Eigen::Matrix3f rotation = orientation.inverse().matrix();
    Eigen::Matrix4f trasl, rot, guess, tcen, rotY;
    trasl<<Eigen::MatrixXf::Identity(3,3),-offset,Eigen::MatrixXf::Zero(1,3),1;
    rot<<rotation,Eigen::MatrixXf::Zero(3,1),Eigen::MatrixXf::Zero(1,3),1;
    tcen<<Eigen::MatrixXf::Identity(3,3),t_cen,Eigen::MatrixXf::Zero(1,3),1;
    guess =  tcen * rot * trasl;
    vector<candidate> final_cand;
    final_cand.resize(queries.size());
    for (size_t n=0; n< queries.size(); ++n)
    {
      if (!iterative_alignment) //old method
      {
        if (use_NDT)
          print_highlight("[Query %d]\tPerforming NDT alignment ...\n", n+1);
        else
          print_highlight("[Query %d]\tPerforming ICP alignment ...\n", n+1);
        timer.reset();
        bool cand_found (false);
        int cand_type (0); //6 found under rmse, 7 found over rmse
        PointCloud<PointXYZRGBA>::Ptr query_p (new PointCloud<PointXYZRGBA>);
        copyPointCloud (queries[n], *query_p);
        for(cand_iter it=(comp_list.begin()+ k*n); it!=(comp_list.begin()+ k*n +k); ++it)
        {
          PointCloud<PointXYZRGBA> cl;
          it->cloud=cl.makeShared();
          if (loadPCDFile ((db_path.string() + "Clouds/" + it->name + ".pcd"), *it->cloud) <0)
          {
            print_error ("[clusteredMatch]\tError loading candidate cloud.\n");
            exit(0);
          }  
          PointCloud<PointXYZRGBA> trans_cloud;
          if (clutter) //tmp doesnt work with changewd reference,  yet ...
          {
            vector<string> vst;
            split (vst, it->name, boost::is_any_of("_"), boost::token_compress_on);
            float rot_angle = (float)(stoi(vst[1])*D2R);
            Eigen::Matrix4f ry;
            Eigen::Matrix3f r;
            r<<cos(rot_angle), 0, sin(rot_angle), 0, 1, 0, -sin(rot_angle), 0 , cos(rot_angle);
            ry<<r, (Eigen::Vector3f() << cen.x,cen.y,cen.z).finished(),Eigen::MatrixXf::Zero(1,3),1;
            guess = ry;
          }
          transformPointCloud(*it->cloud, trans_cloud, offset, orientation);
          copyPointCloud(trans_cloud, *it->cloud);
          print_value("%20s", it->name.c_str());
          if (!use_NDT)
            ICPiteration(*it, query_p, itera, guess);
          else 
            NDTiteration(*it, query_p, itera, guess);
          print_info("\tRMSE:");
          print_value("%12g\n", it->rmse);
          if (it->rmse < rmse_thresh)
          {
            cand_found = true;
            cand_type = 6;
            PointCloud<PointXYZRGBA> cl;
            final_cand[n].name = it->name;
            final_cand[n].cloud = cl.makeShared();
            copyPointCloud ( *it->cloud, *final_cand[n].cloud);
            final_cand[n].rmse = it->rmse;
            final_cand[n].transformation = it->transformation;
            break;
          }
        }
        float time = timer.getTime();
        print_value("\t%g ", time);
        print_info("ms elapsed\n");
        if (!cand_found)
        {
          print_highlight("[Query %d] Could not find a suitable candidate...\n", n+1);
          sort(comp_list.begin() + k*n, comp_list.begin() +k*n +k,
              [](candidate const & a, candidate const & b)
              {
              return (a.rmse < b.rmse);
              });  //sort them by min rmse
          if (comp_list[k*n].rmse < rmse_thresh*10)
          {
            cand_found = true;
            cand_type = 7;
            final_cand[n].name = comp_list[k*n].name;
            PointCloud<PointXYZRGBA> cl;
            final_cand[n].cloud = cl.makeShared();
            copyPointCloud ( *comp_list[k*n].cloud, *final_cand[n].cloud);
            final_cand[n].rmse = comp_list[k*n].rmse;
            final_cand[n].transformation = comp_list[k*n].transformation;
            print_highlight("[Query %d] My guess is ",n+1);
            print_value("%s ",final_cand[n].name.c_str());
            print_info("with confidence ");
            print_value("%.2f%%\n", rmse_thresh/final_cand[n].rmse*100);
            print_info("\tTransformation is:\n");
            cout<<final_cand[n].transformation<<endl;
          }
          else
          {
            final_cand[n].name = "NoCandidate";
            final_cand[n].rmse = 0;
            final_cand[n].rank = 0;
            final_cand[n].cloud = nullptr;
            final_cand[n].transformation.setIdentity();
          }
        }
        else
        {
          print_highlight("[Query %d]\t Final candidate is: ", n+1);
          print_value("%s ", final_cand[n].name.c_str());
          print_info("RMSE: ");
          print_value("%g\n", final_cand[n].rmse);
          print_info("\tTransformation is:\n");
          cout<<final_cand[n].transformation<<endl;
        }
        if (test_l)
        {
          ofstream cand_result;
          cand_result.open ("candidate.test", fstream::out | fstream::app);
          cand_result<<query_name.c_str()<<"_";
          if (cand_found)
          {
            if (final_cand[n].name.compare(query_name) == 0 ) //same name as GT
              cand_result<<"1_";
            else
            {
              vector<string> vq, vc;
              split (vq, query_name, boost::is_any_of("_"), boost::token_compress_on);
              split (vc, final_cand[n].name, boost::is_any_of("_"), boost::token_compress_on);
              if (vq.at(0).compare(vc.at(0)) == 0  ) //name is equal
              {
                if( (stoi(vq.at(1)) == stoi(vc.at(1))+10) || (stoi(vq.at(1)) == stoi(vc.at(1))-10) ) //direct neighbourn 
                  cand_result<<"2_";
                else
                  cand_result<<"3_"; //same object 
              }
              else
                cand_result<<"4_"; //false 
            }
          }
          else
            cand_result<<"0_"; //no candidate found
          cand_result<<time<<"_";
          cand_result<<cand_type<<endl;
          cand_result.close();
        }
      }
      else //iterative alignment
      {
        float size = k;
        itera = 5; //make small steps
        if (use_NDT)
          print_highlight("[Query %d]\tPerforming NDT alignment ...\n", n+1);
        else
          print_highlight("[Query %d]\tPerforming ICP alignment ...\n", n+1);
        timer.reset();
        bool cand_found (false);
        int cand_type (0); //6 found under rmse, 7 found over rmse
        PointCloud<PointXYZRGBA>::Ptr query_p (new PointCloud<PointXYZRGBA>);
        copyPointCloud (queries[n], *query_p);
        vector<candidate> clist;
        clist.resize(k);
        copy ((comp_list.begin()+ k*n),(comp_list.begin()+ k*n +k), clist.begin()); //copy candidate list into temporary list
        while(clist.size() > 1)
        {
          for(cand_iter it=clist.begin(); it!=clist.end(); ++it)
          {
            if (clist.size()==k) //first step needs to load the clouds
            {
              PointCloud<PointXYZRGBA> cl;
              it->cloud=cl.makeShared();
              if (loadPCDFile ((db_path.string() + "Clouds/" + it->name + ".pcd"), *it->cloud) <0)
              {
                print_error ("[clusteredMatch]\tError loading candidate cloud.\n");
                exit(0);
              }  
              PointCloud<PointXYZRGBA> trans_cloud;
              transformPointCloud(*it->cloud, trans_cloud, offset, orientation);
              copyPointCloud(trans_cloud, *it->cloud);
              print_value("%20s", it->name.c_str());
              if (!use_NDT)
                ICPiteration(*it, query_p, itera, guess);
              else 
                NDTiteration(*it, query_p, itera, guess);
            }
            else
            {
              print_value("%20s", it->name.c_str());
              if (!use_NDT)
                ICPiteration(*it, query_p, itera, it->transformation);
              else 
                NDTiteration(*it, query_p, itera, it->transformation);
            }
            print_info("\tRMSE:");
            print_value("%12g\n", it->rmse);
            if (it->rmse < rmse_thresh)
            {
              cand_found = true;
              cand_type = 6;
              PointCloud<PointXYZRGBA> cl;
              final_cand[n].name = it->name;
              final_cand[n].cloud = cl.makeShared();
              copyPointCloud ( *it->cloud, *final_cand[n].cloud);
              final_cand[n].rmse = it->rmse;
              final_cand[n].transformation = it->transformation;
              break;
            }
          }
          if (cand_found)
            break;
          print_info("Reordering...\n");
          sort(clist.begin(), clist.end(),
              [](candidate const & a, candidate const & b)
              {
              return (a.rmse < b.rmse);
              });  //sort them by min rmse
          clist.resize(ceil(size/2));
          size /=2;
        }
        float time = timer.getTime();
        print_value("\t%g ", time);
        print_info("ms elapsed\n");
        if (!cand_found)
        {
          print_highlight("[Query %d] Could not find a suitable candidate...\n", n+1);
          if (clist[0].rmse < rmse_thresh*10)
          {
            cand_found = true;
            cand_type = 7;
            final_cand[n].name = clist[0].name;
            PointCloud<PointXYZRGBA> cl;
            final_cand[n].cloud = cl.makeShared();
            copyPointCloud ( *clist[0].cloud, *final_cand[n].cloud);
            final_cand[n].rmse = clist[0].rmse;
            final_cand[n].transformation = clist[0].transformation;
            print_highlight("[Query %d] My guess is ",n+1);
            print_value("%s ",final_cand[n].name.c_str());
            print_info("with confidence ");
            print_value("%.2f%%\n", rmse_thresh/final_cand[n].rmse*100);
            print_info("\tTransformation is:\n");
            cout<<final_cand[n].transformation<<endl;
          }
          else
          {
            final_cand[n].name = "NoCandidate";
            final_cand[n].rmse = 0;
            final_cand[n].rank = 0;
            final_cand[n].cloud = nullptr;
            final_cand[n].transformation.setIdentity();
          }
        }
        else
        {
          print_highlight("[Query %d]\t Final candidate is: ", n+1);
          print_value("%s ", final_cand[n].name.c_str());
          print_info("RMSE: ");
          print_value("%g\n", final_cand[n].rmse);
          print_info("\tTransformation is:\n");
          cout<<final_cand[n].transformation<<endl;
        }
        if (test_l)
        {
          ofstream cand_result;
          cand_result.open ("candidate.test", fstream::out | fstream::app);
          cand_result<<query_name.c_str()<<"_";
          if (cand_found)
          {
            if (final_cand[n].name.compare(query_name) == 0 ) //same name as GT
              cand_result<<"1_";
            else
            {
              vector<string> vq, vc;
              split (vq, query_name, boost::is_any_of("_"), boost::token_compress_on);
              split (vc, final_cand[n].name, boost::is_any_of("_"), boost::token_compress_on);
              if (vq.at(0).compare(vc.at(0)) == 0  ) //name is equal
              {
                if( (stoi(vq.at(1)) == stoi(vc.at(1))+10) || (stoi(vq.at(1)) == stoi(vc.at(1))-10) ) //direct neighbourn 
                  cand_result<<"2_";
                else
                  cand_result<<"3_"; //same object 
              }
              else
                cand_result<<"4_"; //false 
            }
          }
          else
            cand_result<<"0_"; //no candidate found
          cand_result<<time<<"_";
          cand_result<<cand_type<<endl;
          cand_result.close();
        }
        if (test_u)
        {
          ofstream cand_result;
          cand_result.open ("unknown.test", fstream::out | fstream::app);
          cand_result<<query_name.c_str()<<"_";
          cand_result<<final_cand[n].name.c_str()<<"_";
          cand_result<<cand_type<<endl;
          cand_result.close();
        }
      }
    }
    double global_time = global_timer.getTime();
    print_highlight("[Pose Estimation] Complete, total execution time: ");
    print_value("%g",global_time);
    print_info(" ms\n");
    if (test_l)
    {
      ofstream gtime;
      gtime.open ("GlobalTime.test", fstream::out | fstream::app);
      gtime<<global_time<<endl;
    }
    //   
    //  Visualization
    // 
    if (visualize)
    {
      vector<visualization::PCLVisualizer> viewers;
      viewers.resize(queries.size());
     // string scene_path("/home/tabjones/Documents/Unipi/Tesi/ObjectDB/Acquired/RotatingTable/Scenes2/"); //tmp  change accordingly if needed
      string scene_path("/home/tabjones/Documents/Unipi/Tesi/Code/PoseEstimation/"); //tmp  change accordingly if needed
      string model_path("/home/tabjones/Documents/Unipi/Tesi/ObjectDB/Models/"); //tmp  change accordingly if needed
      vector<string> vst;
      split (vst, query_name, boost::is_any_of("_"), boost::token_compress_on);
      if (vis_scene)
        loadPCDFile ( (scene_path + vst[0] + ".pcd" ).c_str(), scene_cloud); 
        //loadPCDFile ( (scene_path + vst[0] + ":Scene_" + vst[1] + ".pcd" ).c_str(), scene_cloud); 

      for (size_t n=0; n< queries.size(); ++n)
      {
        PointCloud<PointXYZRGBA>::Ptr model (new PointCloud<PointXYZRGBA>);
        split (vst, final_cand[n].name, boost::is_any_of("_"), boost::token_compress_on);
        if (show_model)
          loadPCDFile ( (model_path + vst[0] +  ".pcd" ).c_str(), *model); 
        viewers[n].setWindowName("Pose Estimation: Query " + to_string(n+1));
        viewers[n].setBackgroundColor(0,0,0);
        if ( !(final_cand[n].name.compare("NoCandidate") == 0) )
        {
          if (changed_reference)
          {
            transformPointCloud(*model, *model, offset, orientation);
            pcl::visualization::PointCloudColorHandlerCustom<PointXYZRGBA> candidate_color_handler (final_cand[n].cloud, 255, 0, 0);
            viewers[n].addPointCloud(final_cand[n].cloud, candidate_color_handler, "candidate");
            viewers[n].setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "candidate");
            viewers[n].addText(final_cand[n].name + " not aligned", 30,15, 1, 0.2, 0.3, "info2");
          }
          PointCloud<PointXYZRGBA>::Ptr trans_cloud (new PointCloud<PointXYZRGBA>);
          if (!show_model)
            transformPointCloud(*final_cand[n].cloud, *trans_cloud, final_cand[n].transformation);
          else
            transformPointCloud(*model, *trans_cloud, final_cand[n].transformation);
          pcl::visualization::PointCloudColorHandlerCustom<PointXYZRGBA> trans_color_handler (trans_cloud, 0, 255, 0);
          viewers[n].addPointCloud(trans_cloud, trans_color_handler, "aligned");
          viewers[n].setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "aligned");
          viewers[n].addText(final_cand[n].name + "aligned with RMSE "+ to_string(final_cand[n].rmse), 30,30, 0.2, 1, 0.3, "info");
        }
        //viewers[n].addCoordinateSystem(0.1,"axes");
        //no need for cluttered objects
        //if (vis_scene){
        //Eigen::Affine3f rotation;
        //float rot_angle = (float)(stoi(vst[1])*D2R);
        //rotation = Eigen::AngleAxisf(rot_angle, Eigen::Vector3f::UnitY());   
        //pcl::transformPointCloud (scene_cloud, scene_cloud, rotation); }
        viewers[n].addPointCloud(scene_cloud.makeShared(), "scene");
        viewers[n].setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene");
      }
      while(!viewers[0].wasStopped())
      {
        for (size_t i=0; i<viewers.size(); ++i)
          viewers[i].spinOnce();
      }
    }
  } //end test_f
  return (1);
}//END
