/* This file describes the PoseEstimation interface, for relative implementation
 * look in PoseEstimation_interface.hpp
 */

//Doxygen documentation

/** 
 *  \todo add view of candidates, estimation, query  etc..
 */

/** \mainpage notitle 
Pose Estimation interface Documentation and usage
-------------------------------------------------
This documentation describes the use of Pose Estimation interface, that implements a method to achieve pose estimation of a query object, represented by a point cloud.
The method makes use of several global and semi-global features and combines their results together in effort to achieve a sort of consensus. The best candidates selected
from consensus are then refined with ICP in order to get the final estimation. For more information on the matter please look into the author's master thesis available online 
<a href="http://etd.adm.unipi.it/theses/available/etd-09022014-142255/">here.</a>

Full source code of the project is available on <a href="https://bitbucket.org/Tabjones/poseestimation">bitbucket.</a>
*/
/** \page param Parameters usage and description
 * 
 * This page contains a list of configuration parameters that can be set to customize the behaviour of pose estimation.
 * All parameters are configured in a key = value fashion, to set them see PoseEstimation class methods.
 * For example to set a single parameter use setParam(), to set a large number of parameters either call setParam() several times, or write them in a configuration file and load them with initParams().
 * Configuration file must have .conf extension and contains only entry like key=value in each line. En example configuration file that sets default values for all parameters is located in the conf directory of source code.
 *
 * Basic Parameters
 * ------------------
 *
| key     | Default Value | Description                                                          |
|:-------:|:-------------:|:---------------------------------------------------------------------|
| useVFH  | 1             | Tell PoseEstimation to use Viewpoint Feature Histogram in matching phase (1), along with the others if present. Or disable it's computation (0). Note that at least one feature must be enabled|
| useESF  | 1             | Tell PoseEstimation to use Ensemble of Shape Functions in matching phase (1), along with the others if present. Or disable it's computation (0). Note that at least one feature must be enabled|
| useCVFH |            1  | Tell PoseEstimation to use Clustered Viewpoint Feature Histogram in matching phase (1), along with the others if present. Or disable it's computation (0). Note that at least one feature must be enabled|
| useOURCVFH |         1  | Tell PoseEstimation to use Oriented, Unique and Repeatable CVFH  in matching phase (1), along with the others if present. Or disable it's computation (0). Note that at least one feature must be enabled|
| downsampling |  1       | Tell PoseEstimation to downsample the query with Voxel Grid (1), or don't (0). Other parameters control the voxel grid intensity|
| upsampling  | 0         | Tell PoseEstimation to upsample the query with MLS random uniform density, before the eventual downsampling (1), or don't upsample it (0). Other parameters control MLS procedure|
| filtering   | 0         | Tell PoseEstimation to filter the query with Statistical Outliers Filter, before the eventual upsampling and downsampling (1), or don't filter it (0). Other parameters control the filter behaviour|
| progBisection | 1       | Tell PoseEstimation to use Progressive Bisection (1), or Brute Force (0), during candidates refinement| 
| kNeighbors | 20         | How many neighbors to retrieve during matching phase, this value determines the length of each candidate list|
| rmseThreshold | 0.003   | RMSE Threshold during Candidate Refinement, when a candidate rmse falls below this threshold, the refinement procedure terminates. If progBisection=1 this can be set to zero to always choose the surviving candidate at the end of progressive bisection |
| verbosity | 1           | Controls the verbosity of PoseEstimation: (0) Silent behaviour, only errors are printed, (1) Normal behaviour, errors and warnings are printed, (2) Verbose behaviour, all kind of messages are printed (can be very spammy, useful in debug or testing) |
| maxIterations | 200     | Maximum iterations of ICP (termination condition) for each candidate during Brute Force candidate refinement. This parameter is relevant only if progBisection=0 |

Advanced Parameters 
-----------------------------------------------------------------------------
These are advanced parameters, that can significantly alter the behaviour of pose estimation, change them if you know what you are doing.
For example when changing preprocessing pipeline (i.e. altering search radius of MLS, or voxel grid leaf size) make sure that the query and the poses in database have undergone the same exact process, or you may not find consistent matches.
| key | Default Value | Description |
|:----:|:-------------:|:------------------------|
| mlsPolyOrder | 2    | Set polynomial order of MLS to the value specified, for most objects  second order polynomial functions are more than enough to correctly approximate the object surface. Relevant only if upsampling=1 |
| mlsPointDensity | 250 | Set desired point density on MLS surface to the value specified,  higher values produce more points, thus increasing the upsampling. Relevant only if upsampling=1 |
| mlsPolyFit   | 1 | Tell MLS to fit the surface on polynomial functions with order mlsPolyOrder (1). Or don't use polynomial fitting (0). Relevant only if upsampling=1 |
| mlsSearchRadius  | 0.03 | Set search radius of MLS to the value specified, a value of 1 means one meter. Relevant if upsampling=1 |
| filterMeanK | 50 | How many neighboring points to consider in the statistical distribution calculated by the filter, relevant if filtering=1 |
| filterStdDevMulThresh | 3 | Multiplication factor to apply to Standard Deviation of the statistical distribution during filtering process (higher value, means less aggressive filter). Relevant only if filtering=1 |
| progItera   | 5 | ICP iterations to perform for all candidates on each step of Progressive Bisection Candidate Refinement, lowering this value may speed up the process at the cost of a less accurate estimation. Relevant only if progBisection=1 |
| progFraction | 0.5 | Size of candidate list gets multiplied by this value on each step of Progressive Bisection Refinement, the default is to split the list in half (0.5). Relevant only if progBisection=1 |
| vgridLeafSize | 0.003 | Set the leaf size of VoxelGrid downsampling to the value specified, 1 means one meter. Only relevant if downsampling=1 |
| neRadiusSearch | 0.015 | Set radius that defines the neighborhood of each point during Normal Estimation, value of 1 means one meter. If normals are not used, i.e. only ESF is computed this parameter is ignored |
| useSOasViewpoint | 0 | Tell PoseEstimation to use the sensor_origin_ member of query point cloud as the viewpoint for every operation (1), where a viewpoint is need, i.e in normals estimation or in VFH descriptor. If set to '0' a viewpoint should be set manually with method setQueryViewpoint() or calculated from name (see computeViewpointFromName). If the above fails features will refuse to compute. If set to '1' and sensor_origin_ is not representing a correct viewpoint, unexpected results may happen, such as normals not correctly oriented, probably resulting in a wrong pose estimation |
| computeViewpointFromName | 1 | If set to '1' PoseEstimation will try to calculate the viewpoint from the query name, however to use this feature, query names should follow a naming convenction, that is name_latitude_longitude (i.e Funnel_30_60). Note also that setting this parameter to 1 will override 'useSOasViewpoint' parameter, regardless of its value. If set to 0 and also 'useSOasViewpoint' is set to 0 a view point must be supplied with setQueryViewpoint(), or feature will refuse to compute |
| cvfhEPSAngThresh |7.5 deg | Set maximum allowable deviation of the normals, in the region segmentation step of CVFH computation. The value recommended from relative paper is 7.5 degrees, it should be supplied in degrees and class will convert  it in radians. Relevant only if useCVFH=1 |
| cvfhCurvThresh  | 0.025 | Set maximum allowable disparity of curvatures during region segmentation step of CVFH estimation. The value recommended from relative paper is 0.025. Relevant only if useCVFH=1 |
| cvfhClustTol  | 0.01 | Euclidean clustering tolerance, during CVFH segmentation. Points distant more than this value from each other, will likely be grouped in different clusters. A value of 1 means one meter. Relevant only if useCVFH=1 |
| cvfhMinPoints  | 50 | Set minimum number of points a cluster should contain to be considered such, during CVFH clustering. Relevant only if useCVFH=1 |
| ourcvfhEPSAngThresh  | 7.5 deg | Set maximum allowable deviation of the normals, in the region segmentation step of OURCVFH computation. The value recommended from relative paper is 7.5 degrees. It should be supplied in degrees and class will convert it in radians. Relevant only if useOURCVFH=1 |
| ourcvfhCurvThresh | 0.025 | Set maximum allowable disparity of curvatures during region segmentation step of OURCVFH estimation. The value recommended from relative paper is 0.025. Relevant only if useOURCVFH=1|
| ourcvfhClustTol | 0.01 | Euclidean clustering tolerance, during OURCVFH segmentation. Points distant more than this value from each other, will likely be grouped in different clusters. A value of 1 means one meter. Relevant only if useOURCVFH=1 |
| ourcvfhMinPoints  |50 | Set minimum number of points a cluster should contain to be considered such, during OURCVFH clustering. Relevant only if useOURCVFH=1 |
| ourcvfhAxisRatio  | 0.95 | Set the minimum axis ratio between the SGURF axes. At the disambiguation phase of OURCVFH, this will decide if additional Reference Frames need to be created for the cluster, if they are ambiguous. Relevant only if useOURCVFH=1 |
| ourcvfhMinAxisValue  | 0.01 | Set the minimum disambiguation axis value to generate several SGURFs for the cluster when disambiguition is difficult. Relevant if useOURCVFH=1 |
| ourcvfhRefineClusters  |  1 | Set refinement factor for clusters during OURCVFH clustering phase, a value of 1 means 'dont refine clusters', while values between 0 and 1 will reduce clusters size by that number. Relevant only if useOURCVFH=1 |
*/

#ifndef __INTERFACE_H_INCLUDED__
#define __INTERFACE_H_INCLUDED__

#include <iostream>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <stdexcept>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/norms.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
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
#include <boost/range/algorithm/copy.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>


using namespace pcl;
using namespace std;
using namespace pcl::console;
using namespace boost;
using namespace boost::filesystem;
using namespace flann;

/** \addtogroup Definitions
 * 
 * Easier code writing
 * @{ */

/**Degrees to radians conversion*/
#define D2R 0.017453293 

/**Current local time, seconds precision*/
#define TIME_NOW boost::posix_time::second_clock::local_time()

/** Chosen metric for VFH and Index type*/
typedef flann::Index<flann::ChiSquareDistance<float> > indexVFH;
/** Chosen metric for ESF and Index type*/
typedef flann::Index<flann::L2<float> > indexESF;
/** FLANN matrix used to store database histograms*/
typedef flann::Matrix<float> histograms;
/** Chosen Point Type to store clouds, change this  if you want to change PointType globally*/
typedef PointXYZRGBA PT;
/** Short writing for PointCloud containing the chosen Point Type*/
typedef PointCloud<PT> PC;
/** Map that stores configuration parameters in a key=value fashion*/
typedef std::unordered_map<std::string,float> parameters;
/** Typedef for shorter writing of timestamps */
typedef boost::posix_time::ptime timestamp;
/** Enumerator for list of candidates, see getCandidateList() for an example usage*/
enum class listType {vfh, esf, cvfh, ourcvfh, composite};
/** @} */

class PoseEstimation;

/**\brief Stores the database of poses for Pose Estimation 
 *
 * This class is used internally by PoseEstimation, however it can be used to create or 
 * load multiple databases and test pose estimation with them. I.e. with setDatabase() method of PoseEstimation.
 * Some examples:
 * \code
 * PoseDB database; //empty database
 * database.load("LOCATION"); //load a database from a location
 * PoseDB database2("LOCATION2"); //create another database and load it from another location
 * database = database2; //Now database holds a copy of database2
 * database2.clear(); //database2 is now empty
 * database2.create("PCD_FILES_LOCATION"); //create a new database from scratch using the poses found at supplied location
 * //...
 * PoseEstimation prova; //create a pose estimation with default parameters
 * prova.setDatabase(database); //tell prova to use the first database
 * //...  estimate
 * prova.setDatabase(database2); //now use the second database
 * //... estimate again with another database
 * \endcode
 * \author Federico Spinelli
 */
class PoseDB{
  
  friend class PoseEstimation;
  boost::shared_ptr<histograms> vfh_, esf_, cvfh_, ourcvfh_;
  vector<string> names_;
  vector<string> names_cvfh_, names_ourcvfh_;
  boost::filesystem::path dbPath_; 
  vector<PC> clouds_;
  boost::shared_ptr<indexVFH> vfh_idx_;
  boost::shared_ptr<indexESF> esf_idx_;

  /**\brief Calculates unnormalized distance of objects, based on their cluster distances, internal use.
   * \param[in] query Pointer to the query histogram(s)
   * \param[in] feat Enum that indicates which list: listType::cvfh or listType::ourcvfh
   * \param[out] distIdx Vector of unnormalized distances of objects and they relative index
   * \return _True_ if distances are correctly computed, _false_ otherwise
   */
  bool computeDistFromClusters_(PointCloud<VFHSignature308>::Ptr query, listType feat, vector<pair<float, int> >& distIdx);

  public:
    /** \brief Default empty Constructor
     */
    PoseDB(){}
    
    /** \brief Constructor which loads a database from disk
     * \param[in] pathDB Path to the directory containing the database of poses
     */
    PoseDB(boost::filesystem::path pathDB){this->load(pathDB);}
    
    /** \brief Copy constructor from another PoseDB
     * \param[in] db Object to copy from
     */
    PoseDB(const PoseDB& db);

    /** \brief Load a database from disk, knowing its location
     * \param[in] pathDB Path to the directory containing the database of poses
     */
    bool load(boost::filesystem::path pathDB);

    /** \brief Save a database to disk
     * \param[in] pathDB Path to a directory on disk, inside which to save the database, directory must not exist.
     * \return _True_ if successful, _false_ otherwise
     *
     * pathDB must be a valid path and must not already exists, overwriting is not supported
     */
    bool save(boost::filesystem::path pathDB);
    
    /** \brief Compute the whole database from scratch and store it in memory.
     * \param[in] pathClouds Path to a directory on disk that contains all the pcd files of object poses
     * \param[in] params Shared pointer to parameters to use during database creation
     * \return _True_ if successful, _false_ otherwise
     * 
     * This method uses the provided set of parameters to create the database, erasing any previously loaded databases.
     * Please note that:
- Constructing a database from scratch can take several minutes at least.
- In order to use this method, PCD files must follow a naming convention, that is objName_latitude_longitude.pcd  (i.e. funnel_20_30.pcd). Not using this naming convention may result in corrupted or unusable database.
- PCD files must represent previously segmented objects and must be expressed in a local reference system, i.e. a system centered at the object base. This system must be consistent with all the PCDs provided.
- PCDs should have stored the viewpoint location (coordinates of where the sensor was positioned during acquisition) inside their sensor_origin_ member for optimal results, although this is not mandatory
     */
    bool create(boost::filesystem::path pathClouds, boost::shared_ptr<parameters> params);
    
    /** \brief Compute the whole database from scratch and store it in memory.
     * \param[in] pathClouds Path to a directory on disk that contains all the pcd files of object poses
     * \return _True_ if successful, _false_ otherwise
     *  
     * This method creates a set of default parameters and creates the database from it, erasing any previously loaded databases. 
     * Please note that:
- Constructing a database from scratch can take several minutes at least.
- In order to use this method, PCD files must follow a naming convention, that is objName_latitude_longitude.pcd  (i.e. funnel_20_30.pcd). Not using this naming convention may result in corrupted or unusable database.
- PCD files must represent previously segmented objects and must be expressed in a local reference system, i.e. a system centered at the object base. This system must be consistent with all the PCDs provided.
- PCDs should have stored the viewpoint location (coordinates of where the sensor was positioned during acquisition) inside their sensor_origin_ member for optimal results, although this is not mandatory
     */
    bool create(boost::filesystem::path pathClouds);
      
    /** \brief Copy assignment operator
     * \param[in] db OBject to copy
     *
     * Example usage:
     * \code
     * PoseDB a; //a is empty
     * PoseDB b(path_to_database); //b is loaded from path
     * a = b; // now a holds a copy of b
     * \endcode
     */
    PoseDB& operator= (const PoseDB& db);

    /** \brief Erase the database from memory, leaving it unset
    */
    void clear();

    /** \brief Tell if the database is empty
     * 
     * \return _True_ if database is not loaded or empty, _False_ otherwise
     */
    bool isEmpty();

    /** \brief Check if a path contains a valid database
     * \param[in] dbPath Path to directory containing database to check
     * \return _True_ if valid, _False_ otherwise
     *
     * Checks if the directory has a valid database structure, clouds of poses, FLANN matrices of histograms, indexes and name files
     */
    bool isValidPath(boost::filesystem::path dbPath);
};

/** \brief Describes a single candidate object to the query 
 *
 * This class is used internally by PoseEstimation, a few methods are present to look at rank, distance, RMSE and transformation, if needed.
 * However it is not meant to be used directly
 * \author Federico Spinelli
 **/
class Candidate{
  
  friend class PoseEstimation;
  string name_;
  PC::Ptr cloud_;
  int rank_;
  float distance_;
  float normalized_distance_;
  float rmse_;
  Eigen::Matrix4f transformation_;

  public:
    /** \brief Default empty Constructor
     */
    Candidate ();
    
    /** \brief Constructor with name and cloud
     * \param[in] str The object name the candidate will have
     * \param[in] cl Point cloud which holds the object
     */
    Candidate (string str, PC& cl);
    
    /** \brief Constructor with name and cloud pointer
     * \param[in] str The object name the candidate will have
     * \param[in] clp Shared pointer to the point cloud that contains the object
     * Note that clp parameter should not be empty pointer, or the contructor will throw an error
     */
    Candidate (string str, PC::Ptr clp);
    
    /** \brief Copy Constructor from another Candidate
     * \param[in] c Candidate to copy from
     */
    Candidate (const Candidate& c);

    /** \brief Copy assignment from another Candidate
     * \param[in] c Candidate to copy from
     */
    Candidate& operator= (const Candidate& c);
    
    /** \brief Get Candidate Rank from the list of candidates it belongs
     * \return The rank of Candidate in the list
     *
     * A Candidate has a rank only after list(s) of Candidates are built by PoseEstimation
     */
    int getRank () const;
    
    /** \brief Get the distance of Candidate from Query in the metric chosen by the feature
     * \return The distance the candidate has from the query
     *
     * A Candidate has a distance only after list(s) of Candidates are built by PoseEstimation
     */
    float getDistance () const;
    
    /** \brief Get the normalized distance of Candidate from Query
     * \return The normalized distance the candidate has from the query
     * 
     * Normalized distances range from 0 to 1, zero at "rank 1" and one at "rank k", they are indipendent of the
     * current metric chosen to calculate them, thus could be compared with other normalized distances from other 
     * lists. A Candidate has a normalized distance only after list(s) of Candidates are built by PoseEstimation
     */
    float getNormalizedDistance () const;
    
    /** \brief Get Root Mean Square Error of Candidate as it was after the refinement
     * \return The Root Mean Square Error of the Candidate
     *
     * A Candidate has an RMSE only after the refinement process has been performed by PoseEstimation
     */
    float getRMSE () const;
    
    /** \brief Get Homogeneous Transformation that brings the Candidate over the Query
     * \param[out] t The transformation that brings the Candidate over the Query
     * 
     * The transformation is expressed in the Candidate Reference System and it only has a meaning after the refinement
     * process has been performed by PoseEstimation
     */
    void getTransformation (Eigen::Matrix4f& t) const;

    /** \brief Get the point cloud that represent the Candidate
     * \param[out] cloud Copy of the point cloud of the candidate
     */
    void getCloud (PC& cloud) const;

    /** \brief Get Candidate name
     * \param[out] name The name of the Candidate
     */
    void getName(string& name) const;
};

/**\brief Implements the procedure to achieve pose estimation of a given query object.

  The ideal procedure can be summarized as follows:
- Initialize the class parameters either with the constructors or with initParams()
\code
  PoseEstimation pe; //empty constructor, default parameters are set
  PoseEstimation pe2("config/params.config"); //constructor with path to a configuration file
  pe.initParams("config/params.config"); //change all parameters specified in the configuration file
  pe.setParam("verbosity", 2); //change parameter "verbosity" to 2
\endcode
- Set the query object to be identified with method setQuery() and the database to use with setDatabase()
\code
  PointCloud<PointXYZRGBA> cl;
  loadPCDFile("cloud.pcd", cl);
  pe.setQuery("funnel_20_30", cl); //set a new query to be a point cloud contained in "cl" and name it "funnel_20_30"
  pe.setDatabase("../Database"); //load a database from disk contained in directory "../Database"
\endcode
- Generate the list(s) of candidates to the query with method generateLists()
\code
  pe.generateLists(); //list of candidates are computed, based on type of parameters set
\endcode
- Obtain the final candidate with method refineCandidates()
\code
  pe.refineCandidates(); //Select one final candidate from the lists, representing the pose estimation
\endcode
- Print, Get or Save the final pose estimation with corresponding methods
\code
  pe.saveEstimation("file.pose"); //Save the estimation to a file on disk
  pe.printEstimation(); //print the estimation on screen
  pe.viewEstimation(); //open a pcl visualizer to view the estimation
\endcode

Alternatively the method estimate() can be called right after initialization with constructor
\code
  PoseEstimation pe;
  PointCloud<PointXYZRGBA> cloud;
  //... 
  //Put somethging in cloud
  //...
  pe.estimate("query object", cloud, "DB_PATH"); //calls setQuery("query object", cloud), setDatabase("DB_PATH"), generateList(), refineCandidates()
\endcode

\author Federico Spinelli
 */
class PoseEstimation {
  ///Map to store all parameters as key=value pairs
  parameters params_;
  
  ///Database used for Pose Estimation
  PoseDB database_;

  ///Pointer to the final Candidate that holds the pose estimation of the query
  boost::shared_ptr<Candidate> pose_estimation_;

  ///Internal counter used to count how many feature the class uses
  int feature_count_;
  
  ///The name of the query to estimate
  string query_name_;
  
  ///The cloud pointer that represent the query to estimate as supplied before any computation
  PC::Ptr query_cloud_;
  
  ///Cloud pointer to query pre-processed cloud (if preprocessing is made, otherwise it's the same as query_cloud
  PC::Ptr query_cloud_processed_;
  
  ///List of candidates to the query calculated from VFH
  vector<Candidate> vfh_list_;
  ///List of candidates to the query calculated from ESF
  vector<Candidate> esf_list_;
  ///List of candidates to the query calculated from CVFH
  vector<Candidate> cvfh_list_;
  ///List of candidates to the query calculated from OURCVFH
  vector<Candidate> ourcvfh_list_;
  ///List of candidates to the query composed from the other features
  vector<Candidate> composite_list_;
  
  ///Viewpoint coordinate x, used in computations like VFH and Normal estimation
  float vpx_;
  ///Viewpoint coordinate y, used in computations like VFH and Normal estimation
  float vpy_;
  ///Viewpoint coordinate z, used in computations like VFH and Normal estimation
  float vpz_;
  
  ///Internal parameter to check if viewpoint has been supplied from setQueryViewpoint.
  bool vp_supplied_;
  ///Internal to see if viewpoint was calculated
  bool vp_set_;
  ///Internal parameter to check if the query was succesfully set and its features estimated
  bool query_set_;
  ///Internal parameter to check if list(s) of candidates are successfully generated
  bool candidates_found_;
  ///Internal parameter to check if candidate refinement has been done and it was succesfull
  bool refinement_done_;
  ///Internal parameter to check if a database was loaded in memory and it's now ready to be used
  bool db_set_;
  
  ///Container that hold the query VFH feature
  PointCloud<VFHSignature308> vfh_;
  ///Container that hold the query CVFH feature
  PointCloud<VFHSignature308> cvfh_;
  ///Container that hold the query OURCVFH feature
  PointCloud<VFHSignature308> ourcvfh_;
  ///Container that hold the query ESF feature
  PointCloud<ESFSignature640> esf_;
  ///Container that hold the query normals
  PointCloud<Normal> normals_;
  
  ///Set a parameter of Pose Estimation from a string representing its value, used internally when reading parameters from a file
  bool setParam_ (string, string&);
  
  ///Initialize the Query by computing preprocessing and features, returns true if success, internal use
  bool initQuery_();
  
  ///Internal method to filter the query with Statistical Outlier Removal, internal use
  void filtering_();
  
  ///Internal method to upsample the query with MLS Random Uniform Density, internal use
  void upsampling_();
  
  ///Internal method to downsample the query with VoxelGrid, internal use
  void downsampling_();
  
  ///Internal method to compute Surface Normals of the query, internal use, return true if success
  bool computeNormals_();
  
  ///Internal method to compute VFH feature of the query, internal use
  void computeVFH_();
  
  ///Internal method to compute ESF feature of the query, internal use
  void computeESF_();
  
  ///Internal method to compute CVFH feature of the query, internal use
  void computeCVFH_();
  
  ///Internal method to compute OURCVFH feature of the query, internal use
  void computeOURCVFH_();

  /**\brief Searches a list for a candidate and eliminates it, saving its distance. Internal use
   * \param[in] list The list to inspect and modifiy
   * \param[in] name The name to search in list
   * \param[out] dist The distace of Candidate found in list (normalized)
   * 
   * Return true if the candidate is found on the list, false otherwise
   */
  bool findCandidate_(vector<Candidate>& list, string name, float& dist);

  public:
  ///Default Empty Constructor that sets default parameters (see them in config file "config/parameters.conf")
  PoseEstimation(); 
  
  /**\brief Set a parameter of the Class directly, knowing its name
  * \param[in] key the parameter name to change
  * \param[in] value the value that key should assume
  * \return _True_ if successful, _false_ otherwise
  * 
  * For more information on possible parameters look at \ref param.
  * Example Usage:
  * \code
  * PoseEstimation pe; //construct and sets default parameters
  * pe.setParam("useVFH", 0); //pe now skips VFH computation for query
  * string str;
  * str = "verbosity";
  * pe.setParam(str, 2); //pe now has verbosity set to 2
  * \endcode
  * Note that parameters should not be updated until current estimation is completed (i.e. between call of setQuery() and call of refineCandidates())
  */
  bool setParam (string key, float value);
  
  /**\brief Set a parameter of the Class directly, knowing its name
  * \param[in] key the parameter name to change
  * \param[in] value the value that key should assume
  * \return _True_ if successful, _false_ otherwise
  *
  * Overloaded for ints.
  * For more information on possible parameters look at \ref param.
  * Note that parameters should not be updated until current estimation is completed (i.e. between call of setQuery() and call of refineCandidates())
  */
  bool setParam (string key, int value) {return (this->setParam(key, (float)value) );}
  
  /**\brief Set a parameter of the Class directly, knowing its name
  * \param[in] key the parameter name to change
  * \param[in] value the value that key should assume
  * \return _True_ if successful, _false_ otherwise
  *
  * Overloaded for double
  * For more information on possible parameters look at \ref param.
  * Note that parameters should not be updated until current estimation is completed (i.e. between call of setQuery() and call of refineCandidates())
  */
  bool setParam (string key, double value) {return (this->setParam(key, (float)value) );}
  
  /** \brief Initialize the class with parameters found in config file
  * \param[in] config_file Path to a config file to use
  * \return How many parameters were correctly found and set, or (-1) in case of errors
  *
  * Configuration file must have extension .conf and follow certain naming conventions, 
  * look at example .conf file provided for more details or at \ref param.
  * Note that parameters should not be updated until current estimation is completed (i.e. between call of setQuery() and call of refineCandidates())
  */
  int initParams (boost::filesystem::path config_file);

  /** \brief Initialize parameters of PoseEstimation from the map provided
   * \param[in] map shared pointer to unordered_map containing parameters to use
   * \return How many parameters were correctly set, or (-1) in case of errors
   *
   * unordered_map must contain valid keys and values, otherwise they will be ignored. For more inforamtion look at \ref param.
  * Note that parameters should not be updated until current estimation is completed (i.e. between call of setQuery() and call of refineCandidates())
   */
  int initParams (boost::shared_ptr<parameters> map);
  
  /**\brief Constructor with parameters to set.
   *\param[in] map Shared pointer to unordered_map containing parameters to use
   *
   * For more information on parameters look at \ref param.
   * Note: This constructor uses C++11 functionality and will  not compile without -std=c++11 
   * It delegates construction to empty contructor then calls initParams()
   * It is the same way as calling empty constructor and then initParams() method.
   */
  PoseEstimation(boost::shared_ptr<parameters> map) : PoseEstimation() {this->initParams(map);}
  
  /**\brief Constructor with path to a configuration file containing parameters to set.
   *\param[in] config_file Path to a config file to use
   *
   * Configuration file must have extension .conf and follow certain naming conventions, 
   * look at example .conf file provided for more details, or look at \ref param.
   * Note: This constructor uses C++11 functionality and will probably not compile without -std=c++11 
   * It delegates construction to empty contructor then calls initParams()
   * It is the same way as calling empty constructor and then initParams() method
   */
  PoseEstimation(boost::filesystem::path config_file) : PoseEstimation() {this->initParams(config_file);}

  /**\brief Explicitly set the query viewpoint for future query computations
   *\param[in] x Coordinate x of the viewpoint
   *\param[in] y Coordinate y of the viewpoint
   *\param[in] z Coordinate z ot the viewpoint
   *
   * __This method overrides any viewpoint parameters set__ , thus Pose Estimation
   * will ignore "useSOasViewpoint" and "computeViewpointFromName" parameters regardless of their value, 
   * and will use only the viewpoint set this way for the computations where a viewpoint is needed 
   * (normals, VFH, CVFH ...) until a new viewpoint is supplied again with this method.
   * If you want to reset the viewpoint supplied this way and go back using the above parameters, use resetViewpoint() method.
   * This method should be used only if the the viewpoint cannot be obtained from sensor_origin of query cloud
   * or if it cannot be computed from query name (i.e. the above cited parameters failed to set it correctly)
   * The recommended and preferred way is to use the sensor_origin field, so prepare your query cloud accordingly
   * and use the "useSOasViewpoint" parameter instead of this method
   */
  void setQueryViewpoint(float x, float y, float z);
  
  /**\brief Set the Pose Estimation query (the object to be identified)
  * \param[in] str The name the query should assume
  * \param[in] cl  Point cloud containing only the object to be estimated (i.e. already segmented)
  */
  void setQuery (string str, PC& cl);
  
  /** \brief Set the Pose Estimation query (the object to be identified)
  * \param[in] str the name the query should assume
  * \param[in] clp Shared pointer containing only the pointcloud of the object to be estimated (i.e. already segmented)
  */
  void setQuery (string str, PC::Ptr clp);

  /// \brief Print current parameter values on screen
  void printParams();

  /** \brief Save current configuration parameters into a .conf file
   * \param[in] file Path on disk where to write the .conf file
   * \return _True_ if operation was successful, _false_ otherwise
   *
   * The path specified must not point to an already existant file, or saveParams() will refuse to write. 
   * The path may be absolute or relative and may or may not contain the extension, however remember that config files must have .conf extension to be accepted from PoseEstimation as valid config files, see also initParams()
   * Example code:
   * \code
   * PoseEstimation pe;
   * pe.setParam("verbosity", 2);
   * pe.saveParams("./verbose.conf"); //relative path with extension specified
   * pe.saveParams("../verbose"); //relative path without extension, in this case a .conf will be appended
   * pe.saveParams("verbose.wrong"); //the file will be written with warnings but initParams() will not read it back since the file has a wrong extension
   * \endcode
*/
  bool saveParams(boost::filesystem::path file);

  /// \brief Print list(s) of Candidate to the query on screen
  void printCandidates();

  /** \brief Save list(s) of Candidate to query on a file
   * \param[in] file Path to a file where to write the candidate list(s)
   * \return _True_ if operation was successful, _false_ otherwise
   *
   * Path specified may exist, in that case the file will be appended
   */
  bool saveCandidates(boost::filesystem::path file);

  /** \brief Set a database of known poses to be used for pose estimation procedure.
   * \param[in] dbPath The path to the directory containing the database
   *
   * Note that this method calls PoseDB::load() on the path specified
   */
  void setDatabase(boost::filesystem::path dbPath);
  /** \brief Set a database of known poses to be used for pose estimation procedure.
   * \param[in] database PoseDB object to use as database
   */
  void setDatabase(PoseDB& database);

  /** \brief Generates list(s) of candidates to the query using the database provided as argument
   * \param[in] dbPath The path to the directory containing the database of poses, previously generated.
   * \return _True_ if successful, _false_ otherwise
   *
   * Note that this method also calls setDatabase() on the path specified for future lists computations.
   */
  bool generateLists(boost::filesystem::path dbPath);
  
  /** \brief Generates list(s) of candidates to the query using the database provided as argument
   * \param[in] db The PoseDB object containing the database of poses, previously generated or loaded.
   * \return _True_ if successful, _false_ otherwise
   *
   * Note that this method also calls setDatabase() for future lists computations.
   */
  bool generateLists(PoseDB& db);
  
  /** \brief Generates list(s) of candidates to the query using previously set database
   * \return _True_ if successful, _false_ otherwise
   */
  bool generateLists();

  /**\brief Start the refinement procedure with ICP to obtain a final candidate from the composite list
   * \return _True_ if refinement is successful and one final candidates is obtained, _false_ otherwise
   *
   * Currently two methods for refinemente are implemented: Progressive Bisection (default) and Brute Force
   * To chose Progressive Bisection set "progBisection" parameter to 1, to chose Brute Force set it to 0
- Brute Force:
  1. Start align rank 1 candidate on composite list with ICP, until "maxIterations" (default 200) are reached or candidate RMSE falls below "rmseThreshold" (default 0.003)
  2. Align all the candidates until one converges, that one is the final pose estimation
  3. If no one converges the Pose Estimation will produce no answer. Set an higher "rmseThreshold" parameter
- Progressive Bisection:
  1. Align all candidates on composite list with ICP, but performs at most "progItera" (default 5) iterations
  2. Resort the list based on minimum RMSE of candidates
  3. Discard a fraction of the list multiplying its size by "progFraction" (default 0.5), eliminating candidates with worst rmse
  4. Repeat from step 1 until one candidates falls below the "rmseThreshold" (default 0.003) or only one candidate survives
  */
  bool refineCandidates();

  /** \brief Undergo the whole process of pose estimation
   *\param[in] name Name of the new query object to estimate
    \param[in] cloud Point cloud containing the query object to estimate (segmented)
    \param[in] db_path Path to a directory containing the database of poses to load
    \return _True_ if pose estimation was successful in all its parts, _false_ otherwise

    This method calls is sequence setQuery() setDatabase() generateLists() and refineCandidates(), using the
    already set parameters 
    */
  bool estimate(string name, PC& cloud, boost::filesystem::path db_path);
  
  /** \brief Undergo the whole process of pose estimation
   *\param[in] name Name of the new query object to estimate
    \param[in] cloud_pointer Pointer to a point cloud containing the query object to estimate (segmented)
    \param[in] db_path Path to a directory containing the database of poses to load
    \return _True_ if pose estimation was successful in all its parts, _false_ otherwise

    This method calls is sequence setQuery() setDatabase() generateLists() and refineCandidates(), using the
    already set parameters 
    */
  bool estimate(string name, PC::Ptr cloud_pointer, boost::filesystem::path db_path);
  
  /** \brief Undergo the whole process of pose estimation
   *\param[in] name Name of the new query object to estimate
    \param[in] cloud Point cloud containing the query object to estimate (segmented)
    \param[in] database PoseDB object to use as database 
    \return _True_ if pose estimation was successful in all its parts, _false_ otherwise

    This method calls is sequence setQuery() setDatabase() generateLists() and refineCandidates(), using the
    already set parameters 
    */
  bool estimate(string name, PC& cloud, PoseDB& database);
  
  /** \brief Undergo the whole process of pose estimation
   *\param[in] name Name of the new query object to estimate 
    \param[in] cloud_pointer Pointer to a point cloud containing the query object to estimate (segmented)
    \param[in] database PoseDB object to use as database 
    \return _True_ if pose estimation was successful in all its parts, _false_ otherwise

    This method calls is sequence setQuery() setDatabase() generateLists() and refineCandidates(), using the
    already set parameters 
    */
  bool estimate(string name, PC::Ptr cloud_pointer, PoseDB& database);

  /** \brief Print final estimation informations (such as name, distance, rmse and transformation) on screen
   */
  void printEstimation();

  /**\brief Save final estimation informations (such as name, distance, rmse and transformation) on a file with path specified
      \param[in] file Path to a location on disk, where the file will be created or renewed
      \param[in] append Chose to append to the end of file (true)(default), or truncate its contents (false), if file does not exists this parameter has no effectinal
      \return _True_ if file is correctly written, _False_ otherwise

      The path can specify a directory or a file, if the former a new text file of name "query_name.estimation" will be created inside the specified directory.
      If the latter, the specified file will be written.
      Note on paths: if file path does not exists, a file must be specified with an extension, otherwise it will be treated as a directory.
      For example "/foo/bar" or "/foo/bar/" will be both treated as directories if they don't exists already.
      "foo/bar.f" will be treated as file "bar.f" inside relative directory "foo"
      */
  bool saveEstimation(boost::filesystem::path file, bool append = true);
  /** \brief Get a shared pointer of a copy of parameters used by the pose estimation
   *  \return Shared pointer of a copy of parameters used
   *
   * Returned pointer can be modified but any changes are not reflected back to the class,
   * use setParam() or initParams() to modify PoseEstimation parameters
  */ 
  boost::shared_ptr<parameters> getParams();
  /** \brief Get a shared pointer of a copy of parameters used by the pose estimation
   *  \param[out] params Shared pointer of a copy of parameters used
   *  \return _True_ if params correctly contains the class parameters, _false_ otherwise
   *
   * Passed pointer can be modified but any changes are not reflected back to the class,
   * use setParam() or initParams() to modify PoseEstimation parameters
  */ 
  bool getParams(boost::shared_ptr<parameters> params);

  /** \brief Get a pointer to a copy of final chosen Candidate, representing the pose estimation
   * \return A pointer to a copy of the final Candidate
   */
  boost::shared_ptr<Candidate> getEstimation();
  
  /** \brief Get a pointer to a copy of final chosen Candidate, representing the pose estimation
   * \param[out] est A pointer to a copy of the final Candidate chosen by refinement
   * \return _True_ if est correctly points to final pose estimation, _false_ otherwise
   */
  bool getEstimation(boost::shared_ptr<Candidate> est);

  /** \brief Reset the viewpoint for current query, so that it can be computed again
   */
  void resetViewpoint(){ vp_supplied_ = false; }

  /** \brief Get a vector containing a list of Candidates to the Query
   * \param[out] list Vector containing a copy of Candidates list, ordered by rank
   * \param[in] type listType enumerate to select which list is to be copied among those created
   * \return _True_ if the selected list is correctly copied, _false_ otherwise
   *
   * Composite list is created by generateLists() and it is used by the refinement procedure, get that by passing type= listType::composite
   * vfh, esf, cvfh and ourcvfh lists are created by the corresponding features and may or may not exists, depending on parameters set.
   * To select one of them either pass type= listType::vfh, listType::esf, listType::cvfh or listType::ourcvfh
   * Code example:
   * \code
   * PoseEstimation pe;
   * // ... initialize query and generate list of candidates
   * vector<Candidate> mylist;
   * pe.getCandidateList(mylist, listType::composite ); //get the composite list, that always exists
   * pe.getCandidateList(mylist, listType::vfh ); //if vfh list exists, now mylist holds it
   * \endcode
   */
  bool getCandidateList(vector<Candidate>& list, listType type);

  /** \brief Print some information on Query object on screen
   */
  void printQuery();

  /** \brief Get name and  pointers holding point clouds of query object (before and after eventual preprocessing)
   * \param[out] name String containing query name
   * \param[out] clp Shared Pointer to a copy of query point cloud before preprocessing
   * \param[out] clp_pre Shared Pointer to a copy of query point cloud after preprocessing
   * \return _True_ if operation was successful, _false_ otherwise 
   *
   * If no preprocessing was done (i.e. downsampling, upsampling and filtering parameters are all zero), clp and clp_pre point the same cloud
   */
  bool getQuery (string& name, PC::Ptr clp, PC::Ptr clp_pre);

  /** \brief Get the estimated features (including normals) from the query
   * \param[out] vfh Point cloud pointer that will hold the VFH feature
   * \param[out] cvfh Point cloud pointer that will hold the CVFH feature
   * \param[out] ourcvfh Point cloud pointer that will hold the OURCVFH feature
   * \param[out] esf Point cloud pointer that will hold the ESF feature
   * \param[out] normals Point cloud pointer that will hold the feature normals
   * \return How many features were currently copied, or (-1) in case of errors
   *
   * Note that some features may not be available for the current query, because they were not estimated, check parameter settings
   */
  int getQueryFeatures(PointCloud<VFHSignature308>::Ptr vfh, PointCloud<VFHSignature308>::Ptr cvfh, PointCloud<VFHSignature308>::Ptr ourcvfh, PointCloud<ESFSignature640>::Ptr esf, PointCloud<Normal>::Ptr normals);

};
#endif
