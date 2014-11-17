/* This file describes the PoseEstimation interface, for relative implementation
 * look in PoseEstimation_interface.hpp
 */
//Doxygen documentation
/** \mainpage 
Pose Estimation interface Documentation and usage
-------------------------------------------------
This documentation describes the use of Pose Estimation interface, that implements a method to achieve pose estimation of a query object, represented by a point cloud.
The method makes use of several global and semi-global features and combines their results together in effort to achieve a sort of consensus. The best candidates selected
from consensus are then refined with ICP in order to get the final estimation. For more information on the matter please look into the author's master thesis available online 
<a href="http://etd.adm.unipi.it/theses/available/etd-09022014-142255/">here.</a>

Full source code of the project is available on <a href="https://bitbucket.org/Tabjones/poseestimation">bitbucket.</a>
*/
/** \page Parameters
 * 
 * This page contains a list of configuration parameters that can be set to customize the behaviour of pose estimation.
 * All parameters are configured in a key = value fashion, to set them see PoseEstimation class methods.
 *
 * Basic Parameters
 * ------------------
 *
| Key     | Default Value | Description                                                          |
|:-------:|:-------------:|:---------------------------------------------------------------------|
|useVFH   | 1             | Tell PoseEstimation to use Viewpoint Feature Histogram (VFH) in matching phase|
|useESF   | 1             | Tell PoseEstimation to use Ensemble of Shape Functions (ESF) in matching phase|
 
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

/** \addtogroup Definitions
 * 
 * Easier code writing
 * @{ */
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
typedef unordered_map<string,float> parameters;
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
  vector<int> clusters_cvfh_, clusters_ourcvfh_;
  boost::filesystem::path dbPath_; 
  vector<PC> clouds_;
  boost::shared_ptr<indexVFH> vfh_idx_;
  boost::shared_ptr<indexESF> esf_idx_;

  /**\brief Calculates unnormalized distance of objects, based on their cluster distances, internal use.
   * \param[in] query Pointer to the query histogram(s)
   * \param[in] idx Index of object to compare with
   * \param[in] feature String containing either "CVFH" or "OURCVFH", to tell the method which feature is involved
   * \param[out] distance Unnormalized distance of object at index from query
   */
  void computeDistanceFromClusters_(PointCloud<VFHSignature308>::Ptr query, int idx, string feature, float& distance);

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
     * Return true if succesfull, false otherwise
     */
    void save(boost::filesystem::path pathDB);
    
    /** \brief Compute the whole database from scratch and store it in memory.
     * \param[in] pathClouds Path to a directory on disk that contains all the pcd files of object poses
     * \param[in] params Shared pointer to parameters to use during database creation
     * 
     * This method uses the provided set of parameters to create the database, erasing any previously loaded databases.
     * Please note that:
- Constructing a database from scratch can take several minutes at least.
- In order to use this method, PCD files must follow a naming convention, that is objName_latitude_longitude.pcd  (i.e. funnel_20_30.pcd). Not using this naming convention may result in corrupted or unusable database.
- PCD files must represent previously segmented objects and must be expressed in a local reference system, i.e. a system centered at the object base. This system must be consistent with all the PCDs provided.
- PCDs should have stored the viewpoint location (coordinates of where the sensor was positioned during acquisition) inside their sensor_origin_ member for optimal results, although this is not mandatory
     */
    void create(boost::filesystem::path pathClouds, boost::shared_ptr<parameters> params);
    
    /** \brief Compute the whole database from scratch and store it in memory.
     * \param[in] pathClouds Path to a directory on disk that contains all the pcd files of object poses
     *  
     * This method creates a set of default parameters and creates the database from it, erasing any previously loaded databases. 
     * Please note that:
- Constructing a database from scratch can take several minutes at least.
- In order to use this method, PCD files must follow a naming convention, that is objName_latitude_longitude.pcd  (i.e. funnel_20_30.pcd). Not using this naming convention may result in corrupted or unusable database.
- PCD files must represent previously segmented objects and must be expressed in a local reference system, i.e. a system centered at the object base. This system must be consistent with all the PCDs provided.
- PCDs should have stored the viewpoint location (coordinates of where the sensor was positioned during acquisition) inside their sensor_origin_ member for optimal results, although this is not mandatory
     */
    void create(boost::filesystem::path pathClouds);
      
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
    
    /** \brief Get Candidate Rank in the list of candidates
     * \param[out] r The rank the candidate has in the list
     */
    void getRank (int& r) const;
    
    /** \brief Get the distance of Candidate from Query in the metric chosen by the feature
     * \param[out] d The distance the candidate has from the query
     */
    void getDistance (float& d) const;
    
    /** \brief Get the Normalize distance of Candidate from Query
     * \param[out] d The distance the candidate has from the query
     * Normalized distances range from 0 to 1, zero at "rank 1" and one at "rank k"
     */
    void getNormalizedDistance (float& d) const;
    
    /** \brief Get Root Mean Square Error of Candidate as it was after the refinement
     * \param[out] e The Root Mean Square Error of the Candidate
     */
    void getRMSE (float& e) const;
    
    /** \brief Get Homogeneous Transformation that brings the Candidate over the Query
     * \param[out] t Transformation that brings the Candidate over the Query
     * The transformation is expressed in the Candidate Reference System
     */
    void getTransformation (Eigen::Matrix4f& t) const;
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
- Generate the list(s) of candidates to the query with member generateLists()
\code
  pe.generateLists(); //list of candidates are computed, based on type of parameters set
\endcode
- Obtain the final candidate with member refineCandidates()
\code
  pe.refineCandidates(); //Select one final candidate from the lists, representing the pose estimation
\endcode
- Print, Get or Save the final pose estimation with the corresponding member functions
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
  pe.estimate("query object", cloud, "DB_PATH"); //calls setQuery(), setDatabase(), generateList(), refineCandidates()
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
  
  ///List of candidates to the query
  vector<Candidate> vfh_list_, esf_list_, cvfh_list_, ourcvfh_list_, composite_list_;

  ///Path to the directory containing the database of known poses, created previously
  boost::filesystem::path dbPath_;
  
  ///Viewpoint coordinates, used in computations like VFH and Normal estimation
  float vpx_, vpy_, vpz_;
  
  ///Internal parameters to check if various stages of pose estimation are completed correctly
  bool vp_supplied_, query_set_, candidates_found_, refinement_done_, db_set_;
  
  ///Containers that hold the query features
  PointCloud<VFHSignature308> vfh_, cvfh_, ourcvfh_;
  PointCloud<ESFSignature640> esf_;
  PointCloud<Normal> normals_;
  
  ///Set a parameter of Pose Estimation from a string representing its value, used internally when reading parameters from a file
  void setParam_ (string, string&);
  
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

  /**\brief Searches a list for a candidate and eliminates it, saving its distance. internal use
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
  * 
  * Example Usage:
  * \code
  * PoseEstimation pe; //construct and sets default parameters
  * pe.setParam("useVFH", 0); //pe now skips VFH computation for query
  * string str;
  * str = "verbosity";
  * pe.setParam(str, 2); //pe now has verbosity set to 2
  * \endcode
  */
  void setParam (string key, float value);
  
  /**\brief Set a parameter of the Class directly, knowing its name
  * \param[in] key the parameter name to change
  * \param[in] value the value that key should assume
  *
  * Overloaded for ints
  */
  void setParam (string key, int value) {this->setParam(key, (float)value);}
  
  /**\brief Set a parameter of the Class directly, knowing its name
  * \param[in] key the parameter name to change
  * \param[in] value the value that key should assume
  *
  * Overloaded for double
  */
  void setParam (string key, double value) {this->setParam(key, (float)value);}
  
  /** \brief Initialize the class with parameters found in config file
  * \param[in] config_file Path to a config file to use 
  *
  * Configuration file must have extension .conf and follow certain naming conventions, 
  * look at example .conf file provided for more details (i.e "config/parameters.conf")
  */
  void initParams (boost::filesystem::path config_file);

  /** \brief Initialize parameters of PoseEstimation from the map provided
   * \param[in] map shared pointer to unordered_map containing parameters to use
   *
   * unordered_map must contain valid keys and values, otherwise they will be ignored
   */
  void initParams (boost::shared_ptr<parameters> map);
  
  /**\brief Constructor with parameters to set.
   *\param[in] map Shared pointer to unordered_map containing parameters to use
   *
   * NOTE: This constructor uses C++11 functionality and will probably not compile without -std=c++11 
   * It delegates construction to empty contructor then calls initParams()
   * It is the same way as calling empty constructor and then initParams() method
   */
  PoseEstimation(boost::shared_ptr<parameters> map) : PoseEstimation() {this->initParams(map);}
  
  /**\brief Constructor with path to a configuration file containing parameters to set.
   *\param[in] config_file Path to a config file to use
   *
   * Configuration file must have extension .conf and follow certain naming conventions, 
   * look at example .conf file provided for more details (i.e "config/parameters.conf")
   * NOTE: This constructor uses C++11 functionality and will probably not compile without -std=c++11 
   * It delegates construction to empty contructor then calls initParams()
   * It is the same way as calling empty constructor and then initParams() method
   */
  PoseEstimation(boost::filesystem::path config_file) : PoseEstimation() {this->initParams(config_file);}

  /**\brief Explicitly set the query viewpoint
   *\param[in] x Coordinate x of the viewpoint
   *\param[in] y Coordinate y of the viewpoint
   *\param[in] z Coordinate z ot the viewpoint
   *
   * THIS METHOD OVERRIDES ANY VIEWPOINT PARAMETERS SET, thus Pose Estimation
   * will ignore "useSOasViewpoint" and "computeViewpointFromName" parameters regardless of their value, 
   * and will use the viewpoint set this way for the computations where a viewpoint is needed 
   * (normals, VFH, CVFH ...)
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

  /// \brief Print List of Candidates to the query on screen
  void printCandidates();

  /** \brief Set a database of known poses to be used for pose estimation procedure.
   * \param[in] dbPath The path to the directory containing the database
   */
  void setDatabase(boost::filesystem::path dbPath);
  /** \brief Set a database of known poses to be used for pose estimation procedure.
   * \param[in] database PoseDB ojbect to use as database
   */
  void setDatabase(PoseDB& database);

  /** \brief Generates list(s) of candidates to the query using the database provided as argument
   * \param[in] dbPath The path to the directory containing the database of poses, previously generated.
   * 
   * This member also sets the database to the one specified for future lists computations.
   */
  void generateLists(boost::filesystem::path dbPath);
  
  /** \brief Generates list(s) of candidates to the query using previously set database
   */
  void generateLists();

  /**\brief Start the refinement procedure with ICP to obtain a final candidate from the composite list
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
  void refineCandidates();

  /** \brief Undergo the whole process of pose estimation
   *\param[in] name Name of the new query object to estimate
    \param[in] cloud Point cloud containing the query object to estimate (segmented)
    \param[in] db_path Path to a directory containing the database of poses to load

    This method calls is sequence setQuery() setDatabase() generateLists() and refineCandidates(), using the
    already set parameters 
    */
  void estimate(string name, PC& cloud, boost::filesystem::path db_path);
  
  /** \brief Undergo the whole process of pose estimation
   *\param[in] name Name of the new query object to estimate
    \param[in] cloud_pointer Pointer to a point cloud containing the query object to estimate (segmented)
    \param[in] db_path Path to a directory containing the database of poses to load

    This method calls is sequence setQuery() setDatabase() generateLists() and refineCandidates(), using the
    already set parameters 
    */
  void estimate(string name, PC::Ptr cloud_pointer, boost::filesystem::path db_path);
  
  /** \brief Undergo the whole process of pose estimation
   *\param[in] name Name of the new query object to estimate
    \param[in] cloud Point cloud containing the query object to estimate (segmented)
    \param[in] database PoseDB object to use as database 

    This method calls is sequence setQuery() setDatabase() generateLists() and refineCandidates(), using the
    already set parameters 
    */
  void estimate(string name, PC& cloud, PoseDB& database);
  
  /** \brief Undergo the whole process of pose estimation
   *\param[in] name Name of the new query object to estimate 
    \param[in] cloud_pointer Pointer to a point cloud containing the query object to estimate (segmented)
    \param[in] database PoseDB object to use as database 

    This method calls is sequence setQuery() setDatabase() generateLists() and refineCandidates(), using the
    already set parameters 
    */
  void estimate(string name, PC::Ptr cloud_pointer, PoseDB& database);

  /** \brief Print final estimation informations (such as name, distance, rmse and transformation) on screen
   */
  void printEstimation();

  /**\brief Save final estimation informations (such as name, distance, rmse and transformation) on a file with path specified
      \param[in] file Path to a location on disk, where the file will be created or renewed
      \param[in] append Chose to append to the end of file (true)(default), or truncate its contents (false), if file does not exists this parameter has no effect
      \return _True_ if file is correctly written, _False_ otherwise

      The path can specify a directory or a file, if the former a new text file of name "query_name.estimation" will be created inside the specified directory.
      If the latter, the specified file will be written.
      Note on paths: if file path does not exists, a file must be specified with an extension, otherwise it will be treated as a directory.
      For example "/foo/bar" or "/foo/bar/" will be both treated as directories if they don't exists already.
      "foo/bar.f" will be treated as file "bar.f" inside relative directory "foo"
      */
  bool saveEstimation(boost::filesystem::path file, bool append = true);
  /** \brief Returns a shared pointer of a copy of parameters used by the pose estimation
   *
   * 
   * Returned pointer can be modified but any changes are not reflected back to the class,
   * use setParam() or initParams() to modify PoseEstimation parameters
  */ 
  boost::shared_ptr<parameters> getParameters();

  /** \brief Reset the viewpoint for current query, so that i can be computed again
   */
  void resetViewpoint(){ vp_supplied_ = false; }
};
#endif
