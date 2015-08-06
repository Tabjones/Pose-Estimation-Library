/*
 * Software License Agreement (BSD License)
 *
 *   Pose Estimation Library (PEL) - https://bitbucket.org/Tabjones/pose-estimation-library
 *   Copyright (c) 2014-2015, Federico Spinelli (fspinelli@gmail.com)
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder(s) nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef PEL_POSE_ESTIMATION_H_
#define PEL_POSE_ESTIMATION_H_

#include <pel/database/database.h>
#include <pel/candidates/candidate.h>
#include <pel/param_handler.h>
#include <cmath>
#include <stdexcept>
#include <pcl/common/norms.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
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
#include <pcl/visualization/histogram_visualizer.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/filesystem.hpp>

namespace pel
{
  /**\brief PoseEstimation base class.
  //TODO rewrite docs

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
  class PoseEstimationBase : public ParamHandler, public Database
  {
    protected:
      ///Pointer to the final Candidate that holds the pose estimation of the query
      boost::shared_ptr<Candidate> pose_estimation_;

      ///Internal counter used to count how many feature the class uses
      int feature_count_;

      ///The name of the query to estimate
      std::string query_name_;

      ///The cloud pointer that represent the query to estimate as supplied before any computation
      PtC::Ptr target_cloud_;

    ///Internal parameter to check if the target was succesfully set and its features estimated
    bool target_set_;
    ///Internal parameter to check if list(s) of candidates are successfully generated
    bool candidates_found_;
    ///Internal parameter to check if candidate refinement has been done and it was succesfull
    bool refinement_done_;
    ///Internal parameter to check if a database was loaded in memory and it's now ready to be used
    bool db_set_;
    ///Internal parameter to check if query was supplied in a local refernce system
    bool local_query_;

    ///Container that hold the query VFH feature
    pcl::PointCloud<pcl::VFHSignature308> vfh_;
    ///Container that hold the query CVFH feature
    pcl::PointCloud<pcl::VFHSignature308> cvfh_;
    ///Container that hold the query OURCVFH feature
    pcl::PointCloud<pcl::VFHSignature308> ourcvfh_;
    ///Container that hold the query ESF feature
    pcl::PointCloud<pcl::ESFSignature640> esf_;
    ///Container that hold the query normals
    pcl::PointCloud<pcl::Normal> normals_;
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
    bool findCandidate_(std::vector<Candidate>& list, std::string name, float& dist);

    public:
    ///Default Empty Constructor that sets default parameters (see them in config file "config/parameters.conf")
    PoseEstimation();

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

    /**\brief Set the Pose Estimation query (the object to be identified)
     * \param[in] str The name the query should assume
     * \param[in] cl  Point cloud containing only the object to be estimated (i.e. already segmented)
     */
    void setQuery (std::string str, PtC& cl);

    /** \brief Set the Pose Estimation query (the object to be identified)
     * \param[in] str the name the query should assume
     * \param[in] clp Shared pointer containing only the pointcloud of the object to be estimated (i.e. already segmented)
     */
    void setQuery (std::string str, PtC::Ptr clp);

    /** \brief Set a database of known poses to be used for pose estimation procedure.
     * \param[in] dbPath The path to the directory containing the database
     *
     * Note that this method calls PoseDB::load() on the path specified
     */
    void setDatabase(boost::filesystem::path dbPath);
    /** \brief Set a database of known poses to be used for pose estimation procedure.
     * \param[in] database PoseDB object to use as database
     */
    void setDatabase(Database& database);

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
    bool generateLists(Database& db);

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
    bool estimate(std::string name, PtC& cloud, boost::filesystem::path db_path);

    /** \brief Undergo the whole process of pose estimation
     *\param[in] name Name of the new query object to estimate
     \param[in] cloud_pointer Pointer to a point cloud containing the query object to estimate (segmented)
     \param[in] db_path Path to a directory containing the database of poses to load
     \return _True_ if pose estimation was successful in all its parts, _false_ otherwise

     This method calls is sequence setQuery() setDatabase() generateLists() and refineCandidates(), using the
     already set parameters
     */
    bool estimate(std::string name, PtC::Ptr cloud_pointer, boost::filesystem::path db_path);

    /** \brief Undergo the whole process of pose estimation
     *\param[in] name Name of the new query object to estimate
     \param[in] cloud Point cloud containing the query object to estimate (segmented)
     \param[in] database PoseDB object to use as database
     \return _True_ if pose estimation was successful in all its parts, _false_ otherwise

     This method calls is sequence setQuery() setDatabase() generateLists() and refineCandidates(), using the
     already set parameters
     */
    bool estimate(std::string name, PtC& cloud, Database& database);

    /** \brief Undergo the whole process of pose estimation
     *\param[in] name Name of the new query object to estimate
     \param[in] cloud_pointer Pointer to a point cloud containing the query object to estimate (segmented)
     \param[in] database PoseDB object to use as database
     \return _True_ if pose estimation was successful in all its parts, _false_ otherwise

     This method calls is sequence setQuery() setDatabase() generateLists() and refineCandidates(), using the
     already set parameters
     */
    bool estimate(std::string name, PtC::Ptr cloud_pointer, Database& database);

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

    /** \brief Get a pointer to a copy of final chosen Candidate, representing the pose estimation
     * \return A pointer to a copy of the final Candidate
     */
    boost::shared_ptr<Candidate> getEstimation();

    /** \brief Get a pointer to a copy of final chosen Candidate, representing the pose estimation
     * \param[out] est A pointer to a copy of the final Candidate chosen by refinement
     * \return _True_ if est correctly points to final pose estimation, _false_ otherwise
     */
    bool getEstimation(boost::shared_ptr<Candidate> est);

    /** \brief Get a copy of final poseEstimation transformation
     * \param[out] t A matrix that will contain the final transformation
     * \return _True_ if t is correctly initialized, _false_ otherwise
     */
    bool getEstimationTransformation(Eigen::Matrix4f& t);

    /** \brief Print some information on Query object on screen
    */
    void printQuery();

    /** \brief Get name and  pointers holding point clouds of query object (before and after eventual preprocessing)
     * \param[out] name String containing query name
     * \param[out] clp Shared Pointer to a copy of query point cloud after eventual preprocessing
     * \return _True_ if operation was successful, _false_ otherwise
     *
     * If no preprocessing was done (i.e. downsampling, upsampling and filtering parameters are all zero), clp and clp_pre point the same cloud
     */
    bool getQuery (std::string& name, PtC::Ptr clp);

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
    int getQueryFeatures(pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh, pcl::PointCloud<pcl::VFHSignature308>::Ptr cvfh, pcl::PointCloud<pcl::VFHSignature308>::Ptr ourcvfh, pcl::PointCloud<pcl::ESFSignature640>::Ptr esf, pcl::PointCloud<pcl::Normal>::Ptr normals);

    /** \brief Visualize the query into a PCL visualizer, this method blocks execution until the visualizer is closed. Query cloud is displayed after preprocessing
    */
    void viewQuery();

    /** \brief Visualize Pose Estimation into a PCL visualizer, this method blocks execution until the visualizer is closed.
     *
     * Query point cloud is displayed and on top of it the aligned candidate in green color. Other informations are displayed in the visualizer window
     */
    void viewEstimation();

    /** \brief Register pose estimation results into a file for testing purpose
     *  \param[in] file path to a txt file to write into, if file already exists it will be appended.
     *  \param[in] gt ground truth reference to check correctness, must be in form <obj>_<lat>_<long>.
     *  \return _True_ if operation is succesful, _false_ otherwise.
     *
     *  Writes into a file pose estimation results for testing purpose. Given the final pose estimation, it saves the ranks of each list
     *  on which ground truth was found. It also saves if the chosen final candidate matches the ground truth or not. For example a line could be like this:
     *  \code
     *  #VFH  #ESF  #CVFH #OURCVFH  #COMP   #FINAL
     *  2     1     6      4        1       1
     *  \code
     *  In this example the ground truth was found on rank 2 in VFH list, rank 1 in ESF, etc...
     *  Plus the final candidate chosen was correct (1), because it matches the ground truth name.
     *  Final candidate can be:
     *    + 0: If ground truth and pose estimation match completely (success)
     *    + 1: If ground truth and pose estimation match with a tolerance of 10 degrees in latitude and/or longitude (direct neighbor)
     *    + 2: If ground truth and pose estimation match only in names, hence orientation is wrong, but object was correctly identified (same object)
     *    + 3: If ground truth and pose estimation do not match, hence the estimation was wrong (false)
     */
    bool saveTestResults(boost::filesystem::path file, std::string gt);

    /** \brief Register pose estimation results into a file for testing purpose
     *  \param[in] file path to a txt file to write into, if file already exists it will be appended.
     *  \return _True_ if operation is succesful, _false_ otherwise.
     *
     *  Writes into a file pose estimation results for testing purpose. Given the final pose estimation, it saves the ranks of each list
     *  on which ground truth was found, ground truth used is query name, that must follow naming convention <obj>_<lat>_<long>. It also saves if the chosen final candidate matches the ground truth or not. For example a line could be like this:
     *  \code
     *  #VFH  #ESF  #CVFH #OURCVFH  #COMP   #FINAL
     *  2     1     6      4        1       1
     *  \code
     *  In this example the ground truth was found on rank 2 in VFH list, rank 1 in ESF, etc...
     *  Plus the final candidate chosen was correct (1), because it matches the ground truth name.
     *  Final candidate can be:
     *    + 0: If ground truth and pose estimation match completely (success)
     *    + 1: If ground truth and pose estimation match with a tolerance of 10 degrees in latitude and/or longitude (direct neighbor)
     *    + 2: If ground truth and pose estimation match only in names, hence orientation is wrong, but object was correctly identified (same object)
     *    + 3: If ground truth and pose estimation do not match, hence the estimation was wrong (false)
     */
    bool saveTestResults(boost::filesystem::path file);

    /** \brief Elaborate test results written with saveTestResults()
     * \param[in] file test file written by saveTestResults()
     * \param[in] result path to a file in which write results
     * \return _True_ if elaboration is successful, _false_ otherwise
     *
     * Result file will be truncated, if it already exists
     */
    bool elaborateTests(boost::filesystem::path file, boost::filesystem::path result);
  };
}

#endif //PEL_POSE_ESTIMATION_H_
