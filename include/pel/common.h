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
 *   list of conditions and the following disclaimer.nce
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

//Doxygen Manual Page//////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

/*! \mainpage notitle
 *  \section intro Introduction
 *  Pose Estimation Library (PEL) provides tools to perform accurate and fast pose estimations of objects, represented by point clouds, to be used in Robotics applications in order
 *  to achieve robust grasps. Since PEL uses point clouds, it makes extensive use of <a href="http://pointclouds.org">Point Cloud Library (PCL)</a> for filtering, features and transformation estimations needed
 *  for Pose Estimation applications. In particular, PEL employs global features to describe poses of objects (or views) in order to populate a database of known objects, then when a new view is presented (the target),
 *  it tries to find the _k_ most similar views from database (or candidates). Finally a single candidate is selected from the pool, by means of iterative procedures, which use ICP algorithm. The final candidate, along with
 *  its transformation aliging it over the target, is the sought pose estimation.
 *
 *  Pose Estimation Library is mainly developed within the <a href="http://www.pacman-project.eu/">PaCMan Project</a> as a tool to obtain fast and accurate
 *  Pose Estimations of known graspable objects. The work implemented here is derived from the author's master thesis in Robotics Engineering, which is available
 *  online, as a further in depth reference, from University of Pisa <a href="https://etd.adm.unipi.it/theses/available/etd-09022014-142255/unrestricted/tesi.pdf">Online Thesis Archive</a>.
 *
 *  Full source code and installation instructions are available on <a href="https://bitbucket.org/Tabjones/pose-estimation-library">Bitbucket</a>.
 *  \section guide User Guide
 *  Namespace pel::interface provides the user with some classes implementing Pose Estimation procedures. The goal of the procedure is to identify an object view, called Target, and estimate
 *  its 6DOF pose in space.
 *  Currently two procedures exists: Brute Force and Progressive Bisection with their multi-threaded variants.
 *  The library is designed in a way that the user should only use the classes exposed in namespace pel::interface, because they contain all the aspects and procedure
 *  needed to achieve Pose Estimation of objects. From now on the above mentioned classes will be referred to as _interface classes_. Each _interface class_ follows the
 *  same conceptual workflow, but differs from the others in the last "refinement" step.
 *  The workflow is summarized as follows:
 *    1. Estimate global features of Target.
 *    2. Assemble Lists of Candidates, by querying Database with Target features.
 *    3. Assemble a _composite list_ by building a consensus among all features list.
 *    4. Refine the composite list selecting one Candidate to be the Pose Estimation.
 *
 *  In _step 1_ a Target point cloud, representing a view of an object, is initialized and its features estimated. According to some parameters, discussed later on, Target point cloud may or may not undergo
 *  a series of pre processing steps, like downsampling or outliers filtering; then global features are estimated from target point cloud and used in the next step to assemble lists of candidates.
 *
 *  In _step 2_ Target global features are matched against a database of features, retrieving _k nearest neighbors_ for each feature; Database assembling is discussed later on the guide. For every feature, each neighbor retrieved
 *  is sorted according to minimum distance from Target feature, thus assembling a list of "Candidates" that are the most similar views to the Target, in terms of geometric shape.
 *
 *  _Step 3_ takes all the lists created from each single feature and tries to assemble, or fuse them, into a single list, called the Composite List. This is done to robustify the Candidates, by building a sort of consensus
 *  among different features. This also increases recognition performance of the procedure.
 *
 *  The last step (_4_) analyzes the composite list and tries to determine which of the _k_ candidates is the best approximation for the target. It does this by trying to overlap a Candidate point cloud over the Target one and measures the root
 *  mean square error of point distances. The procedure outputs a rigid homogeneous transformation which express the 6DOF pose of the Target, thus completing the Pose Estimation.
 *  All the procedure is discussed in detail into the <a href="https://etd.adm.unipi.it/theses/available/etd-09022014-142255/unrestricted/tesi.pdf">author's Master Thesis</a>.

 *  The user can control all the four aspect of the procedure from _interface classes_ directly, by using inherited members.
 *  In detail:
 *    - For step _1_, _2_ and _3_ various parameters are provided to control features estimation and lists generation (pel::ParamHandler members).
 *    - For step _4_, _interface class_ provides specialized methods to control the behaviour of the procedure.
 *    - Database, used during the procedure, can be inspected from pel::Database inherited members.
 *  The rest of the section explains how the user can control the above mentioned aspects, and provides some working examples.
 *
 *  Plese refer to the index below:
 *    - \ref params - For handling parameters and their description.
 *    - \ref database - For Database usage and creation.
 *    - \ref refine - For Refinements Methods and _interface classes_ usage.
 *    - \ref apps - For useful example applications.
 *
 *  \subsection params Parameters
 *  Parameters are handled in a key=value fashion and are already initialized and set to default values
 *  when the user constructs one the interface class. The main methods to read and set a parameter are
 *  pel::ParamHandler::getParam(key) and pel::ParamHandler::setParam(key, value), inherited by _interface classes_.
 *  Note that each parameter has minimum and maximum allowable values, even if they are _floats_ internally. Also each method to handle parameters won't let
 *  you set a key that doesn't exist or an out of range value. All available parameters are described in the table below:

| key         | Default Value | Range | Description                                                          |
|:-----------:|:-------------:|:------:|:-----------------------------------------------------------------------|
| cvfh_ang_thresh | 7.5     |>0 | Set maximum allowable deviation of the normals in degrees, in the region segmentation step of CVFH computation. The value recommended from relative paper is 7.5 degrees. Relevant only if use_cvfh is enabled.<sup>2</sup>|
| cvfh_curv_thresh  | 0.025 |>0 | Set maximum allowable disparity of curvatures during region segmentation step of CVFH estimation. The value recommended from relative paper is 0.025. Relevant only if use_cvfh is enabled.<sup>2</sup>|
| cvfh_clus_tol  | 0.01     | >0 | Euclidean clustering tolerance, during CVFH segmentation. Points distant more than this value from each other, will likely be grouped in different clusters. A value of 1 means one meter. Relevant only if use_cvfh is enabled.<sup>2</sup>|
| cvfh_clus_min_points  | 50 | >=1 | Set minimum number of points a cluster should contain to be considered such, during CVFH clustering. Relevant only if use_cvfh is enabled.<sup>2</sup>|
| downsamp    |  1           | 0 or 1 | (1) Downsample the Target point cloud with Voxel Grid after eventual outlier filter and MLS resampling. (0) Don't apply this filter.<sup>2</sup>|
| downsamp_leaf_size| 0.005  | >0     | Set leaf size of Voxel Grid downsampling, a value of 1 means one meter. Only relevant if Voxel Grid is enabled.<sup>2</sup>|
| filter     | 0            | 0 or 1| (1) Filter Target point cloud with Statistical Outliers Removal, before the eventual upsampling and downsampling. (0) Or don't apply this filter.<sup>2</sup>|
| filter_mean_k | 50        | >0 | How many neighboring points to consider in the statistical distribution calculated by the filter, relevant if filter is enabled.<sup>2</sup>|
| filter_std_dev_mul_thresh | 3 | >0 | Multiplication factor to apply at Standard Deviation of the statistical distribution during filtering process (higher value, means less aggressive filter). Relevant only if filter is enabled.<sup>2</sup>|
| lists_size | 20           | >=1 | The size of generated lists of Candidates. Also the k-nearest neighbors to the Target retrieved from Database. Increasing this value may increase recognition rate at the cost of computational time.|
| normals_radius_search | 0.02 | >0 | Set radius that defines the neighborhood of each point during Normal Estimation, value of 1 means one meter. If normals are not computed, i.e. only ESF is estimated, this parameter is ignored.<sup>2</sup>|
| ourcvfh_ang_thresh  | 7.5 |>0 | Set maximum allowable deviation of normals, in the region segmentation step of OURCVFH computation. The value recommended from relative paper is 7.5 degrees. Relevant only if use_ourcvfh is enabled.<sup>2</sup>|
| ourcvfh_curv_thresh | 0.025 |>0| Set maximum allowable disparity of curvatures during region segmentation step of OURCVFH estimation. The value recommended from relative paper is 0.025. Relevant only if use_ourcvfh is enabled.<sup>2</sup>|
| ourcvfh_clus_tol | 0.01 | >0 | Euclidean clustering tolerance, during OURCVFH segmentation. Points distant more than this value from each other, will likely be grouped in different clusters. A value of 1 means one meter. Relevant only if use_ourcvfh is enabled.<sup>2</sup>|
| ourcvfh_clus_min_points  | 50 | >=1 | Set minimum number of points a cluster should contain to be considered such, during OURCVFH clustering. Relevant only if use_ourcvfh is enabled.<sup>2</sup>|
| ourcvfh_axis_ratio  | 0.95 | >0 | Set the minimum axis ratio between the SGURF axes. At the disambiguation phase of OURCVFH, this will decide if additional Reference Frames need to be created for the cluster, if they are ambiguous. Relevant only if use_ourcvfh is enabled.<sup>2</sup>|
| ourcvfh_min_axis_value  | 0.01 | >0 | Set the minimum disambiguation axis value to generate several SGURFs for the cluster when disambiguition is difficult. Relevant if use_ourcvfh is enabled.<sup>2</sup>|
| ourcvfh_refine_clusters  |  1 | >=0, <=1 | Set refinement factor for clusters during OURCVFH clustering phase, a value of 1 means 'dont refine clusters', while values between 0 and 1 will reduce clusters size by that number. Relevant only if use_ourcvfh is enabled.<sup>2</sup>|
| use_vfh     | 1            | 0 or 1 | (1) Use the Viewpoint Feature Histogram (VFH) in feature estimation of Target. (0) Or disable it.<sup>1</sup>|
| use_esf     | 1            | 0 or 1 | (1) Use the Ensemble of Shape Functions (ESF) in feature estimation of Target. (0) Or disable it.<sup>1</sup>|
| use_cvfh    |           1  | 0 or 1 | (1) Use Clustered Viewpoint Feature Histogram (CVFH) in feature estimation of Target. (0) Or disable it.<sup>1</sup>|
| use_ourcvfh |           1  | 0 or 1 | (1) Use Oriented, Unique and Repeatable CVFH  in feature estimation of Target. (0) Or disable it.<sup>1</sup>|
| upsamp      | 0            | 0 or 1 | (1) Resample the Target point cloud with MLS at random uniform density, before the eventual downsampling with Voxel Grid. (0) Don't apply this filter.<sup>2</sup>|
| upsamp_poly_order| 2        | >1 | Set polynomial order of MLS to the value specified, for most objects  second order polynomial functions are more than enough to correctly approximate the object surface. Relevant only if upsamp is enabled.<sup>2</sup>|
| upsamp_point_density|  200 | >=1 | Set desired point density on MLS surface to the value specified,  higher values produce more points, thus increasing the upsampling ratio. Relevant only if upsamp and upsamp_poly_fit is enabled.<sup>2</sup>|
| upsamp_poly_fit| 1          | 0 or 1 | (1) Fit MLS surface on polynomial functions with order upsamp_poly_order. (0) Or don't use polynomial fitting. Relevant only if upsamp is enabled.<sup>2</sup>|
| upsamp_search_radius| 0.05 | >0 | Set search radius of MLS to the value specified, a value of 1 means one meter. Relevant if upsamp is enabled.<sup>2</sup>|
| verbosity   | 1            | 0,1 or 2| Controls the verbosity of PEL (0) Silent behaviour, only errors are printed, (1) Normal behaviour, errors and warnings are printed, (2) Verbose behaviour, spams all kind of messages.|

Notes:
  1. At least one feature must be enabled. Also if you disable this feature during Database creation (see pel::DatabaseCreator), resulting database will not have that feature, so all pose estimations using that database must also disable the feature.
  2. For optimal results, all parameters regarding filtering and feature estimation should be the same between Database and pose estimation Target.

 *
 * \subsection database Database
 * Pose Esimation procedure needs a Database to match Target features, so that the current object view can be compared and approximated with one of those in the Database. The user can download the PaCMan object Databse <TODO ADD LINK>, and load it in its code with
 * pel::DatabaseReader class. This Database is composed by 46 kitchen objects from Ikea catalog, each one is composed by 108 point clouds from various viewpoints, all their features estimated with default parameters (see \ref params section above) and a reconstructed
 * polygonal mesh of the object; useful for visualization purposes.
 *
 * User can also build its own custom Database with pel::DatabaseCreator class, however the procedure to create Databases is a bit more involved and requires the user to follow these guidelines:
 *
 *  1. Prepare point clouds of objects:
 *    - Choose a "local" reference system for each object you want to acquire; for example you could center it on the object centroid, have _z-axis_ pointing up and _x-axis_ pointing to an instantly recognizable object feature (like an handle). See the example picture below.
 *    - For each object, acquire _n_ partial point clouds of the object representing various views, having care to:
 *      1. Express all the point clouds in the "local" reference system for this object, you chose at previous step. I.E. the points in the cloud must have euclidean coordinates expressed in the "local" reference system, so a transformation is needed.
 *        Please note that all the _n_ clouds of the object must be expressed in the same "local" reference system.
 *      2. Save the transformation to go back in the kinect reference frame, i.e. the inverse of the one you calculated to go from kinect to local. Save this transformation into sensor_origin_ and sensor_orientation_ fields of the pcl::PointCloud class, for each point cloud.
 *      3. Segment all the object views, so that only the points belonging to the object remain. For example you have to remove the table underneat the object and the surrounding environments.
 *      4. Save all the clouds of all objects into the same directory, each pcd file must have unique name.
 *    - Optionally you could undergo a registration process for the clouds of each object, to make sure each one has the same reference system as accurately as possible. For example you could register all the clouds on top of the first.
 *  2. Make a program that uses pel::DatabaseCreator and pass it the previous saved clouds directory.
 *  3. Adjust pel::DatabaseCreator parameters to your need with inherited members (look at \ref params previous section). Please note that each pose estimation using this database must ratain the same parameters.
 *  4. Build the Database and save it somewhere on disk with pel::DatabaseWriter class.
 *
 ![Example of two poses of the same object taken with asus xtion sensor. Note how both point clouds are expressed in the same reference frame, even if they are taken from different viewpoints.](@ref mugposes.png)
 *
 * \subsection refine Refinement
 * Refinement is the process involved when selecting one Candidate among a list of Candidates. Each Candidate represents an object view from Database, that approximates best the Target view (the one you want to identify and estimate).
 * When Refinement takes place, the best _k_ Candidates that approximate the Target are already selected and assembled in the _composite list_. Refinemente procedure selects which of these _k_ Candidates is the best Pose Estimation
 * of the Target.
 * Currently two procedures are available, each will be described further down the section:
 *  - Brute Force.
 *  - Progressive Bisection.
 *
 * The user can select one of these procedures by using the relative classes: pel::interface::PEBruteForce, pel::interface::PEProgressiveBisection.
 * Each class implements the whole Pose Estimation procedure and differs from one another only in the refinement procedure. After pose estimation terminates the user can access the transformation, expressing the 6DOF pose of the Target in the sensor reference system,
 * with the method pel::Candidate::getTransformation().
 *  \subsubsection brute Brute Force
 *  Brute Force simply selects the Candidate at least distance (in feature space) from the Target and starts aligning its point cloud with ICP over Target point cloud. If its Root Mean Square Error (RMSE) falls below an user set threshold, the procedure
 *  terminates and the Candidate it's elected as Pose Estimation of the Target. The Candidate's transformation expresses the 6DOF pose of the Target with respect to the sensor reference frame (i.e. the kinect).
 *
 *  If the Candidate selected does not converge and the fixed user set number of ICP steps are expired, the procedure selects the second Candidate on the list and restarts aligning from it.
 *  If no Candidates on the lists converge the procedure fails.
 *
 *  The algorithm is summarized as follows:
 *  1. Align Candidate over Targer for _n_ steps of ICP.
 *  2. If convergence, Candidate is Pose Estimation and terminate.
 *  3. Repeat from point 1 with next Candidate. When no more Candidates are left to try, procedure fails and terminates.
 *
 *  Brute Force is generally less performing and more time consuming than the next method, however is the most used one in Pose Estimation literature.
 *
 *  \subsubsection progressive Progressive Bisection
 *  Progressive Bisection does a few steps (typically 5) of ICP alignment for all the Candidates on composite list, then evaluates their Root Mean Square Error (RMSE) and resort the list by minimum RMSE. Finally truncates a portion of the list (typically its half) and
 *  repeat with alignment. This goes on until a Candidate converges (under the user set threshold) or composite list size is one.
 *
 *  Basically the procedure "tastes" all the Candidates and determines which one is more likely to converge in the next ICP steps, positioning them on top of the list. Then discards the "bad" Candidates, located near the bottom, concentrating resources on aligning
 *  the "good" ones, which are progressively moved on top.
 *  The user can select if the procedure is to fail when no Candidate converges or if it has to take as correct Pose Estimation the only Candidate remained in the list after progressive bisections. This is generally safe to set
 *  (in fact it's the default behaviour), because the last Candidate remained is guaranteed to be the best (in terms of RMSE) we could find in the list, thus is likely an accurate enought pose estimation, even if convergence did not occur.
 *
 *  In fact the user could also set an impossible to meet RMSE threshold (like 1e-5), and always take as pose estimation the surviving Candidate.
 *
 *  The algorithm summary is presented below:
 *  1. Align all the Candidates over Target for a few ICP steps.
 *  2. If one converges, Candidate is Pose Estimation and terminate.
 *  3. Sort composite list on minimum RMSE on top.
 *  4. Truncate the list.
 *  5. If list size is one, terminate, otherwise repeat from 1.
 *
 * \subsection apps Example Applications
 * Example applications can be optionally built and installed to provide some out-of-the-box functionality and as examples of library usage.
 * Their source code is located inside the "Example Apps" folder.
 *  \subsubsection app1 App1 TODO
 */
//////// End of Doxygen ////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PEL_COMMON_H_
#define PEL_COMMON_H_

#include <pel/pel_config.h>
#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <cmath>
#include <unordered_map>
#include <Eigen/Dense>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <utility>
#include <iostream>
#include <iterator>

///Current local time, seconds precision
#define TIME_NOW boost::posix_time::second_clock::local_time()

namespace pel
{
  ///Default Point Type used
  typedef pcl::PointXYZ Pt;
  ///Default Point Cloud used
  typedef pcl::PointCloud<Pt> PtC;
  ///FLANN matrix used to store histograms
  typedef flann::Matrix<float> histograms;
  ///FLANN Index for VFH
  typedef flann::Index<flann::ChiSquareDistance<float> > indexVFH;
  ///FLANN Index for ESF
  typedef flann::Index<flann::L2<float> > indexESF;
  ///Short writing of timestamps
  typedef boost::posix_time::ptime timestamp;
  ///Enumerator for list of candidates
  enum class ListType {vfh, esf, cvfh, ourcvfh, composite};
  /// Map that stores configuration parameters in a key=value fashion
  typedef std::unordered_map<std::string,float> parameters;

  /**\brief Compute the MinMax distance between two histograms, used by CVFH and OURCVFH
   * \param[in] a The first histogram
   * \param[in] b The second histogram
   * \param[in] size Size of vectors
   * \returns The computed distance _D_
   *
   * The distance _D_ is defined by the following metric:
   * \f[
   *  D = 1 - \frac{1+\sum_i^n{min\left(a_i,b_i\right)}}{1+\sum_i^n{max\left(a_i,b_i\right)}}
   * \f]
   * where n=308 for CVFH/OURCVFH histograms
   */
  float
  getMinMaxDistance (float* a, float* b, int size=308);

  /**\brief Check if passed path could contain a valid database
   * \param[in] db_path The path to check
   * \returns _True_ if valid, _False_ otherwise
   */
  bool
  isValidDatabasePath (boost::filesystem::path db_path);
}
#endif //PEL_COMMON_H_
