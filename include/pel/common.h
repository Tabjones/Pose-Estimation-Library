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

//Doxygen Main Page
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
| verbosity   | 1            | 0,1 or 2| Controls the verbosity of PEL (0) Silent behaviour, only errors are printed, (1) Normal behaviour, errors and warnings are printed, (2) Verbose behaviour, spams all kind of messages.|
| use_vfh     | 1            | 0 or 1 | (1) Use the Viewpoint Feature Histogram (VFH) in feature estimation of Target. (0) Or disable it.<sup>1</sup>|
| use_esf     | 1            | 0 or 1 | (1) Use the Ensemble of Shape Functions (ESF) in feature estimation of Target. (0) Or disable it.<sup>1</sup>|
| use_cvfh    |           1  | 0 or 1 | (1) Use Clustered Viewpoint Feature Histogram (CVFH) in feature estimation of Target. (0) Or disable it.<sup>1</sup>|
| use_ourcvfh |           1  | 0 or 1 | (1) Use Oriented, Unique and Repeatable CVFH  in feature estimation of Target. (0) Or disable it.<sup>1</sup>|
| downsamp    |  1           | 0 or 1 | (1) Downsample the Target point cloud with Voxel Grid after eventual outlier filter and MLS resampling. (0) Don't apply this filter.<sup>2</sup>|
| downsamp_leaf_size| 0.005  | >0     | Set leaf size of Voxel Grid downsampling, a value of _1_ means one meter. Only relevant if Voxel Grid is enabled.<sup>2</sup>|
| upsamp      | 0            | 0 or 1 | (1) Resample the Target point cloud with MLS at random uniform density, before the eventual downsampling with Voxel Grid. (0) Don't apply this filter.<sup>2</sup>|
| upsamp_poly_order| 2        | >1 | Set polynomial order of MLS to the value specified, for most objects  second order polynomial functions are more than enough to correctly approximate the object surface. Relevant only if upsamp is enabled.<sup>2</sup>|
| upsamp_point_density|  200 | >=1 | Set desired point density on MLS surface to the value specified,  higher values produce more points, thus increasing the upsampling ratio. Relevant only if upsamp and upsamp_poly_fit is enabled.<sup>2</sup>|
| upsamp_poly_fit| 1          | 0 or 1 | (1) Fit MLS surface on polynomial functions with order upsamp_poly_order. (0) Or don't use polynomial fitting. Relevant only if upsamp is enabled.<sup>2</sup>|
| upsamp_search_radius| 0.05 | >0 | Set search radius of MLS to the value specified, a value of _1_ means one meter. Relevant if upsamp is enabled.<sup>2</sup>|
| filter     | 0            | 0 or 1| (1) Filter Target point cloud with Statistical Outliers Removal, before the eventual upsampling and downsampling. (0) Or don't apply this filter.<sup>2</sup>|
| filter_mean_k | 50        | >0 | How many neighboring points to consider in the statistical distribution calculated by the filter, relevant if filter is enabled.<sup>2</sup>|
| filter_std_dev_mul_thresh | 3 | >0 | Multiplication factor to apply at Standard Deviation of the statistical distribution during filtering process (higher value, means less aggressive filter). Relevant only if filter is enabled.<sup>2</sup>|
| lists_size | 20           | >=1 | The size of generated lists of Candidates. Also the k-nearest neighbors to the Target retrieved from Database. Increasing this value may increase recognition rate at the cost of computational time.|
| normals_radius_search | 0.02 | >0 | Set radius that defines the neighborhood of each point during Normal Estimation, value of _1_ means one meter. If normals are not computed, i.e. only ESF is estimated, this parameter is ignored.<sup>2</sup>|
| cvfh_ang_thresh | 7.5     |>0 | Set maximum allowable deviation of the normals in degrees, in the region segmentation step of CVFH computation. The value recommended from relative paper is 7.5 degrees. Relevant only if use_cvfh is enabled.<sup>2</sup>|
| cvfh_curv_thresh  | 0.025 |>0 | Set maximum allowable disparity of curvatures during region segmentation step of CVFH estimation. The value recommended from relative paper is 0.025. Relevant only if use_cvfh is enabled.<sup>2</sup>|
| cvfh_clus_tol  | 0.01     | >0 | Euclidean clustering tolerance, during CVFH segmentation. Points distant more than this value from each other, will likely be grouped in different clusters. A value of _1_ means one meter. Relevant only if use_cvfh is enabled.<sup>2</sup>|
| cvfh_clus_min_points  | 50 | >=1 | Set minimum number of points a cluster should contain to be considered such, during CVFH clustering. Relevant only if use_cvfh is enabled.<sup>2</sup>|
| ourcvfh_ang_thresh  | 7.5 |>0 | Set maximum allowable deviation of normals, in the region segmentation step of OURCVFH computation. The value recommended from relative paper is 7.5 degrees. Relevant only if use_ourcvfh is enabled.<sup>2</sup>|
| ourcvfh_curv_thresh | 0.025 |>0| Set maximum allowable disparity of curvatures during region segmentation step of OURCVFH estimation. The value recommended from relative paper is 0.025. Relevant only if use_ourcvfh is enabled.<sup>2</sup>|
| ourcvfh_clus_tol | 0.01 | >0 | Euclidean clustering tolerance, during OURCVFH segmentation. Points distant more than this value from each other, will likely be grouped in different clusters. A value of _1_ means one meter. Relevant only if use_ourcvfh is enabled.<sup>2</sup>|
| ourcvfh_clus_min_points  | 50 | >=1 | Set minimum number of points a cluster should contain to be considered such, during OURCVFH clustering. Relevant only if use_ourcvfh is enabled.<sup>2</sup>|
| ourcvfh_axis_ratio  | 0.95 | >0 | Set the minimum axis ratio between the SGURF axes. At the disambiguation phase of OURCVFH, this will decide if additional Reference Frames need to be created for the cluster, if they are ambiguous. Relevant only if use_ourcvfh is enabled.<sup>2</sup>|
| ourcvfh_min_axis_value  | 0.01 | >0 | Set the minimum disambiguation axis value to generate several SGURFs for the cluster when disambiguition is difficult. Relevant if use_ourcvfh is enabled.<sup>2</sup>|
| ourcvfh_refine_clusters  |  1 | >=0, <=1 | Set refinement factor for clusters during OURCVFH clustering phase, a value of 1 means 'dont refine clusters', while values between 0 and 1 will reduce clusters size by that number. Relevant only if use_ourcvfh is enabled.<sup>2</sup>|

Notes:
  1. At least one feature must be enabled. Also if you disable this feature during Database creation (see pel::DatabaseCreator), resulting database will not have that feature, so all pose estimations using that database must also disable the feature.
  2. For optimal results, all parameters regarding filtering and feature estimation should be the same between Database and pose estimation Target.

 *
 * \subsection database Database
 *
 * \subsection refine Refinement
 *
 * \subsection apps Example Applications
 *
 */

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
