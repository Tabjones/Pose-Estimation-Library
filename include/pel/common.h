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
 *  Pose Estimation Library is mainly developed within the <a href="http://www.pacman-project.eu/">PaCMan Project</a> as a tool to obtain fast and accurate
 *  Pose Estimations of known graspable objects. The work implemented here is derived from the author's master thesis in Robotics Engineering, which is available
 *  online, as a further reference, from University of Pisa <a href="https://etd.adm.unipi.it/theses/available/etd-09022014-142255/unrestricted/tesi.pdf">Online Thesis Archive</a>.
 *  Full source code and installation instructions are available on <a href="https://bitbucket.org/Tabjones/pose-estimation-library">Bitbucket</a>.
 *  \section theory Theoreticl Primer
 *  \section structure User Guide
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
