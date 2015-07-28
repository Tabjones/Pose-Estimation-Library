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
  /// Enumerator for list of candidates
  enum class listType {vfh, esf, cvfh, ourcvfh, composite};
  /** Map that stores configuration parameters in a key=value fashion*/
  typedef std::unordered_map<std::string,float> parameters;

  /**\addtogroup global Global Functions
   *
   * General utilities functions
   * @{ */
  /**\brief Compute the MinMax distance between two histograms, used by CVFH and OURCVFH
   * \param[in] a The first histogram
   * \param[in] b The second histogram
   * \param[in] size Size of vectors
   * \returns The computed dstance _D_
   *
   * The distance _D_ is defined by the following metric:
   * \f[
   *  D = 1 - \frac{1+\sum_i^n{min\left(a_i,b_i\right)}}{1+\sum_i^n{max\left(a_i,b_i\right)}}
   * \f]
   * where n=308 for CVFH/OURCVFH histograms
   */
  float
  getMinMaxDistance (float* a, float* b, int size);

  /**\brief Check if passed path could contain a valid database
   * \param[in] db_path The path to check
   * \returns _True_ if valid, _False_ otherwise
   * @}
   */
  bool
  isValidDatabasePath (boost::filesystem::path db_path);
}
#endif //PEL_COMMON_H_
