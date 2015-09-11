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

#ifndef PEL_POSE_ESTIMATION_BASE_H_
#define PEL_POSE_ESTIMATION_BASE_H_

#include <pel/database/database.h>
#include <pel/param_handler.h>
#include <pel/candidates/target.h>
#include <pel/candidates/candidate_list.h>
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
  /**\brief PoseEstimation base class. All exposed user interface inherits from this.
  \author Federico Spinelli
  */
  class PoseEstimationBase : public ParamHandler, public Database, public CandidateLists, public Target
  {
    public:
      PoseEstimationBase () : feature_count_(0)
      {
        target_cloud.reset(new PtC);
        target_cloud_processed.reset(new PtC);
      }
      virtual ~PoseEstimationBase () {}
    protected:
      ///Internal counter used to count how many feature the class uses
      int feature_count_;

      /**\brief Generate Lists of Candidates based on distance from target
       *\returns _True_ if succesful, _False_ otherwise
       */
      virtual bool
      generateLists();
      ///\brief Initialize a Target for PoseEstimation
      virtual bool
      initTarget ();
      ///\brief computeVFH feature of target
      virtual void
      computeVFH ();
      ///\brief computeESF feature of target
      virtual void
      computeESF ();
      ///\brief computeCVFH feature of target
      virtual void
      computeCVFH ();
      ///\brief computeOURCVFH feature of target
      virtual void
      computeOURCVFH ();
      ///\brief computeNormals features of target
      virtual void
      computeNormals ();
      ///\brief removeOutliers by applying Statistical Outliers Filter
      virtual void
      removeOutliers ();
      ///\brief applyDownsampling with VoxelGrid Filter
      virtual void
      applyDownsampling ();
      ///\brief applyUpsampling With MLS with Random uniform sampling
      virtual void
      applyUpsampling ();
      ///Estimate prototype
      virtual void
      estimate (Candidate& estimation)=0;
    public:
      ///\brief Set the target for next Pose Estimation
      ///\param[in] target Point cloud of target object
      ///\returns _True_ if succesful, _False_ otherwise
      virtual bool
      setTarget (PtC::Ptr target, std::string name="target");
      ///\brief Load a Database from disk and set it to be used for next Pose Estimation
      ///\param[in] db_path Path to directory containing Database to load
      ///return _True_ if succesful, _False_ otherwise
      virtual bool
      loadAndSetDatabase (boost::filesystem::path db_path);
      /**\brief Set a Database from another object already loaded in memory
       *\param[in] db Database to set for current Pose Estimation
       *\return _True_ if succesful, _False_ otherwise
       */
      virtual bool
      setDatabase (const Database& db);
      /**\brief Copy Assignemnt from Database to PoseEstimationBase
       *\param[in] other Database to copy from
       */
      PoseEstimationBase& operator= (const Database& other);
      /**\brief Move Assignemnt from Database to PoseEstimationBase
       * \param[in] other Database to move from
       */
      PoseEstimationBase& operator= (Database&& other);
  };
}
#endif //PEL_POSE_ESTIMATION_BASE_H_
