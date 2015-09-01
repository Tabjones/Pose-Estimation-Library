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

#ifndef PEL_PE_BRUTE_FORCE_H_
#define PEL_PE_BRUTE_FORCE_H_

#include <pel/pose_estimation_base.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>

namespace pel
{
  //TODO remove parameters for icp and add methods to control them here.
  /**\brief Brute Force explanation docs.
   */
  class PEBruteForce : public PoseEstimationBase
  {
    protected:
      pcl::IterativeClosestPoint<Pt, Pt, float> icp_;
      pcl::registration::TransformationEstimationDualQuaternion<Pt,Pt,float>::Ptr te_dq_;
      pcl::registration::TransformationEstimationLM<Pt,Pt,float>::Ptr te_lm_;
      pcl::registration::TransformationEstimationSVD<Pt,Pt,float>::Ptr te_svd_;
    public:
      PEBruteForce ();
      virtual ~PEBruteForce () {}
      /**\brief Get a Pose Estimation for the previously set target, using the previously set Database, with Brute Force method
       * \param[out] estimation Final Pose Estimation of the Target as a Candidate object
       */
      virtual void
      estimate (Candidate& estimation);
      /**\brief Tell ICP to use reciprocal correspondences or not
       * \param[in] setting Whenever to use them or not
       */
      virtual inline void
      setUseReciprocalCorrespondences (const bool setting = true)
      {
        icp_.setUseReciprocalCorrespondences(setting);
      }
      /**\brief Set Maximum ICP iterations to perform for each Candidate
       *\param[in] iterations Desired number of iterations
      */
      virtual inline void
      setMaxIterations (const unsigned int iterations = 100)
      {
        if (iterations > 0)
          icp_.setMaximumIterations(iterations);
      }
      /**\brief Set RMSE threshold for a Candidate to converge
       *\param[in] thresh The RMSE threshold to set
       */
      virtual inline void
      setRMSEThreshold (const float thresh = 0.005)
      {
        if (thresh > 0)
          icp_.setEuclideanFitnessEpsilon (std::pow(thresh,2));
      }
      /**\brief Set transformation estimation for ICP to Dual Quaternion method (default)
       */
      virtual inline void
      setUseDQ()
      {
        icp_.setTransformationEstimation(te_dq_);
      }
      /**\brief Set transformation estimation for ICP to Levenberg Marquardt method.
       * Default is to use Dual Quaternion Method
       */
      virtual inline void
      setUseLM()
      {
        icp_.setTransformationEstimation(te_lm_);
      }
      /**\brief Set transformation estimation for ICP to SVD-based method
       * Default is to use Dual Quaternion Method
       */
      virtual inline void
      setUseSVD()
      {
        icp_.setTransformationEstimation(te_svd_);
      }
  };
}
#endif
