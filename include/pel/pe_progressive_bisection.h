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

#ifndef PEL_PE_PROGRESSIVE_BISECTION_H_
#define PEL_PE_PROGRESSIVE_BISECTION_H_

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
  namespace interface
  {
    /**\brief Implements Pose Estimation procedure with Progressive Bisection ICP refinement.
     *
     * This class is used to achieve pose estimation of a Target point cloud, using a loaded Database.
     * Progressive Bisection method generates a list of Candidate(s), selecting them from Database,
     * according to various global descriptors (VFH, ESF, ...) combined togheter in a sort consensus.
     * Then it aligns all the Candidates with a few steps of ICP. After all Candidates have undergone this partial
     * alignment, the list is reordered according to minimum RMSE and truncated in half. Then another few steps
     * of ICP takes place with remaining Candidates, list resorted and truncated again, and so on...
     * This goes on until one Candidate RMSE falls below the user set threshold or the list has only one Candidate remaining.
     * This procedure progressively discards the worst Candidates, which are at the bottom of the list
     * after sorting, and concentrates processing resources on "good" Candidates.
     * \note This procedure is tipically faster than BruteForce and always converges at the best possible
     * Candidate we could find in Database, thus it is generally a better choice over BruteForce.
     *
     * Typical usage:
     * \code
     * #include <pel/pe_progressive_bisection.h>
     * //... Load a point cloud from disk, containing a view of an object to be identified. Store it in obj_cloud pointer.
     * pel::interface::PEBruteForce estimator;
     * estimator.loadAndSetDatabase("path_to_db_on_disk"); //Load a Database and use it for Pose Estimation
     * estimator.setTarget(obj_cloud, "my target"); //Set my newly loaded target
     * //Optionally adjust Brute Force behaviour
     * estimator.setMaxIterations(300); //How many steps of ICP for each Candidate
     * estimator.setRMSEThreshold(0.001); //Convergence criteria for ICP: Candidate must align over target with no more than 1mm of error.
     * estimator.setUseLM(); //Use Levenberg Marquardt method to estimate a transformation during ICP steps.
     * pel::Candidate final_pose_estimation;
     * estimator.estimate (final_pose_estimation); //Perform Pose Estimation and put winner Candidate into final_pose_estimation.
     * //... We can check its RMSE and Transformation
     * std::cout<<"Winner is: "<<final_pose_estimation.getName().c_str()<<std::endl;
     * std::cout<<"RMSE: "<<final_pose_estimation.getRMSE()<<std::endl;
     * std::cout<<"Rigid Transformation:"<<std::endl;
     * std::cout<<final_pose_estimation.getTransformation();
     * \endcode
     */
    class PEProgressiveBisection : public PoseEstimationBase
    {
      protected:
        bool success_on_size_one_;
        int list_size_, step_iterations_;
        float bisection_fraction_;
        float RMSE_thresh_;
        pcl::IterativeClosestPoint<Pt, Pt, float> icp_;
        pcl::registration::TransformationEstimationDualQuaternion<Pt,Pt,float>::Ptr te_dq_;
        pcl::registration::TransformationEstimationLM<Pt,Pt,float>::Ptr te_lm_;
        pcl::registration::TransformationEstimationSVD<Pt,Pt,float>::Ptr te_svd_;
      public:
        PEProgressiveBisection ();
        virtual ~PEProgressiveBisection () {}
        /**\brief Get a Pose Estimation for the previously set target, using the previously set Database, with Progressive Bisection method.
         * \param[out] estimation Final Pose Estimation of the Target as a Candidate object
         */
        virtual void
        estimate (Candidate& estimation);
        /**\brief Tell ICP to use reciprocal correspondences or not.
         * \param[in] setting Whenever to use them or not
         */
        virtual inline void
        setUseReciprocalCorrespondences (const bool setting = true)
        {
          icp_.setUseReciprocalCorrespondences(setting);
        }
        /**\brief Set how many ICP iterations to perform for each Candidate on each phase of Progressive Bisection, i.e. before sorting and truncating the list.
         *\param[in] iterations Desired number of iterations

         \note Each Candidate will perform at least this number of ICP iterations.
         */
        virtual inline void
        setStepIterations (const unsigned int iterations = 5)
        {
          if (iterations > 0)
          {
            step_iterations_ = iterations;
            icp_.setMaximumIterations(iterations);
          }
        }
        /**\brief Set RMSE threshold for a Candidate to converge.
         *\param[in] thresh The RMSE threshold to set
         *
         * \note First termination criteria of the procedure.
         */
        virtual inline void
        setRMSEThreshold (const float thresh = 0.005)
        {
          if (thresh > 0)
            RMSE_thresh_ = thresh;
        }
        /**\brief Set transformation estimation for ICP to Dual Quaternion method (default).
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
        /**\brief Set transformation estimation for ICP to SVD-based method.
         * Default is to use Dual Quaternion Method
         */
        virtual inline void
        setUseSVD()
        {
          icp_.setTransformationEstimation(te_svd_);
        }
        /**\brief Set how much of the list is kept during bisection
         *\param[in] fraction Fraction of the list to keep on each bisection step.

         \note fraction gets thresholded between 0 (keep 0% of list) and 1 (keep 100% of list), limits excluded.
         \note default value is 0.5: keep exactly half the list. Please note that by increasing this fraction you will
         inevitably increase computation time.
        */
        virtual void
        setBisectionFraction(const float fraction = 0.5);
        /**\brief Set if the procedure should succesfully terminate when composite list size reduces to one, or if it should fail.
         * \param[in] success Tell the procedure to consider list size = 1 a success, or not.
         *
         * Since composite list gets reduced on each step, it could happen that no Candidate converges
         * under the user set RMSE threshold, before list size is inevitably reduced to one. Treat this
         * situation as a success, i.e. take the last remaining Candidate as the final Pose Estimation; or don't
         * and fail to deliver a Pose Estimation.
         */
        virtual inline void
        setConsiderSuccessOnListSizeOne(const bool success = true)
        {
          success_on_size_one_ = success;
        }
    };
  }
}

#endif
