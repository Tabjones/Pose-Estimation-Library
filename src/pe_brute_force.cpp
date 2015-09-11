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
 * * Neither the name of copyright holder(s) nor the names of its
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

#include <pel/pe_brute_force.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>

using namespace pcl::console;

namespace pel
{
  namespace interface
  {
    PEBruteForce::PEBruteForce ()
    {
      te_dq_.reset(new pcl::registration::TransformationEstimationDualQuaternion<Pt,Pt,float>);
      te_lm_.reset(new pcl::registration::TransformationEstimationLM<Pt,Pt,float>);
      te_svd_.reset(new pcl::registration::TransformationEstimationSVD<Pt,Pt,float>);
      icp_.setUseReciprocalCorrespondences(true);
      icp_.setMaximumIterations (100);
      icp_.setTransformationEpsilon (1e-9);
      icp_.setEuclideanFitnessEpsilon (std::pow(0.005,2));
      icp_.setTransformationEstimation(te_dq_);
      RMSE_thresh_ = 0.005;
    }

    void
    PEBruteForce::estimate(Candidate& estimation)
    {
      pcl::StopWatch timer;
      timer.reset();
      if (this->generateLists())
      {
        pcl::CentroidPoint<Pt> target_cen_est;
        for (const auto x : target_cloud_processed->points)
          target_cen_est.add(x);
        Pt target_centroid;
        target_cen_est.get(target_centroid);
        //BruteForce Procedure
        if (getParam("verbosity")>1)
          print_info("%*s]\tStarting Brute Force...\n",20,__func__);
        icp_.setInputTarget(target_cloud_processed);
        for (auto& x: composite_list)
        {
          PtC::Ptr aligned (new PtC);
          PtC::Ptr candidate (new PtC);
          pcl::copyPointCloud(x.getCloud(), *candidate);
          //icp align source over target, result in aligned
          candidate->sensor_origin_.setZero();
          candidate->sensor_orientation_.setIdentity();
          //candidate cloud we want to try aligning over the target
          icp_.setInputSource(candidate);
          Eigen::Matrix4f T_kli, T_cen, guess;
          pcl::CentroidPoint<Pt> cct;
          Pt candidate_centroid;
          Eigen::Matrix3f sensor_rotation( x.getCloud().sensor_orientation_ );
          Eigen::Vector4f sensor_translation;
          sensor_translation = x.getCloud().sensor_origin_;
          //Transformation from local object reference frame to kinect frame (as it was during database acquisition)
          T_kli << sensor_rotation(0,0), sensor_rotation(0,1), sensor_rotation(0,2), sensor_translation(0),
                sensor_rotation(1,0), sensor_rotation(1,1), sensor_rotation(1,2), sensor_translation(1),
                sensor_rotation(2,0), sensor_rotation(2,1), sensor_rotation(2,2), sensor_translation(2),
                0,                    0,                    0,                    1;
          PtC::Ptr cloud_in_k (new PtC);
          pcl::transformPointCloud(x.getCloud(), *cloud_in_k, T_kli);
          for (const auto x: cloud_in_k->points)
            cct.add(x);
          cct.get(candidate_centroid);
          T_cen << 1,0,0, (target_centroid.x - candidate_centroid.x),
                0,1,0, (target_centroid.y - candidate_centroid.y),
                0,0,1, (target_centroid.z - candidate_centroid.z),
                0,0,0, 1;
          //initial guess for ICP
          guess = T_cen*T_kli;
          icp_.align(*aligned, guess);
          x.setTransformation(icp_.getFinalTransformation());
          x.setRMSE(sqrt(icp_.getFitnessScore()));
          if (getParam("verbosity")>1)
          {
            print_info("%*s]\tCandidate: ",20,__func__);
            print_value("%-15s",x.getName().c_str());
            print_info(" just performed ICP alignment, its RMSE is: ");
            print_value("%g\n",x.getRMSE());
          }
          if (x.getRMSE() <= RMSE_thresh_)
          {
            //convergence we have a winner
            estimation = x;
            if (getParam("verbosity")>1)
            {
              print_info("%*s]\tCandidate %s converged with RMSE %g\n",20,__func__,estimation.getName().c_str(), estimation.getRMSE());
              print_info("%*s]\tFinal transformation is:\n",20,__func__);
              std::cout<<estimation.getTransformation()<<std::endl;
              print_info("%*s]\tTotal time elapsed for complete Pose Estimation: ",20,__func__);
              print_value("%g",timer.getTime());
              print_info(" ms\n");
            }
            return;
          }
        }
        //no candidate converged, pose estimation failed
        if (getParam("verbosity")>0)
          print_warn("%*s]\tCannot find a suitable candidate, try raising the RMSE threshold\n",20,__func__);
        if (getParam("verbosity")>1)
        {
          print_info("%*s]\tPose Estimation failed!",20,__func__);
          print_info("%*s]\tTotal time elapsed: ",20,__func__);
          print_value("%g",timer.getTime());
          print_info(" ms\n");
        }
      }
      //failed to generate lists
      print_error("%*s]\tFailed to generate lists of Candidates. Aborting pose estimation...",20,__func__);
    }
  }//end of namespace
}
