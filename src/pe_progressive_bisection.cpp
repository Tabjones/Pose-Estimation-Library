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

#include <pel/pe_progressive_bisection.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>

using namespace pcl::console;

namespace pel
{
  namespace interface
  {
    PEProgressiveBisection::PEProgressiveBisection ()
    {
      list_size_ = getParam("lists_size") ;
      te_dq_.reset(new pcl::registration::TransformationEstimationDualQuaternion<Pt,Pt,float>);
      te_lm_.reset(new pcl::registration::TransformationEstimationLM<Pt,Pt,float>);
      te_svd_.reset(new pcl::registration::TransformationEstimationSVD<Pt,Pt,float>);
      icp_.setUseReciprocalCorrespondences(true);
      icp_.setMaximumIterations (5);
      icp_.setTransformationEpsilon (1e-9);
      icp_.setEuclideanFitnessEpsilon (1e-9);
      icp_.setTransformationEstimation(te_dq_);
      success_on_size_one_ = true;
      bisection_fraction_ = 0.5;
      step_iterations_ = 5;
      RMSE_thresh_ = 0.005;
    }

    void
    PEProgressiveBisection::setBisectionFraction (const float fraction)
    {
      if (fraction <=0 )
        bisection_fraction_ = 0.01;
      else if (fraction >=1)
        bisection_fraction_ = 0.99;
      else
        bisection_fraction_ = fraction;
    }

    void
    PEProgressiveBisection::estimate (Candidate& estimation)
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
        //ProgressiveBisection
        if (getParam("verbosity")>1)
          print_info("%*s]\tStarting Progressive Bisection...\n",20,__func__);
        //make a temporary list to manipulate
        std::vector<Candidate> list = getCandidateList(ListType::composite);
        icp_.setInputTarget(target_cloud_processed); //Target
        int steps (0);
        while (list.size() > 1 )
        {
          for (auto& x: list)
          {
            PtC::Ptr aligned (new PtC);
            PtC::Ptr candidate (new PtC);
            pcl::copyPointCloud(x.getCloud(), *candidate);
            candidate->sensor_origin_.setZero();
            candidate->sensor_orientation_.setIdentity();
            //icp align source over target, result in aligned
            icp_.setInputSource(candidate); //the candidate
            Eigen::Matrix4f guess;
            if (steps >0)
              guess = x.getTransformation();
            else
            {
              Eigen::Matrix4f T_kli, T_cen;
              pcl::CentroidPoint<Pt> cct;
              Pt candidate_centroid;
              Eigen::Matrix3f R (x.getCloud().sensor_orientation_);
              Eigen::Vector4f t (x.getCloud().sensor_origin_);
              T_kli << R(0,0), R(0,1), R(0,2), t(0),
                    R(1,0), R(1,1), R(1,2), t(1),
                    R(2,0), R(2,1), R(2,2), t(2),
                    0,      0,      0,      1;
              PtC::Ptr cloud_in_k (new PtC);
              pcl::transformPointCloud(*candidate, *cloud_in_k, T_kli);
              for (const auto x: cloud_in_k->points)
                cct.add(x);
              cct.get(candidate_centroid);
              T_cen << 1,0,0, (target_centroid.x - candidate_centroid.x),
                    0,1,0, (target_centroid.y - candidate_centroid.y),
                    0,0,1, (target_centroid.z - candidate_centroid.z),
                    0,0,0, 1;
              guess = T_cen*T_kli;
            }
            icp_.align(*aligned, guess); //initial gross estimation
            x.setTransformation(icp_.getFinalTransformation());
            x.setRMSE(sqrt(icp_.getFitnessScore()));
            if (getParam("verbosity")>1)
            {
              print_info("%*s]\tCandidate: ",20,__func__);
              print_value("%-15s",x.getName().c_str());
              print_info(" just performed %d ICP iterations, its RMSE is: ", step_iterations_);
              print_value("%g\n", x.getRMSE());
            }
          }
          ++steps;
          //now resort list
          if (sortListByRMSE(list))
          {
            //check if candidate fell under rmse threshold, no need to check them all since list is now sorted with min rmse on top
            if (list[0].getRMSE() <= RMSE_thresh_ )
            {
              //convergence
              estimation = list[0];
              estimation.setRank(1);
              if (getParam("verbosity")>1)
              {
                print_info("%*s]\tCandidate %s converged with RMSE %g\n",20,__func__,list[0].getName().c_str(), list[0].getRMSE());
                print_info("%*s]\tFinal transformation is:\n",20,__func__);
                std::cout<<list[0].getTransformation()<<std::endl;
                print_info("%*s]\tTotal time elapsed for complete Pose Estimation: ",20,__func__);
                print_value("%g",timer.getTime());
                print_info(" ms\n");
              }
              return;
            }
            else
            {
              //no convergence, resize list
              int size = std::floor(list.size() * bisection_fraction_);
              if (size < list.size())
              {
                list.resize(size);
                if (getParam("verbosity")>1)
                  print_info("%*s]\tResizing composite list... Keeping %.2g%% of list at previous step\n",20,__func__,bisection_fraction_*100);
              }
              else
                break; //break while loop
            }
          }
          else
          {
            print_error("%*s]\tFailed to resort composite list. Aborting...",20,__func__);
            return;
          }
        }
        //only one candidate remained
        if (success_on_size_one_)
        {
          estimation = list[0];
          estimation.setRank(1);
          if (getParam("verbosity")>1)
          {
            print_info("%*s]\tCandidate %s survived progressive bisection with RMSE %g\n",20,__func__,estimation.getName().c_str(), estimation.getRMSE());
            print_info("%*s]\tFinal transformation is:\n",20,__func__);
            cout<<estimation.getTransformation()<<endl;
            print_info("%*s]\tTotal time elapsed for complete Pose Estimation: ",20,__func__);
            print_value("%g",timer.getTime());
            print_info(" ms\n");
          }
        }
        else
        {
          //User requested failure on list size 1
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
        return;
      }
      //Failed to generate lists
      print_error("%*s]\tFailed to generate lists of Candidates. Aborting pose estimation...",20,__func__);
    }
  } //End of namespace
}
