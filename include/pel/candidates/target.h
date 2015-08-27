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

#ifndef PEL_TARGET_H_
#define PEL_TARGET_H_

#include <pel/common.h>

namespace pel
{
  /**\brief Class Target describes an object to be estimated
   */
  class Target
  {
    public:
      /**\brief Empty constructor */
      Target () {}
      /**\brief Destructor */
      virtual ~Target () {}
    protected:
      ///computeVFH prototype
      virtual void
      computeVFH()=0;
      ///computeESF prototype
      virtual void
      computeESF()=0;
      ///computeCVFH prototype
      virtual void
      computeCVFH()=0;
      ///computeOURCVFH prototype
      virtual void
      computeOURCVFH()=0;
      ///computeNormals prototype
      virtual void
      computeNormals()=0;
      ///removeOutliers prototype
      virtual void
      removeOutliers()=0;
      ///applyDownsampling prototype
      virtual void
      applyDownsampling()=0;
      ///applyUpsampling prototype
      virtual void
      applyUpsampling()=0;

      std::string target_name;
      PtC::Ptr target_cloud;
      PtC::Ptr target_cloud_processed;
      ///Container that holds the target VFH feature
      pcl::PointCloud<pcl::VFHSignature308> target_vfh;
      ///Container that holds the target CVFH feature
      pcl::PointCloud<pcl::VFHSignature308> target_cvfh;
      ///Container that holds the target OURCVFH feature
      pcl::PointCloud<pcl::VFHSignature308> target_ourcvfh;
      ///Container that holds the target ESF feature
      pcl::PointCloud<pcl::ESFSignature640> target_esf;
      ///Container that holds the target normals
      pcl::PointCloud<pcl::Normal> target_normals;

  };//End of Class Target
} //End of namespace pel

#endif //PEL_TARGET_H_
