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

#ifndef PEL_CANDIDATE_H_
#define PEL_CANDIDATE_H_

#include <pel/common.h>
#include <pcl/common/io.h>

namespace pel
{
  /**\brief Class Candidate describes a potential candidate to a pose estimation
   */
  class Candidate
  {
    public:
      /**\brief Empty constructor */
      Candidate () : rank_ (0), name_(), distance_(-1), normalized_distance_(-1), rmse_(-1),
        transformation_(Eigen::Matrix4f::Identity ())
      {}
      /**\brief Constructor with name and cloud pointer
       * \param[in] str The Candidate name
       * \parma[in] clp Shared pointer to point cloud containing the candidate
      */
      Candidate (std::string str, PtC::Ptr clp) : rank_(0), name_(str), distance_(-1),
        normalized_distance_(-1), rmse_(-1), cloud_(clp)
      {}
      /**\brief Destructor */
      virtual ~Candidate () {}

      /**\brief assignment operator
       *\param[in] other Candidate to copy from
       */
      Candidate&
      operator= (const Candidate& other)
      {
        rank_ = other.rank_;
        distance_ = other.distance_;
        normalized_distance_ = other.normalized_distance_;
        rmse_ = other.rmse_;
        transformation_ = other.transformation_;
        name_ = other.name_;
        cloud_.reset(new PtC);
        pcl::copyPointCloud(*other.cloud_, *cloud_);
        return (*this);
      }
      /** \brief Get Candidate Rank from the list of candidates it belongs
       * \return The rank of Candidate (if any) in the list, otherwise returns 0
       *
       * \note A Candidate has a rank only after list(s) of Candidates are built by PoseEstimation
       */
      inline int
      getRank () const
      {
        return (rank_);
      }

      /** \brief Get the distance of Candidate from target point cloud in the metric chosen by the feature
       * \return The distance of candidate from target point cloud (if any), otherwise -1
       *
       * \note A Candidate has a distance only after list(s) of Candidates are built by PoseEstimation
       */
      inline float
      getDistance () const
      {
        return (distance_);
      }

      /** \brief Get the normalized distance of Candidate from target point cloud
       * \return The normalized distance of candidate from target point cloud (if any), otherwise -1
       *
       * \note Normalized distances range from 0 to 1, zero at "rank 1" and one at "rank k", they are indipendent of the
       * current metric chosen to calculate them, thus could be compared with other normalized distances from other
       * lists. A Candidate has a normalized distance only after list(s) of Candidates are built by generateLists
       */
      inline float
      getNormalizedDistance () const
      {
        return (normalized_distance_);
      }

      /** \brief Get Root Mean Square Error of Candidate from target point cloud
       * \return The Root Mean Square Error of the Candidate (if any), otherwise -1
       *
       * \noe A Candidate has an RMSE only after pose estimation process has been performed
       */
      inline float
      getRMSE () const
      {
        return (rmse_);
      }

      /** \brief Get Homogeneous Transformation that brings Candidate cloud over target cloud
       * \return The transformation that brings Candidate cloud over target cloud (if any), otherwise returns Identity matrix
       *
       * \note A Candidate has a transformation only after pose estimation process has been performed
       */
      inline Eigen::Matrix4f
      getTransformation () const
      {
        return (transformation_);
      }

      /** \brief Get a copy of the point cloud representing the Candidate
       * \return Point cloud of the candidate
       */
      inline PtC
      getCloud () const
      {
        return (*cloud_);
      }

      /** \brief Get Candidate name
       * \return The name of the Candidate
       */
      inline std::string
      getName() const
      {
        return (name_);
      }
      /**\brief Set Name of the Candidate
       *\param[in] name The name to set
       */
      inline void
      setName (std::string name)
      {
        name_ = name;
      }
      /**\brief Set Point Cloud containing candidate
       *\param[in] cloud Pointer to point cloud to set
        */
      inline void
      setCloud (const PtC::Ptr& cloud)
      {
        cloud_.reset(new PtC);
        pcl::copyPointCloud(*cloud, *cloud_);
      }
      /**\brief Set Rank of Candidate
       *\param[in] rank Rank to set
       */
      inline void
      setRank(int rank)
      {
        rank_ = rank;
      }
      /**\brief Set Distance of Candidate
       *\param[in] dist Distance to set
      */
      inline void
      setDistance (float dist)
      {
        distance_ = dist;
      }
      /**\brief Set Normalized Distance of Candidate
       *\param[in] norm_dist Normalized Distance to set
       */
      inline void
      setNormalizedDistance (float norm_dist)
      {
        normalized_distance_ = norm_dist;
      }
      /**\bief Set RMSE of Candidate
       *\param[in] rmse RMSE to set
        */
      inline void
      setRMSE (float rmse)
      {
        rmse_ = rmse;
      }
      /**\brief Set Transformation of Candidate
       *\param[in] trans Eigen Matrix, expressing the homogeneous transformation to set.
       */
      inline void
      setTransformation (const Eigen::Matrix4f& trans)
      {
        transformation_ = trans;
      }
      /// Avoid alignment errors with Eigen
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
      std::string name_;
      PtC::Ptr cloud_;
      int rank_;
      float distance_;
      float normalized_distance_;
      float rmse_;
      Eigen::Matrix4f transformation_;

  };//End of Class Candidate
} //End of namespace pel

#endif //PEL_CANDIDATE_H_
