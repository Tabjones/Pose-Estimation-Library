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

#ifndef PEL_CANDIDATES_CANDIDATE_LIST_H_
#define PEL_CANDIDATES_CANDIDATE_LIST_H_

#include <pel/common.h>
#include <pel/candidates/candidate.h>
#include <vector>

namespace pel
{
  /**\brief Class CandidateLists implements various lists of Candidates, one for each descriptor plus the composite one
   * \note Lists are generated during the Pose Estimation procedure.
   */
  class CandidateLists
  {
    public:
      CandidateLists () {}
      virtual ~CandidateLists () {}

      /**\brief Return a list of choice
       *\param[in] type ListType enum to choose which list to return
       *\returns A copy of the chosen list or an empty one if it was not yet generated
      */
      std::vector<Candidate>
      getCandidateList (ListType type) const;
      /**\brief Print a chosen list of Candidates
       *\param[in] type ListType enum to chose which list to print
      */
      void
      printCandidateList (ListType type) const;
    protected:
      std::vector<Candidate> vfh_list, esf_list, cvfh_list, ourcvfh_list, composite_list;

      ///\brief Prototype function for list generation
      virtual bool
      generateLists() =0;

      /**\brief Sort lists based on Minimum RMSE
       * \param[in] type Which ListType to sort
       * \returns _True_ if sorting succeded, _False_ otherwise
       */
      bool
      sortListByRMSE (ListType type);
      /**\brief Sort lists based on Minimum Distance from target
       * \param[in] type Which ListType to sort
       * \returns _True_ if sorting succeded, _False_ otherwise
       */
      bool
      sortListByDistance (ListType type);
      /**\brief Sort lists based on Minimum Normalized Distance from target
       * \param[in] type Which ListType to sort
       * \returns _True_ if sorting succeded, _False_ otherwise
       */
      bool
      sortListByNormalizedDistance (ListType type);
      /**\brief Find a Candidate in the specified list by name, save its distance and remove it from the list
       * \param[in] list The List of Candidates to modify
       * \param[in] name The Candidate Name to find
       * \parma[out] dist Distance found
       * \returns _True_ if operation succeded, _False_ otherwise
       */
      bool
      findAndEraseCandidate(std::vector<Candidate>& list, std::string name, float dist);
  };
}
#endif //PEL_CANDIDATES_CANDIDATE_LIST_H_
