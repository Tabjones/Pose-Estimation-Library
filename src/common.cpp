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

#include <pel/common.h>

namespace pel
{
  float
  MinMaxDistance (float* a, float* b, int size)
  {
    float num(1.0f), den(1.0f);
    //Process 4 items with each loop for efficency (since it should be applied to vectors of 308 elements)
    int i=0;
    for (; i<(size-3); i+=4)
    {
      num += std::min(a[i],b[i]) + std::min(a[i+1],b[i+1]) + std::min(a[i+2],b[i+2]) + std::min(a[i+3],b[i+3]);
      den += std::max(a[i],b[i]) + std::max(a[i+1],b[i+1]) + std::max(a[i+2],b[i+2]) + std::max(a[i+3],b[i+3]);
    }
    //process last 0-4 elements (if size!=308)
    while ( i < size)
    {
      num += std::min(a[i],b[i]);
      den += std::max(a[i],b[i]);
      ++i;
    }
    return (1 - (num/den));
  }

  bool
  isValidDatabase (boost::filesystem::path db_path)
  {
    if ( !boost::filesystem::exists(db_path) || !boost::filesystem::is_directory(db_path) )
      return false;
    boost::filesystem::path Pclouds(db_path.string() + "/Clouds");
    if ( !boost::filesystem::exists(Pclouds) || !boost::filesystem::is_directory(Pclouds) )
      return false;
    if ( !boost::filesystem::is_regular_file(db_path.string()+ "/vfh.h5") || !(boost::filesystem::extension(db_path.string()+ "/vfh.h5") == ".h5"))
      return false;
    if ( !boost::filesystem::is_regular_file(db_path.string()+ "/esf.h5") || !(boost::filesystem::extension(db_path.string()+ "/esf.h5") == ".h5"))
      return false;
    if ( !boost::filesystem::is_regular_file(db_path.string()+ "/cvfh.h5") || !(boost::filesystem::extension(db_path.string()+ "/cvfh.h5") == ".h5"))
      return false;
    if ( !boost::filesystem::is_regular_file(db_path.string()+ "/ourcvfh.h5") || !(boost::filesystem::extension(db_path.string()+ "/ourcvfh.h5") == ".h5"))
      return false;
    if ( !boost::filesystem::is_regular_file(db_path.string()+ "/vfh.idx") || !(boost::filesystem::extension(db_path.string()+ "/vfh.idx") == ".idx"))
      return false;
    if ( !boost::filesystem::is_regular_file(db_path.string()+ "/esf.idx") || !(boost::filesystem::extension(db_path.string()+ "/esf.idx") == ".idx"))
      return false;
    if ( !boost::filesystem::is_regular_file(db_path.string()+ "/names.list") || !(boost::filesystem::extension(db_path.string()+ "/names.list") == ".list"))
      return false;
    if ( !boost::filesystem::is_regular_file(db_path.string()+ "/names.cvfh") || !(boost::filesystem::extension(db_path.string()+ "/names.cvfh") == ".cvfh"))
      return false;
    if ( !boost::filesystem::is_regular_file(db_path.string()+ "/names.ourcvfh") || !(boost::filesystem::extension(db_path.string()+ "/names.ourcvfh") == ".ourcvfh"))
      return false;

    return true;
  }
}
