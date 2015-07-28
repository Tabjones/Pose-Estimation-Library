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

#include <pel/database/database_io.h>

namespace pel
{
  bool
  DatabaseReader::load (boost::filesystem::path path, Database& target)
  {
    if ( isValidDatabasePath(path) )
    {
      boost::filesystem::path Pclouds(path.string()+"/Clouds");
      std::vector<boost::filesystem::path> pvec;
      boost::copy (directory_iterator(Pclouds), directory_iterator(), back_inserter(pvec));
      if (pvec.size() <= 0)
      {
        print_error("%*s]\tNo files in Clouds directory of database, cannot load poses, aborting...\n",20,__func__);
        return false;
      }
      boost::sort (pvec.begin(), pvec.end());
      Database tmp;
      tmp.clouds_.resize(pvec.size());
      int i(0);
      for (std::vector<boost::filesystem::path>::const_iterator it(pvec.begin()); it != pvec.end(); ++it, ++i)
      {
        if (boost::filesystem::is_regular_file(*it) && boost::filesystem::extension(*it)==".pcd" )
        {
          if (pcl::io::loadPCDFile (it->string(),tmp.clouds_[i])!=0)
          {
            print_warn("%*s]\tError loading PCD file number %d, name %s, skipping...\n",20,__func__,i+1,it->string().c_str());
            continue;
          }
          Eigen::Vector3f s_orig (tmp.clouds_[i].sensor_origin_(0), tmp.clouds_[i].sensor_origin_(1), tmp.clouds_[i].sensor_origin_(2) ) ;
          Eigen::Quaternionf s_orie = tmp.clouds_[i].sensor_orientation_;
          if (tmp.clouds_[i].points.size() <= 0)
            print_warn("%*s]\tLoaded PCD file number %d, name %s has ZERO points!! Are you loading the correct files?\n",20,__func__,i+1,it->string().c_str());
        }
        else
        {
          print_warn("%*s]\t%s is not a PCD file, skipping...\n",20,__func__,it->string().c_str());
          continue;
        }
      }
      try
      {
        tmp.vfh_.reset(new histograms);
        flann::load_from_file (*(tmp.vfh_), path.string() + "/vfh.h5", "VFH Histograms");
      }
      catch (...)
      {
        print_error("%*s]\tError loading VFH histograms, file is likely corrupted, try recreating database...\n",20,__func__);
        return false;
      }
      try
      {
        tmp.esf_.reset(new histograms);
        flann::load_from_file (*(tmp.esf_), path.string() + "/esf.h5", "ESF Histograms");
      }
      catch (...)
      {
        print_error("%*s]\tError loading ESF histograms, file is likely corrupted, try recreating database...\n",20,__func__);
        return false;
      }
      try
      {
        tmp.cvfh_.reset(new histograms);
        flann::load_from_file (*(tmp.cvfh_), path.string() + "/cvfh.h5", "CVFH Histograms");
        cvfh_ = boost::make_shared<histograms>(m);
      }
      catch (...)
      {
        print_error("%*s]\tError loading CVFH histograms, file is likely corrupted, try recreating database...\n",20,__func__);
        return false;
      }
      try
      {
        tmp.ourcvfh_.reset(new histograms);
        flann::load_from_file (*(tmp.ourcvfh_), path.string() + "/ourcvfh.h5", "OURCVFH Histograms");
        ourcvfh_ = boost::make_shared<histograms>(m);
      }
      catch (...)
      {
        print_error("%*s]\tError loading OURCVFH histograms, file is likely corrupted, try recreating database...\n",20,__func__);
        return false;
      }
      try
      {
        tmp.vfh_idx_.reset(new IndexVFH(*(tmp.vfh_), SavedIndexParams(path.string()+"/vfh.idx")));
        tmp.vfh_idx_ ->buildIndex();
      }
      catch (...)
      {
        print_error("%*s]\tError loading VFH index, file is likely corrupted, try recreating database...\n",20,__func__);
        return false;
      }
      try
      {
        //TODO here
        indexESF idx (*esf_, SavedIndexParams(path.string()+"/esf.idx"));
        esf_idx_ = boost::make_shared<indexESF>(idx);
        esf_idx_ -> buildIndex();
      }
      catch (...)
      {
        print_error("%*s]\tError loading ESF index, file is likely corrupted, try recreating database...\n",20,__func__);
        return false;
      }
      try
      {
        ifstream file ((path.string()+"/names.list").c_str());
        std::string line;
        if (file.is_open())
        {
          while (getline (file, line))
          {
            trim(line); //remove white spaces from line
            names_.push_back(line);
          }//end of file
        }
        else
        {
          print_error("%*s]\tError opening names.list, file is likely corrupted, try recreating database\n",20,__func__);
          return false;
        }
      }
      catch (...)
      {
        print_error("%*s]\tError loading names.list, file is likely corrupted, try recreating database\n",20,__func__);
        return false;
      }
      try
      {
        ifstream file ((path.string()+"/names.cvfh").c_str());
        std::string line;
        if (file.is_open())
        {
          while (getline (file, line))
          {
            trim(line); //remove white spaces from line
            names_cvfh_.push_back(line);
          }//end of file
        }
        else
        {
          print_error("%*s]\tError opening names.cvfh, file is likely corrupted, try recreating database\n",20,__func__);
          return false;
        }
      }
      catch (...)
      {
        print_error("%*s]\tError loading names.cvfh, file is likely corrupted, try recreating database\n",20,__func__);
        return false;
      }
      try
      {
        ifstream file ((path.string()+"/names.ourcvfh").c_str());
        string line;
        if (file.is_open())
        {
          while (getline (file, line))
          {
            trim(line); //remove white spaces from line
            names_ourcvfh_.push_back(line);
          }//end of file
        }
        else
        {
          print_error("%*s]\tError opening names.ourcvfh, file is likely corrupted, try recreating database\n",20,__func__);
          return false;
        }
      }
      catch (...)
      {
        print_error("%*s]\tError loading names.ourcvfh, file is likely corrupted, try recreating database\n",20,__func__);
        return false;
      }
    }
    else
    {
      print_error("%*s]\t%s is not a valid database directory, or does not exist\n",20,__func__,path.string().c_str());
      return false;
    }
    dbPath_=path;
    return true;

  }

}//End of namespace
