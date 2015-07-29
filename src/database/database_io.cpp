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
#include <pel/database/database.h>

using namespace pcl::console;

namespace pel
{
  bool
  DatabaseReader::load (boost::filesystem::path path, Database& target)
  {
    if ( isValidDatabasePath(path) )
    {
      boost::filesystem::path Pclouds(path.string()+"/Clouds");
      std::vector<boost::filesystem::path> pvec;
      copy (boost::filesystem::directory_iterator(Pclouds), boost::filesystem::directory_iterator(), back_inserter(pvec));
      if (pvec.size() <= 0)
      {
        print_error("%*s]\tNo files in Clouds directory of database, cannot load poses, aborting...\n",20,__func__);
        return false;
      }
      sort (pvec.begin(), pvec.end());
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
      }
      catch (...)
      {
        print_error("%*s]\tError loading OURCVFH histograms, file is likely corrupted, try recreating database...\n",20,__func__);
        return false;
      }
      try
      {
        tmp.vfh_idx_.reset(new indexVFH(*(tmp.vfh_), SavedIndexParams(path.string()+"/vfh.idx")));
        tmp.vfh_idx_ ->buildIndex();
      }
      catch (...)
      {
        print_error("%*s]\tError loading VFH index, file is likely corrupted, try recreating database...\n",20,__func__);
        return false;
      }
      try
      {
        tmp.esf_idx_.reset(new indexESF(*(tmp.esf_), SavedIndexParams(path.string()+"/esf.idx")));
        tmp.esf_idx_ -> buildIndex();
      }
      catch (...)
      {
        print_error("%*s]\tError loading ESF index, file is likely corrupted, try recreating database...\n",20,__func__);
        return false;
      }
      try
      {
        std::ifstream file ((path.string()+"/names.list").c_str());
        std::string line;
        if (file.is_open())
        {
          while (getline (file, line))
          {
            boost::trim(line); //remove white spaces from line
            tmp.names_.push_back(line);
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
        std::ifstream file ((path.string()+"/names.cvfh").c_str());
        std::string line;
        if (file.is_open())
        {
          while (getline (file, line))
          {
            boost::trim(line); //remove white spaces from line
            tmp.names_cvfh_.push_back(line);
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
        std::ifstream file ((path.string()+"/names.ourcvfh").c_str());
        std::string line;
        if (file.is_open())
        {
          while (getline (file, line))
          {
            boost::trim(line); //remove white spaces from line
            tmp.names_ourcvfh_.push_back(line);
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
      tmp.db_path_ = path;
      this->last_loaded_ = path;
      target = tmp;
      return true;
    }
    else
    {
      print_error("%*s]\t%s is not a valid database directory, or does not exist\n",20,__func__,path.string().c_str());
      return false;
    }
  }

  Database
  DatabaseReader::load (boost::filesystem::path path)
  {
    Database ret;
    load(path, ret);
    return ret;
  }

  bool
  DatabaseReader::reload (Database& target)
  {
    return load(last_loaded_, target);
  }

  Database
  DatabaseReader::reload ()
  {
    return load(last_loaded_);
  }

  bool
  DatabaseWriter::save (boost::filesystem::path path, const Database& db, bool overwrite)
  {
    if (db.isEmpty())
    {
      print_warn("%*s]\tPassed Database is invalid or empty, not saving it...\n",20,__func__);
      return false;
    }
    if ( (!boost::filesystem::exists (path) && !boost::filesystem::is_directory(path)) ||
        (boost::filesystem::exists (path) && boost::filesystem::is_regular_file (path)) )
    {
      boost::filesystem::create_directories(path.string() + "/Clouds/");
      print_info("%*s]\tCreated directories to contain database in %s\n",20,__func__,path.string().c_str());
    }
    else
    {
      if (isValidDatabasePath(path))
      {
        if (overwrite)
          print_warn("%*s]\t%s already exists and contains a valid database, overwriting it as requested.\n",20,__func__,path.string().c_str());
        else
        {
          print_error("%*s]\t%s already exists and contains a valid database, not overwriting it as requested. aborting...\n",20,__func__,path.string().c_str());
          return false;
        }
      }
      else
      {
        if (overwrite)
          print_warn("%*s]\t%s already exists, but does not look like it contains a valid database, overwriting it as requested.\n",20,__func__,path.string().c_str());
        else
        {
          print_error("%*s]\t%s already exists, but does not look like it contains a valid database, not overwriting it as requested. aborting...\n",20,__func__,path.string().c_str());
          return false;
        }
      }
      if (overwrite)
      {
        if (boost::filesystem::exists(path.string() + "/names.list") && boost::filesystem::is_regular_file(path.string()+ "/names.list"))
          boost::filesystem::remove (path.string()+ "/names.list");
        if (boost::filesystem::exists(path.string() + "/names.cvfh") && boost::filesystem::is_regular_file(path.string()+ "/names.cvfh"))
          boost::filesystem::remove (path.string()+ "/names.cvfh");
        if (boost::filesystem::exists(path.string() + "/names.ourcvfh") && boost::filesystem::is_regular_file(path.string()+ "/names.ourcvfh"))
          boost::filesystem::remove (path.string()+ "/names.ourcvfh");
        if (boost::filesystem::exists(path.string() + "/Clouds") && boost::filesystem::is_directory(path.string()+ "/Clouds"))
        {
          boost::filesystem::remove_all(path.string() + "/Clouds");
          boost::filesystem::create_directories(path.string() + "/Clouds/");
        }
        if (boost::filesystem::exists(path.string() + "/vfh.h5") && boost::filesystem::is_regular_file(path.string()+ "/vfh.h5"))
          boost::filesystem::remove (path.string()+ "/vfh.h5");
        if (boost::filesystem::exists(path.string() + "/esf.h5") && boost::filesystem::is_regular_file(path.string()+ "/esf.h5"))
          boost::filesystem::remove (path.string()+ "/esf.h5");
        if (boost::filesystem::exists(path.string() + "/cvfh.h5") && boost::filesystem::is_regular_file(path.string()+ "/cvfh.h5"))
          boost::filesystem::remove (path.string()+ "/cvfh.h5");
        if (boost::filesystem::exists(path.string() + "/ourcvfh.h5") && boost::filesystem::is_regular_file(path.string()+ "/ourcvfh.h5"))
          boost::filesystem::remove (path.string()+ "/ourcvfh.h5");
        if (boost::filesystem::exists(path.string() + "/vfh.idx") && boost::filesystem::is_regular_file(path.string()+ "/vfh.idx"))
          boost::filesystem::remove (path.string()+ "/vfh.idx");
        if (boost::filesystem::exists(path.string() + "/esf.idx") && boost::filesystem::is_regular_file(path.string()+ "/esf.idx"))
          boost::filesystem::remove (path.string()+ "/esf.idx");
        if (boost::filesystem::exists(path.string() + "/created.info") && boost::filesystem::is_regular_file(path.string()+ "/created.info"))
          boost::filesystem::remove (path.string()+ "/created.info");
      }
    }
    pcl::PCDWriter writer;
    std::ofstream names, c_cvfh, c_ourcvfh, info;
    names.open((path.string()+ "/names.list").c_str());
    c_cvfh.open((path.string()+ "/names.cvfh").c_str());
    c_ourcvfh.open((path.string()+ "/names.ourcvfh").c_str());
    for (size_t i=0; i< db.names_.size(); ++i)
    {
      try
      {
        writer.writeBinaryCompressed(path.string() + "/Clouds/" + db.names_[i] + ".pcd", db.clouds_[i]);
        names << db.names_[i] <<std::endl;
        c_cvfh << db.names_cvfh_[i] <<std::endl;
        c_ourcvfh << db.names_ourcvfh_[i] <<std::endl;
      }
      catch (...)
      {
        print_error("%*s]\tError writing to disk, aborting...\n",20,__func__);
        return false;
      }
    }
    names.close();
    c_cvfh.close();
    c_ourcvfh.close();
    flann::save_to_file (*(db.vfh_), path.string() + "/vfh.h5", "VFH Histograms");
    flann::save_to_file (*(db.esf_), path.string() + "/esf.h5", "ESF Histograms");
    flann::save_to_file (*(db.cvfh_), path.string() + "/cvfh.h5", "CVFH Histograms");
    flann::save_to_file (*(db.ourcvfh_), path.string() + "/ourcvfh.h5", "OURCVFH Histograms");
    db.vfh_idx_->save ( path.string() + "/vfh.idx");
    db.esf_idx_->save ( path.string() + "/esf.idx");
    info.open( (path.string() + "/created.info").c_str() );
    timestamp t(TIME_NOW);
    info << "Database created on "<<to_simple_string(t).c_str()<<std::endl;
    info << "Contains "<<db.names_.size()<<" poses.";
    info << "Saved on path "<<path.string().c_str()<<" by DatabaseWriter::save"<<std::endl;
    print_info("%*s]\tDone saving database, %d poses written to disk\n",20,__func__,db.names_.size());
    return true;
  }
}//End of namespace
