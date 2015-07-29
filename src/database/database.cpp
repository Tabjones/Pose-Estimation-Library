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

#include <pel/database/database.h>
#include <boost/make_shared.hpp>

using namespace pcl::console;

namespace pel
{
  bool
  Database::isEmpty () const
  {
    if ( !(vfh_) || !(esf_) || !(cvfh_) || !(ourcvfh_) )
      return true;
    else if (names_.empty() || names_cvfh_.empty() || names_ourcvfh_.empty() || clouds_.empty() )
      return true;
    else if ( !(vfh_idx_) || !(esf_idx_) )
      return true;
    else
      return false;
  }

  Database::Database (const Database& other)
  {
    //Make a local copy of other, so that this is exception safe
    histograms vfh (new float[other.vfh_->rows * other.vfh_->cols], other.vfh_->rows, other.vfh_->cols);
    for (size_t i=0; i<other.vfh_->rows; ++i)
      for (size_t j=0; j<other.vfh_->cols; ++j)
        vfh[i][j] = (*other.vfh_)[i][j];
    histograms esf (new float[other.esf_->rows * other.esf_->cols], other.esf_->rows, other.esf_->cols);
    for (size_t i=0; i<other.esf_->rows; ++i)
      for (size_t j=0; j<other.esf_->cols; ++j)
        esf[i][j] = (*other.esf_)[i][j];
    histograms cvfh (new float[other.cvfh_->rows * other.cvfh_->cols], other.cvfh_->rows, other.cvfh_->cols);
    for (size_t i=0; i<other.cvfh_->rows; ++i)
      for (size_t j=0; j<other.cvfh_->cols; ++j)
        cvfh[i][j] = (*other.cvfh_)[i][j];
    histograms ourcvfh (new float[other.ourcvfh_->rows * other.ourcvfh_->cols], other.ourcvfh_->rows, other.ourcvfh_->cols);
    for (size_t i=0; i<other.ourcvfh_->rows; ++i)
      for (size_t j=0; j<other.ourcvfh_->cols; ++j)
        ourcvfh[i][j] = (*other.ourcvfh_)[i][j];
    std::vector<std::string> names;
    boost::copy (other.names_, back_inserter(names));
    std::vector<std::string> names_cvfh;
    boost::copy (other.names_cvfh_, back_inserter(names_cvfh));
    std::vector<std::string> names_ourcvfh;
    boost::copy (other.names_ourcvfh_, back_inserter(names_ourcvfh));
    std::vector<PtC> clouds;
    boost::copy (other.clouds_, back_inserter(clouds));
    //only way to copy FLANN indexs that i'm aware of (save it to disk then load it)
    other.vfh_idx_->save(".idx_v_tmp");
    indexVFH idx_vfh (vfh, SavedIndexParams(".idx_v_tmp"));
    other.esf_idx_->save(".idx_e_tmp");
    indexESF idx_esf (esf, SavedIndexParams(".idx_e_tmp"));
    //finally save the local copy into this
    db_path_ = other.db_path_;
    vfh_ = boost::make_shared<histograms>(vfh);
    esf_ = boost::make_shared<histograms>(esf);
    cvfh_ = boost::make_shared<histograms>(cvfh);
    ourcvfh_ = boost::make_shared<histograms>(ourcvfh);
    vfh_idx_ = boost::make_shared<indexVFH>(idx_vfh);
    vfh_idx_ -> buildIndex();
    boost::filesystem::remove(".idx_v_tmp");
    esf_idx_ = boost::make_shared<indexESF>(idx_esf);
    esf_idx_ -> buildIndex();
    boost::filesystem::remove(".idx_e_tmp");
    names_.clear();
    names_cvfh_.clear();
    names_ourcvfh_.clear();
    clouds_.clear();
    boost::copy (names, back_inserter(names_));
    boost::copy (names_ourcvfh, back_inserter(names_ourcvfh_));
    boost::copy (clouds, back_inserter(clouds_));
    boost::copy (names_cvfh, back_inserter(names_cvfh_));
  }

  Database::Database (Database&& other): vfh_(std::move(other.vfh_)), esf_(std::move(other.esf_)),
      cvfh_(std::move(other.cvfh_)), ourcvfh_(std::move(other.ourcvfh_)), db_path_(std::move(other.db_path_)),
      vfh_idx_(std::move(other.vfh_idx_)), esf_idx_(std::move(other.esf_idx_))
  {
    names_.swap(other.names_);
    names_cvfh_.swap(other.names_cvfh_);
    names_ourcvfh_.swap(other.names_ourcvfh_);
    clouds_.swap(other.clouds_);
  }

  Database&
  Database::operator= (const Database& other)
  {
    histograms vfh (new float[other.vfh_->rows * other.vfh_->cols], other.vfh_->rows, other.vfh_->cols);
    for (size_t i=0; i<other.vfh_->rows; ++i)
      for (size_t j=0; j<other.vfh_->cols; ++j)
        vfh[i][j] = (*other.vfh_)[i][j];
    histograms esf (new float[other.esf_->rows * other.esf_->cols], other.esf_->rows, other.esf_->cols);
    for (size_t i=0; i<other.esf_->rows; ++i)
      for (size_t j=0; j<other.esf_->cols; ++j)
        esf[i][j] = (*other.esf_)[i][j];
    histograms cvfh (new float[other.cvfh_->rows * other.cvfh_->cols], other.cvfh_->rows, other.cvfh_->cols);
    for (size_t i=0; i<other.cvfh_->rows; ++i)
      for (size_t j=0; j<other.cvfh_->cols; ++j)
        cvfh[i][j] = (*other.cvfh_)[i][j];
    histograms ourcvfh (new float[other.ourcvfh_->rows * other.ourcvfh_->cols], other.ourcvfh_->rows, other.ourcvfh_->cols);
    for (size_t i=0; i<other.ourcvfh_->rows; ++i)
      for (size_t j=0; j<other.ourcvfh_->cols; ++j)
        ourcvfh[i][j] = (*other.ourcvfh_)[i][j];
    std::vector<std::string> names;
    boost::copy (other.names_, back_inserter(names));
    std::vector<std::string> names_cvfh;
    boost::copy (other.names_cvfh_, back_inserter(names_cvfh));
    std::vector<std::string> names_ourcvfh;
    boost::copy (other.names_ourcvfh_, back_inserter(names_ourcvfh));
    std::vector<PtC> clouds;
    boost::copy (other.clouds_, back_inserter(clouds));
    //only way to copy FLANN indexs that i'm aware of (save it to disk then load it)
    other.vfh_idx_->save(".idx_v_tmp");
    indexVFH idx_vfh (vfh, SavedIndexParams(".idx_v_tmp"));
    boost::filesystem::remove(".idx_v_tmp"); //delete tmp file
    //only way to copy indexs that i'm aware of (save it to disk then load it)
    other.esf_idx_->save(".idx_e_tmp");
    indexESF idx_esf (esf, SavedIndexParams(".idx_e_tmp"));
    boost::filesystem::remove(".idx_e_tmp"); //delete tmp file
    //save tmp db into this
    this->vfh_ = boost::make_shared<histograms>(vfh);
    this->esf_ = boost::make_shared<histograms>(esf);
    this->cvfh_ = boost::make_shared<histograms>(cvfh);
    this->ourcvfh_ = boost::make_shared<histograms>(ourcvfh);
    this->vfh_idx_ = boost::make_shared<indexVFH>(idx_vfh);
    this->vfh_idx_ -> buildIndex();
    this->esf_idx_ = boost::make_shared<indexESF>(idx_esf);
    this->esf_idx_ -> buildIndex();
    this->names_.clear();
    this->names_cvfh_.clear();
    this->names_ourcvfh_.clear();
    this->clouds_.clear();
    boost::copy (names, back_inserter(this->names_));
    boost::copy (names_ourcvfh, back_inserter(this->names_ourcvfh_));
    boost::copy (clouds, back_inserter(this->clouds_));
    boost::copy (names_cvfh, back_inserter(this->names_cvfh_));
    this->db_path_ = other.db_path_;
    return *this;
  }

  Database&
  Database::operator= (Database&& other)
  {
    this->vfh_ = std::move(other.vfh_);
    this->esf_ = std::move(other.esf_);
    this->cvfh_ = std::move(other.cvfh_);
    this->ourcvfh_ = std::move(other.ourcvfh_);
    this->names_ = std::move(other.names_);
    this->names_cvfh_ = std::move(other.names_cvfh_);
    this->names_ourcvfh_ = std::move(other.names_ourcvfh_);
    this->db_path_= std::move(other.db_path_);
    this->clouds_ = std::move(other.clouds_);
    this->vfh_idx_ = std::move(other.vfh_idx_);
    this->esf_idx_ = std::move(other.esf_idx_);
    return *this;
  }

  bool
  Database::computeDistFromClusters (pcl::PointCloud<pcl::VFHSignature308>::Ptr target, listType feat,
      std::vector<std::pair<float, int> >& distIdx)
  {
    if (this->isEmpty())
    {
      print_error("%*s]\tDatabase is empty, cannot continue.\n",20,__func__);
      return false;
    }
    if (target->empty())
    {
      print_error("%*s]\tTarget histogram is empty, cannot continue.\n",20,__func__);
      return false;
    }
    distIdx.clear();
    if ( feat == listType::cvfh )
    { //cvfh list
      for (int n=0; n<target->points.size(); ++n)
      {//for each target cluster
        float d; //tmp distance
        int s=0; //counts objects
        for (int i=0; i<names_cvfh_.size(); ++i)
        {// for each cluster in database
          if (i != 0 && i != (names_cvfh_.size()-1) ) //nor first neither last
          {
            if (names_cvfh_[i].compare(names_cvfh_[i-1]) == 0)
            {//another cluster of the same object
              d = std::min (d, (getMinMaxDistance(target->points[n].histogram, (*cvfh_)[i], 308)) );
            }
            else
            {//another cluster of another object
              if (n==0)
                distIdx.push_back( std::make_pair(d, s++) );
              else
                distIdx[s++].first += d;
              d = getMinMaxDistance (target->points[n].histogram, (*cvfh_)[i], 308);
            }
          }
          else if (i == (names_cvfh_.size() -1) )
          {//last cluster of last object
            if (names_cvfh_[i].compare(names_cvfh_[i-1]) == 0)
            {//last cluster is still part of previous object
              d = std::min (d, (getMinMaxDistance(target->points[n].histogram, (*cvfh_)[i], 308)) );
              if (n==0)
                distIdx.push_back( std::make_pair(d, s++) );
              else
                distIdx[s++].first += d;
            }
            else
            {//last cluster is part of another last object
              d = getMinMaxDistance(target->points[n].histogram, (*cvfh_)[i], 308);
              if (n==0)
                distIdx.push_back( std::make_pair(d, s++) );
              else
                distIdx[s++].first += d;
            }
          }
          else
          {//first cluster of first object
            d = getMinMaxDistance (target->points[n].histogram, (*cvfh_)[i], 308);
          }
        }//end of for each cluster in db
      }//end of for each target cluster
      return true;
    }
    else if ( feat == listType::ourcvfh )
    { //ourcvfh list
      for (int n=0; n<target->points.size(); ++n)
      {//for each target cluster
        float d; //tmp distance
        int s=0; //counts objects
        for (int i=0; i<names_ourcvfh_.size(); ++i)
        {// for each cluster in database
          if (i != 0 && i != (names_ourcvfh_.size()-1) ) //not first neither last
          {
            if (names_ourcvfh_[i].compare(names_ourcvfh_[i-1]) == 0)
            {//another cluster of the same object
              d = std::min (d, (getMinMaxDistance(target->points[n].histogram, (*ourcvfh_)[i], 308)) );
            }
            else
            {//another cluster of another object
              if (n==0)
                distIdx.push_back( std::make_pair(d, s++) );
              else
                distIdx[s++].first += d;
              d = getMinMaxDistance (target->points[n].histogram, (*ourcvfh_)[i], 308);
            }
          }
          else if (i == (names_cvfh_.size() -1) )
          {//last cluster of last object
            if (names_cvfh_[i].compare(names_cvfh_[i-1]) == 0)
            {//last cluster is still part of previous object
              d = std::min (d, (getMinMaxDistance(target->points[n].histogram, (*ourcvfh_)[i], 308)) );
              if (n==0)
                distIdx.push_back( std::make_pair(d, s++) );
              else
                distIdx[s++].first += d;
            }
            else
            {//last cluster is part of another last object
              d = getMinMaxDistance(target->points[n].histogram, (*ourcvfh_)[i], 308);
              if (n==0)
                distIdx.push_back( std::make_pair(d, s++) );
              else
                distIdx[s++].first += d;
            }
          }
          else
          {//first cluster of first object
            d = getMinMaxDistance (target->points[n].histogram, (*ourcvfh_)[i], 308);
          }
        }//end of for each cluster in db
      }//end of for each target
      return true;
    }
    else
    {
      print_error("%*s]\tfeat must be 'listType::cvfh' or 'listType::ourcvfh'! Exiting...\n",20,__func__);
      return false;
    }
  }

  void
  Database::clear ()
  {
    vfh_.reset();
    esf_.reset();
    cvfh_.reset();
    ourcvfh_.reset();
    names_.clear();
    names_cvfh_.clear();
    names_ourcvfh_.clear();
    vfh_idx_.reset();
    esf_idx_.reset();
    clouds_.clear();
    db_path_.clear();
  }
}
