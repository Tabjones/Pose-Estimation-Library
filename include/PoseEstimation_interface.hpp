/* This file implementes the Pose Estimation interface, thus contains definitions,
 * for the interface declarations look in PoseEstimation_interface.h
 */
#ifndef __INTERFACE_HPP_INCLUDED__
#define __INTERFACE_HPP_INCLUDED__

#include <pcl/io/pcd_io.h>
#include <pcl/common/norms.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/vfh.h>
#include <pcl/features/esf.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <stdexcept>
#include <pcl/common/centroid.h>
//Definition header
#include "PoseEstimation_interface.h"


/**\ingroup Definitions
Degrees to radians conversion
*/
#define D2R 0.017453293 

using namespace pcl::console;
using namespace boost;
using namespace boost::filesystem;
using namespace flann;

/**\addtogroup global Global Functions
 *
 * General utilities functions
 * @{ */
/**\brief Compute the MinMax distance between two histograms, used by CVFH and OURCVFH
 * \param[in] a The first histogram
 * \param[in] b The second histogram
 * \returns The computed dstance _D_
 *
 * The distance _D_ is defined by the following metric:
 * \f[
 *  D = 1 - \frac{1+\sum_i^n{min\left(a_i,b_i\right)}}{1+\sum_i^n{max\left(a_i,b_i\right)}}
 * \f]
 * where n=308 for CVFH/OURCVFH histograms
 */
float MinMaxDistance (vector<float>& a, vector<float>& b)
{
  if (a.size() != b.size())
  {
    print_error("%*s]\tVectors size mismatch!\n",20,__func__);
    return (-1);
  }
  else
  {
    int size = a.size();
    float num(1.0f), den(1.0f);
    //Process 11 items with each loop for efficency (since it should be applied to vectors of 308 elements)
    int i=0;
    for (; i<(size-10); i+=11)
    { 
      num += min(a[i],b[i]) + min(a[i+1],b[i+1]) + min(a[i+2],b[i+2]) + min(a[i+3],b[i+3]) + min(a[i+4],b[i+4]) + min(a[i+5],b[i+5]);
      num += min(a[i+6],b[i+6]) + min(a[i+7],b[i+7]) + min(a[i+8],b[i+8]) + min(a[i+9],b[i+9]) + min(a[i+10],b[i+10]);
      den += max(a[i],b[i]) + max(a[i+1],b[i+1]) + max(a[i+2],b[i+2]) + max(a[i+3],b[i+3]) + max(a[i+4],b[i+4]) + max(a[i+5],b[i+5]);
      den += max(a[i+6],b[i+6]) + max(a[i+7],b[i+7]) + max(a[i+8],b[i+8]) + max(a[i+9],b[i+9]) + max(a[i+10],b[i+10]);
    }
    //process last 0-10 elements (if size!=308)
    while ( i < size)
    {
      num += min(a[i],b[i]);
      den += max(a[i],b[i]);
      ++i;
    }
    return (1 - (num/den));
  }
}
/** @}*/
bool PoseDB::isEmpty()
{
  if ( !(vfh_) || !(esf_) || !(cvfh_) || !(ourcvfh_) )
    return true;
  else if (names_.empty() || clusters_cvfh_.empty() || clusters_ourcvfh_.empty() || clouds_.empty() )
    return true;
  else if ( !(vfh_idx_) || !(esf_idx_) )
    return true;
  else
    return false;
}

bool PoseDB::isValidPath(path dbPath)
{
  if ( !exists(dbPath) || !is_directory(dbPath) )
    return false;
  path Pclouds(dbPath.string()+"/Clouds");
  if ( !exists(Pclouds) || !is_directory(Pclouds) )
    return false;  
  if ( !is_regular_file(dbPath.string()+ "/vfh.h5") || !(extension(dbPath.string()+ "/vfh.h5") == ".h5"))
    return false;
  if ( !is_regular_file(dbPath.string()+ "/esf.h5") || !(extension(dbPath.string()+ "/esf.h5") == ".h5"))
    return false;
  if ( !is_regular_file(dbPath.string()+ "/cvfh.h5") || !(extension(dbPath.string()+ "/cvfh.h5") == ".h5"))
    return false;
  if ( !is_regular_file(dbPath.string()+ "/ourcvfh.h5") || !(extension(dbPath.string()+ "/ourcvfh.h5") == ".h5"))
    return false;
  if ( !is_regular_file(dbPath.string()+ "/vfh.idx") || !(extension(dbPath.string()+ "/vfh.idx") == ".idx"))
    return false;
  if ( !is_regular_file(dbPath.string()+ "/esf.idx") || !(extension(dbPath.string()+ "/esf.idx") == ".idx"))
    return false;
  if ( !is_regular_file(dbPath.string()+ "/names.list") || !(extension(dbPath.string()+ "/names.list") == ".list"))
    return false;
  if ( !is_regular_file(dbPath.string()+ "/cvfh.cluster") || !(extension(dbPath.string()+ "/cvfh.cluster") == ".cluster"))
    return false;
  if ( !is_regular_file(dbPath.string()+ "/ourcvfh.cluster") || !(extension(dbPath.string()+ "/ourcvfh.cluster") == ".cluster"))
    return false;
  
  return true;
}

PoseDB::PoseDB(const PoseDB& db)
{
  histograms vfh (new float[db.vfh_->rows * db.vfh_->cols], db.vfh_->rows, db.vfh_->cols);
  for (size_t i=0; i<db.vfh_->rows; ++i)
    for (size_t j=0; j<db.vfh_->cols; ++j)
      vfh[i][j] = (*db.vfh_)[i][j];
  vfh_ = boost::make_shared<histograms>(vfh);
  histograms esf (new float[db.esf_->rows * db.esf_->cols], db.esf_->rows, db.esf_->cols);
  for (size_t i=0; i<db.esf_->rows; ++i)
    for (size_t j=0; j<db.esf_->cols; ++j)
      esf[i][j] = (*db.esf_)[i][j];
  esf_ = boost::make_shared<histograms>(esf);
  histograms cvfh (new float[db.cvfh_->rows * db.cvfh_->cols], db.cvfh_->rows, db.cvfh_->cols);
  for (size_t i=0; i<db.cvfh_->rows; ++i)
    for (size_t j=0; j<db.cvfh_->cols; ++j)
      cvfh[i][j] = (*db.cvfh_)[i][j];
  cvfh_ = boost::make_shared<histograms>(cvfh);
  histograms ourcvfh (new float[db.ourcvfh_->rows * db.ourcvfh_->cols], db.ourcvfh_->rows, db.ourcvfh_->cols);
  for (size_t i=0; i<db.ourcvfh_->rows; ++i)
    for (size_t j=0; j<db.ourcvfh_->cols; ++j)
      ourcvfh[i][j] = (*db.ourcvfh_)[i][j];
  ourcvfh_ = boost::make_shared<histograms>(ourcvfh);
  boost::copy (db.names_, back_inserter(names_));
  boost::copy (db.clusters_cvfh_, back_inserter(clusters_cvfh_));
  boost::copy (db.clusters_ourcvfh_, back_inserter(clusters_ourcvfh_));
  boost::copy (db.clouds_, back_inserter(clouds_));
  dbPath_ = db.dbPath_;
  //only way to copy indexs that i'm aware of (save it to disk then load it)
  db.vfh_idx_->save(".idx__tmp");
  indexVFH idx_vfh (*vfh_, SavedIndexParams(".idx__tmp"));
  vfh_idx_ = boost::make_shared<indexVFH>(idx_vfh);
  vfh_idx_ -> buildIndex();
  boost::filesystem::remove(".idx__tmp");
  //only way to copy indexs that i'm aware of (save it to disk then load it)
  db.esf_idx_->save(".idx__tmp");
  indexESF idx_esf (*esf_, SavedIndexParams(".idx__tmp"));
  esf_idx_ = boost::make_shared<indexESF>(idx_esf);
  esf_idx_ -> buildIndex();
  boost::filesystem::remove(".idx__tmp");
}
PoseDB& PoseDB::operator= (const PoseDB& db)
{
  this->clear();
  histograms vfh (new float[db.vfh_->rows * db.vfh_->cols], db.vfh_->rows, db.vfh_->cols);
  for (size_t i=0; i<db.vfh_->rows; ++i)
    for (size_t j=0; j<db.vfh_->cols; ++j)
      vfh[i][j] = (*db.vfh_)[i][j];
  this->vfh_ = boost::make_shared<histograms>(vfh);
  histograms esf (new float[db.esf_->rows * db.esf_->cols], db.esf_->rows, db.esf_->cols);
  for (size_t i=0; i<db.esf_->rows; ++i)
    for (size_t j=0; j<db.esf_->cols; ++j)
      esf[i][j] = (*db.esf_)[i][j];
  this->esf_ = boost::make_shared<histograms>(esf);
  histograms cvfh (new float[db.cvfh_->rows * db.cvfh_->cols], db.cvfh_->rows, db.cvfh_->cols);
  for (size_t i=0; i<db.cvfh_->rows; ++i)
    for (size_t j=0; j<db.cvfh_->cols; ++j)
      cvfh[i][j] = (*db.cvfh_)[i][j];
  this->cvfh_ = boost::make_shared<histograms>(cvfh);
  histograms ourcvfh (new float[db.ourcvfh_->rows * db.ourcvfh_->cols], db.ourcvfh_->rows, db.ourcvfh_->cols);
  for (size_t i=0; i<db.ourcvfh_->rows; ++i)
    for (size_t j=0; j<db.ourcvfh_->cols; ++j)
      ourcvfh[i][j] = (*db.ourcvfh_)[i][j];
  this->ourcvfh_ = boost::make_shared<histograms>(ourcvfh);
  boost::copy (db.names_, back_inserter(this->names_));
  boost::copy (db.clusters_cvfh_, back_inserter(this->clusters_cvfh_));
  boost::copy (db.clusters_ourcvfh_, back_inserter(this->clusters_ourcvfh_));
  boost::copy (db.clouds_, back_inserter(this->clouds_));
  this->dbPath_ = db.dbPath_;
  //only way to copy indexs that i'm aware of (save it to disk then load it)
  db.vfh_idx_->save(".idx__tmp");
  indexVFH idx_vfh ((*this->vfh_), SavedIndexParams(".idx__tmp"));
  this->vfh_idx_ = boost::make_shared<indexVFH>(idx_vfh);
  this->vfh_idx_ -> buildIndex();
  boost::filesystem::remove(".idx__tmp"); //delete tmp file
  //only way to copy indexs that i'm aware of (save it to disk then load it)
  db.esf_idx_->save(".idx__tmp");
  indexESF idx_esf ((*this->esf_), SavedIndexParams(".idx__tmp"));
  this->esf_idx_ = boost::make_shared<indexESF>(idx_esf);
  this->esf_idx_ -> buildIndex();
  boost::filesystem::remove(".idx__tmp"); //delete tmp file
  return *this;
}
/* Class PoseDB Implementation */
bool PoseDB::load(path pathDB)
{
  
  if ( this->isValidPath(pathDB) )
  {
    path Pclouds(pathDB.string()+"/Clouds");
    vector<path> pvec;
    copy (directory_iterator(Pclouds), directory_iterator(), back_inserter(pvec));
    if (pvec.size() <= 0)
    {
      print_error("%*s]\tNo files in Clouds directory of database, cannot load poses, aborting...\n",20,__func__);
      return false;
    }
    sort (pvec.begin(), pvec.end());
    clouds_.resize(pvec.size());
    int i(0);
    for (vector<path>::const_iterator it(pvec.begin()); it != pvec.end(); ++it, ++i)
    {
      if (is_regular_file(*it) && extension(*it)==".pcd" )
      {  
        if (pcl::io::loadPCDFile (it->string(),clouds_[i])!=0)
        {
          print_warn("%*s]\tError loading PCD file: %s, skipping...\n",20,__func__,it->string().c_str());
          continue;
        }
      }
      else
      {
        print_warn("%*s]\t% is not a PCD file, skipping...\n",20,__func__,it->string().c_str());
        continue;
      }
    }
    try
    {
      histograms m;
      flann::load_from_file (m, pathDB.string() + "/vfh.h5", "VFH Histograms");
      vfh_ = boost::make_shared<histograms>(m);
    }
    catch (...)
    {
      print_error("%*s]\tError loading VFH histograms, file is likely corrupted, try recreating database...\n",20,__func__);
      return false;
    }
    try
    {
      histograms m;
      flann::load_from_file (m, pathDB.string() + "/esf.h5", "ESF Histograms");
      esf_ = boost::make_shared<histograms>(m);
    }
    catch (...)
    {
      print_error("%*s]\tError loading ESF histograms, file is likely corrupted, try recreating database...\n",20,__func__);
      return false;
    }
    try
    {
      histograms m;
      flann::load_from_file (m, pathDB.string() + "/cvfh.h5", "CVFH Histograms");
      cvfh_ = boost::make_shared<histograms>(m);
    }
    catch (...)
    {
      print_error("%*s]\tError loading CVFH histograms, file is likely corrupted, try recreating database...\n",20,__func__);
      return false;
    }
    try
    {
      histograms m;
      flann::load_from_file (m, pathDB.string() + "/ourcvfh.h5", "OURCVFH Histograms");
      ourcvfh_ = boost::make_shared<histograms>(m);
    }
    catch (...)
    {
      print_error("%*s]\tError loading OURCVFH histograms, file is likely corrupted, try recreating database...\n",20,__func__);
      return false;
    }
    try
    {
      indexVFH idx (*vfh_, SavedIndexParams(pathDB.string()+"/vfh.idx"));
      vfh_idx_ = boost::make_shared<indexVFH>(idx);
      vfh_idx_ -> buildIndex();
    }
    catch (...)
    {
      print_error("%*s]\tError loading VFH index, file is likely corrupted, try recreating database...\n",20,__func__);
      return false;
    }
    try
    {
      indexESF idx (*esf_, SavedIndexParams(pathDB.string()+"/esf.idx"));
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
      ifstream file ((pathDB.string()+"/names.list").c_str());
      string line;
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
      ifstream file ((pathDB.string()+"/cvfh.cluster").c_str());
      string line;
      if (file.is_open())
      {
        while (getline (file, line))
        {
          trim(line); //remove white spaces from line
          int c;
          try
          {
            c=stoi(line);
          }
          catch (...)
          {
            print_error("%*s]\tCannot convert string in cvfh.cluster, file is likely corrupted, try recreating database\n",20,__func__);
            return false;
          }
          clusters_cvfh_.push_back(c);
        }//end of file
      }
      else
      {
        print_error("%*s]\tError opening cvfh.cluster, file is likely corrupted, try recreating database\n",20,__func__);
        return false;
      }
    }
    catch (...)
    {
      print_error("%*s]\tError loading cvfh.cluster, file is likely corrupted, try recreating database\n",20,__func__);
      return false;
    }
    try
    {
      ifstream file ((pathDB.string()+"/ourcvfh.cluster").c_str());
      string line;
      if (file.is_open())
      {
        while (getline (file, line))
        {
          trim(line); //remove white spaces from line
          int c;
          try
          {
            c=stoi(line);
          }
          catch (...)
          {
            print_error("%*s]\tCannot convert string in ourcvfh.cluster, file is likely corrupted, try recreating  database\n",20,__func__);
            return false;
          }
          clusters_ourcvfh_.push_back(c);
        }//end of file
      }
      else
      {
        print_error("%*s]\tError opening ourcvfh.cluster, file is likely corrupted, try recreating database\n",20,__func__);
        return false;
      }
    }
    catch (...)
    {
      print_error("%*s]\tError loading ourcvfh.cluster, file is likely corrupted, try recreating database\n",20,__func__);
      return false;
    }
  }
  else
  {
    print_error("%*s]\t%s is not a valid database directory, or does not exist\n",20,__func__,pathDB.string().c_str());
    return false;
  }
  dbPath_=pathDB;
  return true;
}
//////////////////////////////////////
void PoseDB::computeDistanceFromClusters_(PointCloud<VFHSignature308>::Ptr query, int idx, string feature, float& distance)
{
  int clusters_query = query->points.size();
  int clusters;
  distance = 0;
  bool cvfh(false), ourcvfh(false);
  if ( feature.compare("CVFH")==0 )
  {
    cvfh=true;
    clusters = clusters_cvfh_[idx];
  }
  else if ( feature.compare("OURCVFH")==0 )
  {
    ourcvfh=true;
    clusters = clusters_ourcvfh_[idx];
  }
  else
  {
    print_error("%*s]\tFeature must be 'CVFH' or 'OURCVFH'! Exiting...\n",20,__func__);
    return;
  }
  for (int i=0; i< clusters_query; ++i)
  {
    vector<float> tmp_dist;
    vector<float> hist_query;
    for (int n=0; n<308; ++n)
      hist_query.push_back(query->points[i].histogram[n]);
    for (int j=0; j< clusters; ++j)
    {
      vector<float> hist;
      if (cvfh)
      {
        //find index in matrix, because it is different from the one in names cause of clusters
        int idxc=0;
        for (int m=0; m<idx; ++m)
          idxc+=clusters_cvfh_[m];
        for (int n=0; n<cvfh_->cols; ++n)
          hist.push_back((*cvfh_)[idxc+j][n]);
      }
      else if (ourcvfh)
      {
        //find index in matrix, because it is different from the one in names cause of clusters
        int idxc=0;
        for (int m=0; m<idx; ++m)
          idxc+=clusters_ourcvfh_[m];
        for (int n=0; n<ourcvfh_->cols; ++n)
          hist.push_back((*ourcvfh_)[idxc+j][n]);
      }
      tmp_dist.push_back(MinMaxDistance(hist_query, hist));
    }
    distance += *min_element(tmp_dist.begin(), tmp_dist.end());
  }
}
///////////////////////////////////////
void PoseDB::clear()
{
  vfh_.reset();
  esf_.reset();
  cvfh_.reset();
  ourcvfh_.reset();
  names_.clear();
  clusters_cvfh_.clear();
  clusters_ourcvfh_.clear();
  vfh_idx_.reset();
  esf_idx_.reset();
  clouds_.clear();
  dbPath_="UNSET";
}
/////////////////////////////////////////
void PoseDB::save(path pathDB)
{
  if (this->isEmpty())
  {
    print_warn("%*s]\tCurrent database is invalid or empty, not saving it...\n",20,__func__);
    return;
  }
  if ( !exists (pathDB) )
  {
    create_directory(pathDB);
    create_directory(pathDB.string() + "/Clouds/");
    print_info("%*s]\tCreated directory to contain database in %s\n",20,__func__,pathDB.string().c_str());
    dbPath_ = pathDB;
  }
  else
  {
    print_warn("%*s]\t%s already exists, not saving a database there, aborting...\n",20,__func__,pathDB.string().c_str());
    return;
  }
  PCDWriter writer;
  ofstream names, c_cvfh, c_ourcvfh;
  names.open((pathDB.string()+ "names.list").c_str());
  c_cvfh.open((pathDB.string()+ "cvfh.cluster").c_str());
  c_ourcvfh.open((pathDB.string()+ "ourcvfh.cluster").c_str());
  for (size_t i=0; i< names_.size(); ++i)
  {
    writer.writeBinaryCompressed(pathDB.string() + "/Clouds/" + names_[i] + ".pcd", clouds_[i]);
    names << names_[i] <<endl;
    c_cvfh << clusters_cvfh_[i] <<endl;
    c_ourcvfh << clusters_ourcvfh_[i] <<endl;
  }
  names.close();
  c_cvfh.close();
  c_ourcvfh.close();
  flann::save_to_file (*vfh_, pathDB.string() + "/vfh.h5", "VFH Histograms");
  flann::save_to_file (*esf_, pathDB.string() + "/esf.h5", "ESF Histograms");
  flann::save_to_file (*cvfh_, pathDB.string() + "/cvfh.h5", "CVFH Histograms");
  flann::save_to_file (*ourcvfh_, pathDB.string() + "/ourcvfh.h5", "OURCVFH Histograms");
  vfh_idx_->save ( pathDB.string() + "vfh.idx");
  esf_idx_->save ( pathDB.string() + "esf.idx");
  print_info("%*s]\tDone saving database, %d poses written to disk\n",20,__func__,names_.size());
}
/////////////////////////////////////////
void PoseDB::create(boost::filesystem::path pathClouds, boost::shared_ptr<parameters> params)
{
  //Check Parameters correctness 
  if (params->count("filtering"))
  {
    if (params->at("filtering") < 0)
    {
      print_warn("%*s]\tParameter filtering for database creation has negative value, reverting it to default...\n",20,__func__);
      params->at("filtering")=0;
    }
  }
  else 
  {
    print_warn("%*s]\tParameter filtering for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
    params->emplace ("filtering", 0);
  }
  if (params->count("upsampling"))
  {
    if (params->at("upsampling") < 0)
    {
      print_warn("%*s]\tParameter upsampling for database creation has negative value, reverting it to default...\n",20,__func__);
      params->at("upsampling")=0;
    }
  }
  else 
  {
    print_warn("%*s]\tParameter upsampling for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
    params->emplace ("upsampling", 0);
  }
  if (params->count("downsampling"))
  {
    if (params->at("downsampling") < 0)
    {
      print_warn("%*s]\tParameter downsampling for database creation has negative value, reverting it to default...\n",20,__func__);
      params->at("downsampling")=1;
    }
  }
  else 
  {
    print_warn("%*s]\tParameter downsampling for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
    params->emplace ("downsampling", 1);
  }
  if (params->at("filtering")>0)
  {
    if (params->count("filterMeanK"))
    {
      if (params->at("filterMeanK") < 0)
      {
        print_warn("%*s]\tParameter filterMeanK for database creation has negative value, reverting it to default...\n",20,__func__);
        params->at("filterMeanK")=50;
      }
    }
    else 
    {
      print_warn("%*s]\tParameter filterMeanK for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
      params->emplace ("filterMeanK", 50);
    }
    if (params->count("filterStdDevMulThresh"))
    {
      if (params->at("filterStdDevMulThresh") < 0)
      {
        print_warn("%*s]\tParameter filterStdDevMulThresh for database creation has negative value, reverting it to default...\n",20,__func__);
        params->at("filterStdDevMulThresh")=3;
      }
    }
    else 
    {
      print_warn("%*s]\tParameter filterStdDevMulThresh for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
      params->emplace ("filterStdDevMulThresh", 3);
    }
  }
  if (params->at("downsampling")>0)
  {
    if (params->count("vgridLeafSize"))
    {
      if (params->at("vgridLeafSize") < 0)
      {
        print_warn("%*s]\tParameter vgridLeafSize for database creation has negative value, reverting it to default...\n",20,__func__);
        params->at("vgridLeafSize")=0.003;
      }
    }
    else 
    {
      print_warn("%*s]\tParameter vgridLeafSize for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
      params->emplace ("vgridLeafSize", 0.003);
    }
  }
  if (params->at("upsampling")>0)
  { 
    if (params->count("mlsPolyOrder"))
    {
      if (params->at("mlsPolyOrder") < 0)
      {
        print_warn("%*s]\tParameter mlsPolyOrder for database creation has negative value, reverting it to default...\n",20,__func__);
        params->at("mlsPolyOrder")=2;
      }
    }
    else 
    {
      print_warn("%*s]\tParameter mlsPolyOrder for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
      params->emplace ("mlsPolyOrder", 2);
    }
    if (params->count("mlsPointDensity"))
    {
      if (params->at("mlsPointDensity") < 0)
      {
        print_warn("%*s]\tParameter mlsPointDensity for database creation has negative value, reverting it to default...\n",20,__func__);
        params->at("mlsPointDensity")=250;
      }
    }
    else 
    {
      print_warn("%*s]\tParameter mlsPointDensity for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
      params->emplace ("mlsPointDensity",250);
    }
    if (params->count("mlsPolyFit"))
    {
      if (params->at("mlsPolyFit") < 0)
      {
        print_warn("%*s]\tParameter mlsPolyFit for database creation has negative value, reverting it to default...\n",20,__func__);
        params->at("mlsPolyFit")=1;
      }
    }
    else 
    {
      print_warn("%*s]\tParameter mlsPolyFit for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
      params->emplace ("mlsPolyFit",1);
    }
    if (params->count("mlsSearchRadius"))
    {
      if (params->at("mlsSearchRadius") < 0)
      {
        print_warn("%*s]\tParameter mlsSearchRadius for database creation has negative value, reverting it to default...\n",20,__func__);
        params->at("mlsSearchRadius")=0.03;
      }
    }
    else 
    {
      print_warn("%*s]\tParameter mlsSearchRadius for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
      params->emplace ("mlsSearchRadius", 0.03);
    }
  }
  if (params->count("neRadiusSearch"))
  {
    if (params->at("neRadiusSearch") < 0)
    {
      print_warn("%*s]\tParameter neRadiusSearch for database creation has negative value, reverting it to default...\n",20,__func__);
      params->at("neRadiusSearch")=0.015;
    }
  }
  else 
  {
    print_warn("%*s]\tParameter neRadiusSearch for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
    params->emplace ("neRadiusSearch", 0.015);
  }
  if (params->count("useSOasViewpoint"))
  {
    if (params->at("useSOasViewpoint") < 0)
    {
      print_warn("%*s]\tParameter useSOasViewpoint for database creation has negative value, reverting it to default...\n",20,__func__);
      params->at("useSOasViewpoint")=1;
    }
  }
  else 
  {
    print_warn("%*s]\tParameter useSOasViewpoint for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
    params->emplace ("useSOasViewpoint", 1);
  }
  if (params->count("computeViewpointFromName"))
  {
    if (params->at("computeViewpointFromName") < 0)
    {
      print_warn("%*s]\tParameter computeViewpointFromName for database creation has negative value, reverting it to default...\n",20,__func__);
      params->at("computeViewpointFromName")=0;
    }
  }
  else 
  {
    print_warn("%*s]\tParameter computeViewpointFromName for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
    params->emplace ("computeViewpointFromName", 0);
  }
  if (params->count("cvfhEPSAngThresh"))
  {
    if (params->at("cvfhEPSAngThresh") < 0)
    {
      print_warn("%*s]\tParameter cvfhEPSAngThresh for database creation has negative value, reverting it to default...\n",20,__func__);
      params->at("cvfhEPSAngThresh")=7.5;
    }
  }
  else 
  {
    print_warn("%*s]\tParameter cvfhEPSAngThresh for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
    params->emplace ("cvfhEPSAngThresh", 7.5);
  }
  if (params->count("cvfhCurvThresh"))
  {
    if (params->at("cvfhCurvThresh") < 0)
    {
      print_warn("%*s]\tParameter cvfhCurvThresh for database creation has negative value, reverting it to default...\n",20,__func__);
      params->at("cvfhCurvThresh")=0.025;
    }
  }
  else 
  {
    print_warn("%*s]\tParameter cvfhCurvThresh for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
    params->emplace ("cvfhCurvThresh", 0.025);
  }
  if (params->count("cvfhClustTol"))
  {
    if (params->at("cvfhClustTol") < 0)
    {
      print_warn("%*s]\tParameter cvfhClustTol for database creation has negative value, reverting it to default...\n",20,__func__);
      params->at("cvfhClustTol")=0.01;
    }
  }
  else 
  {
    print_warn("%*s]\tParameter cvfhClustTol for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
    params->emplace ("cvfhClustTol", 0.01);
  }
  if (params->count("cvfhMinPoints"))
  {
    if (params->at("cvfhMinPoints") < 0)
    {
      print_warn("%*s]\tParameter cvfhMinPoints for database creation has negative value, reverting it to default...\n",20,__func__);
      params->at("cvfhMinPoints")=50;
    }
  }
  else 
  {
    print_warn("%*s]\tParameter cvfhMinPoints for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
    params->emplace ("cvfhMinPoints", 50);
  }
  if (params->count("ourcvfhEPSAngThresh"))
  {
    if (params->at("ourcvfhEPSAngThresh") < 0)
    {
      print_warn("%*s]\tParameter ourcvfhEPSAngThresh for database creation has negative value, reverting it to default...\n",20,__func__);
      params->at("ourcvfhEPSAngThresh")=7.5;
    }
  }
  else 
  {
    print_warn("%*s]\tParameter ourcvfhEPSAngThresh for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
    params->emplace ("ourcvfhEPSAngThresh", 7.5);
  }
  if (params->count("ourcvfhCurvThresh"))
  {
    if (params->at("ourcvfhCurvThresh") < 0)
    {
      print_warn("%*s]\tParameter ourcvfhCurvThresh for database creation has negative value, reverting it to default...\n",20,__func__);
      params->at("ourcvfhCurvThresh")=0.025;
    }
  }
  else 
  {
    print_warn("%*s]\tParameter ourcvfhCurvThresh for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
    params->emplace ("ourcvfhCurvThresh", 0.025);
  }
  if (params->count("ourcvfhClustTol"))
  {
    if (params->at("ourcvfhClustTol") < 0)
    {
      print_warn("%*s]\tParameter ourcvfhClustTol for database creation has negative value, reverting it to default...\n",20,__func__);
      params->at("ourcvfhClustTol")=0.01;
    }
  }
  else 
  {
    print_warn("%*s]\tParameter ourcvfhClustTol for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
    params->emplace ("ourcvfhClustTol", 0.01);
  }
  if (params->count("ourcvfhMinPoints"))
  {
    if (params->at("ourcvfhMinPoints") < 0)
    {
      print_warn("%*s]\tParameter ourcvfhMinPoints for database creation has negative value, reverting it to default...\n",20,__func__);
      params->at("ourcvfhMinPoints")=50;
    }
  }
  else 
  {
    print_warn("%*s]\tParameter ourcvfhMinPoints for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
    params->emplace ("ourcvfhMinPoints", 50);
  }
  if (params->count("ourcvfhAxisRatio"))
  {
    if (params->at("ourcvfhAxisRatio") < 0)
    {
      print_warn("%*s]\tParameter ourcvfhAxisRatio for database creation has negative value, reverting it to default...\n",20,__func__);
      params->at("ourcvfhAxisRatio")=0.95;
    }
  }
  else 
  {
    print_warn("%*s]\tParameter ourcvfhAxisRatio for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
    params->emplace ("ourcvfhAxisRatio", 0.95);
  }
  if (params->count("ourcvfhMinAxisValue"))
  {
    if (params->at("ourcvfhMinAxisValue") < 0)
    {
      print_warn("%*s]\tParameter ourcvfhMinAxisValue for database creation has negative value, reverting it to default...\n",20,__func__);
      params->at("ourcvfhMinAxisValue")=0.01;
    }
  }
  else 
  {
    print_warn("%*s]\tParameter ourcvfhMinAxisValue for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
    params->emplace ("ourcvfhMinAxisValue", 0.01);
  }
  if (params->count("ourcvfhRefineClusters"))
  {
    if (params->at("ourcvfhRefineClusters") < 0)
    {
      print_warn("%*s]\tParameter ourcvfhRefineClusters for database creation has negative value, reverting it to default...\n",20,__func__);
      params->at("ourcvfhRefineClusters")=1;
    }
  }
  else 
  {
    print_warn("%*s]\tParameter ourcvfhRefineClusters for database creation does not exist in provided map, creating it with default value...\n",20,__func__);
    params->emplace ("ourcvfhRefineClusters", 1);
  }
  //Start database creation
  if (exists(pathClouds) && is_directory(pathClouds))
  {
    this->clear();
    vector<path> pvec;
    copy(directory_iterator(pathClouds), directory_iterator(), back_inserter(pvec));
    sort(pvec.begin(), pvec.end());
    PointCloud<VFHSignature308>::Ptr tmp_vfh (new PointCloud<VFHSignature308>);
    PointCloud<VFHSignature308>::Ptr tmp_cvfh (new PointCloud<VFHSignature308>);
    PointCloud<VFHSignature308>::Ptr tmp_ourcvfh (new PointCloud<VFHSignature308>);
    PointCloud<ESFSignature640>::Ptr tmp_esf (new PointCloud<ESFSignature640>);
    PC::Ptr input (new PC); 
    int i(0);
    for (vector<path>::const_iterator it(pvec.begin()); it != pvec.end(); ++it, ++i)
    {
      if (is_regular_file (*it) && it->extension() == ".pcd")
      {
        if (pcl::io::loadPCDFile(it->string().c_str(), *input)!=0 ) //loadPCDFile returns 0 if success
        {
          print_warn("%*s]\tError Loading Cloud %s, skipping...\n",20,__func__,it->string().c_str());
          continue;
        }
      }
      else
      {
        print_warn("%*s]\tLoaded File (%s) is not a pcd, skipping...\n",20,__func__,it->string().c_str());
        continue;
      }
      vector<string> vst;
      split (vst, it->string(), boost::is_any_of("../\\"), boost::token_compress_on);
      names_.push_back(vst.at(vst.size()-2)); //filename without extension and path
      PC::Ptr output (new PC);
      if (params->at("filtering") >0)
      {
        StatisticalOutlierRemoval<PT> filter;
        filter.setMeanK ( params->at("filterMeanK") );
        filter.setStddevMulThresh ( params->at("filterStdDevMulThresh") );
        filter.setInputCloud(input);
        filter.filter(*output);
        copyPointCloud(*output, *input);
      }
      if (params->at("upsampling") >0)
      {
        MovingLeastSquares<PT, PT> mls;
        search::KdTree<PT>::Ptr tree (new search::KdTree<PT>);
        mls.setInputCloud (input);
        mls.setSearchMethod (tree);
        mls.setUpsamplingMethod (MovingLeastSquares<PT, PT>::RANDOM_UNIFORM_DENSITY);
        mls.setComputeNormals (false);
        mls.setPolynomialOrder ( params->at("mlsPolyOrder") );
        mls.setPolynomialFit ( params->at("mlsPolyFit") );
        mls.setSearchRadius ( params->at("mlsSearchRadius") );
        mls.setPointDensity( params->at("mlsPointDensity") );
        mls.process (*output); //Process Upsampling
        copyPointCloud(*output, *input);
      }
      if ((*params)["downsampling"]>0)
      {
        VoxelGrid <PT> vgrid;
        vgrid.setInputCloud (input);
        vgrid.setLeafSize ( params->at("vgridLeafSize"), params->at("vgridLeafSize"), params->at("vgridLeafSize")); //Downsample to 3mm
        vgrid.setDownsampleAllData (true);
        vgrid.filter (*output); //Process Downsampling
        copyPointCloud(*output, *input);
      }
      clouds_.push_back(*input);
      NormalEstimationOMP<PT, Normal> ne;
      search::KdTree<PT>::Ptr tree (new search::KdTree<PT>);
      PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
      ne.setSearchMethod(tree);
      ne.setRadiusSearch(params->at("neRadiusSearch"));
      ne.setNumberOfThreads(0); //use pcl autoallocation
      ne.setInputCloud(input);
      float vpx(0),vpy(0),vpz(1);
      if (params->at("computeViewpointFromName")>0)
      {
      //Try to compute viewpoint from cloud name
        try
        {
          //assume correct naming convection (name_lat_long)
          //If something goes wrong an exception is catched
          vector<string> vst;
          split(vst, names_[i], boost::is_any_of("_"), boost::token_compress_on);
          int lat,lon;
          lat = stoi(vst.at(1));
          lon = stoi(vst.at(2));
          //assume radius of one meter and reference frame in object center
          vpx = cos(lat*D2R)*sin(lon*D2R);
          vpy = sin(lat*D2R);
          vpz = cos(lat*D2R)*cos(lon*D2R);
        }
        catch (...)
        {
          //something went wrong
          print_error("%*s]\tCannot compute Viewpoint from cloud name... Check naming convention and object reference frame! Aborting...\n",20,__func__);
          return;
        }
      }
      else if (params->at("useSOasViewpoint")>0)
      {
        //Use Viewpoint stored in sensor_origin_ of cloud
        vpx = clouds_[i].sensor_origin_[0];
        vpy = clouds_[i].sensor_origin_[1];
        vpz = clouds_[i].sensor_origin_[2];
      }
      else
      {
        print_error("%*s]\tCannot obtain a viewpoint 'computeViewpointFromName' and 'useSOasViewpoint' are both zero, set one of them to continue...\n",20,__func__);
        return;
      }
      ne.setViewPoint(vpx,vpy,vpz);
      ne.compute(*normals);
      //VFH
      VFHEstimation<PT, Normal, VFHSignature308> vfhE;
      PointCloud<VFHSignature308> out;
      vfhE.setSearchMethod(tree);
      vfhE.setInputCloud (input);
      vfhE.setViewPoint (vpx, vpy, vpz);
      vfhE.setInputNormals (normals);
      vfhE.compute (out);
      tmp_vfh->push_back(out.points[0]);
      //ESF
      ESFEstimation<PT, ESFSignature640> esfE;
      PointCloud<ESFSignature640> out_esf;
      esfE.setSearchMethod(tree);
      esfE.setInputCloud (input);
      esfE.compute (out_esf);
      tmp_esf->push_back(out_esf.points[0]);
      //CVFH
      CVFHEstimation<PT, Normal, VFHSignature308> cvfhE;
      cvfhE.setSearchMethod(tree);
      cvfhE.setInputCloud (input);
      cvfhE.setViewPoint (vpx, vpy, vpz);
      cvfhE.setInputNormals (normals);
      cvfhE.setEPSAngleThreshold(params->at("cvfhEPSAngThresh")*D2R); //angle needs to be supplied in radians
      cvfhE.setCurvatureThreshold(params->at("cvfhCurvThresh"));
      cvfhE.setClusterTolerance(params->at("cvfhClustTol"));
      cvfhE.setMinPoints(params->at("cvfhMinPoints"));
      cvfhE.setNormalizeBins(false);
      cvfhE.compute (out);
      clusters_cvfh_.push_back(out.points.size());
      for (size_t n=0; n<out.points.size(); ++n)
        tmp_cvfh->push_back(out.points[n]);
      //OURCVFH
      //For some reason OURCVFHEstimation is not templated to treat PointXYZRGBA point types...
      //Using PointXYZ...
      OURCVFHEstimation<PointXYZ, Normal, VFHSignature308> ourcvfhE;
      search::KdTree<PointXYZ>::Ptr tree2 (new search::KdTree<PointXYZ>);
      PointCloud<PointXYZ>::Ptr input2 (new PointCloud<PointXYZ>);
      copyPointCloud(*input, *input2);
      ourcvfhE.setSearchMethod(tree2);
      ourcvfhE.setInputCloud (input2);
      ourcvfhE.setViewPoint (vpx, vpy, vpz);
      ourcvfhE.setInputNormals (normals);
      ourcvfhE.setEPSAngleThreshold(params->at("ourcvfhEPSAngThresh")*D2R); //angle needs to be supplied in radians
      ourcvfhE.setCurvatureThreshold(params->at("ourcvfhCurvThresh"));
      ourcvfhE.setClusterTolerance(params->at("ourcvfhClustTol"));
      ourcvfhE.setMinPoints(params->at("ourcvfhMinPoints"));
      ourcvfhE.setAxisRatio(params->at("ourcvfhAxisRatio"));
      ourcvfhE.setMinAxisValue(params->at("ourcvfhMinAxisValue"));
      ourcvfhE.setRefineClusters(params->at("ourcvfhRefineClusters"));
      ourcvfhE.compute (out);
      clusters_ourcvfh_.push_back(out.points.size());
      for (size_t n=0; n<out.points.size(); ++n)
        tmp_ourcvfh->push_back(out.points[n]);
      print_info("%*s]\t%d clouds processed so far...\r",20,__func__,i+1);
      cout<<std::flush;
    }
    cout<<endl;
    if (tmp_vfh->points.size() == 0) //no clouds loaded
    {
      print_warn("%*s]\tNo clouds loaded, exiting...\n",20,__func__);
      return;
    }
    //generate FLANN matrix
    histograms vfh (new float[tmp_vfh->points.size()*308],tmp_vfh->points.size(),308);
    for (i=0; i<vfh.rows; ++i)
      for (int j=0; j<vfh.cols; ++j)
        vfh[i][j] = tmp_vfh->points[i].histogram[j];
    histograms esf (new float[tmp_esf->points.size()*640],tmp_esf->points.size(),640);
    for (i=0; i<esf.rows; ++i)
      for (int j=0; j<esf.cols; ++j)
        esf[i][j] = tmp_esf->points[i].histogram[j];
    histograms cvfh (new float[tmp_cvfh->points.size()*308],tmp_cvfh->points.size(),308);
    for (i=0; i<cvfh.rows; ++i)
      for (int j=0; j<cvfh.cols; ++j)
        cvfh[i][j] = tmp_cvfh->points[i].histogram[j];
    histograms ourcvfh (new float[tmp_ourcvfh->points.size()*308],tmp_ourcvfh->points.size(),308);
    for (i=0; i<ourcvfh.rows; ++i)
      for (int j=0; j<ourcvfh.cols; ++j)
        ourcvfh[i][j] = tmp_ourcvfh->points[i].histogram[j];
    vfh_ = boost::make_shared<histograms>(vfh);
    esf_ = boost::make_shared<histograms>(esf);
    cvfh_ = boost::make_shared<histograms>(cvfh);
    ourcvfh_ = boost::make_shared<histograms>(ourcvfh);
    dbPath_ = "UNSET";
    //and indices
    indexVFH vfh_idx (*vfh_, flann::KDTreeIndexParams(4));
    vfh_idx_ = boost::make_shared<indexVFH>(vfh_idx);
    vfh_idx_->buildIndex();
    indexESF esf_idx (*esf_, flann::KDTreeIndexParams(4));
    esf_idx_ = boost::make_shared<indexESF>(esf_idx);
    esf_idx_->buildIndex();
    print_info("%*s]\tDone creating database, total of %d poses stored in memory\n",20,__func__,names_.size());
  }
  else
  {
    print_error("%*s]\t%s is not a valid directory...\n",20,__func__,pathClouds.string().c_str());
  }
}
void PoseDB::create(boost::filesystem::path pathClouds)
{
  parameters par;
  //setting default parameters for db creation, only relevant ones are created
  par["vgridLeafSize"]=0.003f;
  par["upsampling"]=par["filtering"]=0;
  par["downsampling"]=1;
  par["mlsPolyOrder"]=2;
  par["mlsPointDensity"]=250;
  par["mlsPolyFit"]=1;
  par["mlsSearchRadius"]=0.03f;
  par["filterMeanK"]=50;
  par["filterStdDevMulThresh"]=3;
  par["neRadiusSearch"]=0.015;
  par["useSOasViewpoint"]=1;
  par["computeViewpointFromName"]=0;
  par["cvfhEPSAngThresh"]=7.5;
  par["cvfhCurvThresh"]=0.025;
  par["cvfhClustTol"]=0.01;
  par["cvfhMinPoints"]=50;
  par["ourcvfhEPSAngThresh"]=7.5;
  par["ourcvfhCurvThresh"]=0.025;
  par["ourcvfhClustTol"]=0.01;
  par["ourcvfhMinPoints"]=50;
  par["ourcvfhAxisRatio"]=0.95;
  par["ourcvfhMinAxisValue"]=0.01;
  par["ourcvfhRefineClusters"]=1;
  boost::shared_ptr<parameters> p;
  p=boost::make_shared<parameters>(par);
  create(pathClouds, p);
}
/////////////////////////////////////////
/* Class Candidate Implementation */
Candidate::Candidate ()
{
  name_ = "UNSET";
  rank_ = -1;
  distance_ = -1;
  rmse_ = -1;
  normalized_distance_=-1;
  transformation_.setIdentity();
}
///////////////////////////////////////////////////////////////////
Candidate::Candidate (string str, PC& cl)
{
  name_ = str;
  cloud_ = cl.makeShared();
  rank_ = -1;
  distance_ = -1;
  rmse_ = -1;
  normalized_distance_=-1;
  transformation_.setIdentity();
}
/////////////////////////////////////////////////////////////////////
Candidate::Candidate (string str, PC::Ptr clp)
{
  name_ = str;
  if (clp)
    cloud_ = clp;
  else
  {
    print_error("%*s]\tShared pointer provided is empty... Aborting Candidate creation\n",20,__func__);
    return;
  }
  rank_ = -1;
  distance_ = -1;
  rmse_ = -1;
  normalized_distance_=-1;
  transformation_.setIdentity();
}
///////////////////////////////////////////////////////////////////////
void Candidate::getRank (int& r) const
{ 
  if (rank_==-1)
    print_warn("%*s]\tCandidate is not part of any list (yet), thus it has no rank, writing -1 ...\n",20,__func__);
  r = rank_; 
}
void Candidate::getDistance (float& d) const
{ 
  if (distance_==-1)
    print_warn("%*s]\tCandidate is not part of any list (yet), thus it has no distance, writing -1 ...\n",20,__func__);
  d = distance_; 
}
void Candidate::getNormalizedDistance (float& d) const 
{ 
  if (normalized_distance_==-1)
    print_warn("%*s]\tCandidate is not part of any list (yet), thus it has no distance, writing -1 ...\n",20,__func__);
  d = normalized_distance_; 
}
void Candidate::getRMSE (float& r) const
{ 
  if (rmse_==-1)
    print_warn("%*s]\tCandidate has not been refined (yet), thus it has no RMSE, writing -1 ...\n",20,__func__);
  r = rmse_; 
}
void Candidate::getTransformation (Eigen::Matrix4f& t) const
{
  if (transformation_.isIdentity())
    print_warn("%*s]\tCandidate has Identity transformation, it probably hasn't been refined (yet)...\n",20,__func__);
  t = transformation_; 
}

/* Class PoseEstimation Implementation */
PoseEstimation::PoseEstimation ()
{
  vp_supplied_ = query_set_ = candidates_found_ = refinement_done_ = false;
  feature_count_ = 4;
  params_["verbosity"]=1;
  params_["useVFH"]=params_["useESF"]=params_["useCVFH"]=params_["useOURCVFH"]=1;
  params_["progBisection"]=params_["downsampling"]=1;
  params_["vgridLeafSize"]=0.003f;
  params_["upsampling"]=params_["filtering"]=0;
  params_["kNeighbors"]=20;
  params_["maxIterations"]=200;
  params_["progItera"]=5;
  params_["progFraction"]=0.5f;
  params_["rmseThreshold"]=0.003f;
  params_["mlsPolyOrder"]=2;
  params_["mlsPointDensity"]=250;
  params_["mlsPolyFit"]=1;
  params_["mlsSearchRadius"]=0.03f;
  params_["filterMeanK"]=50;
  params_["filterStdDevMulThresh"]=3;
  params_["neRadiusSearch"]=0.015;
  params_["useSOasViewpoint"]=1;
  params_["computeViewpointFromName"]=0;
  params_["cvfhEPSAngThresh"]=7.5;
  params_["cvfhCurvThresh"]=0.025;
  params_["cvfhClustTol"]=0.01;
  params_["cvfhMinPoints"]=50;
  params_["ourcvfhEPSAngThresh"]=7.5;
  params_["ourcvfhCurvThresh"]=0.025;
  params_["ourcvfhClustTol"]=0.01;
  params_["ourcvfhMinPoints"]=50;
  params_["ourcvfhAxisRatio"]=0.95;
  params_["ourcvfhMinAxisValue"]=0.01;
  params_["ourcvfhRefineClusters"]=1;
}
////////////////////////////////////////////////////////////////////////////////////////
void PoseEstimation::setParam(string key, float value)
{
  int size = params_.size();
  if (value < 0)
  {
    if (params_["verbosity"]>0)
      print_warn("%*s]\tParameter '%s' has a negative value (%g), ignoring...\n", 20,__func__, key.c_str(), value);
    return;
  }
  params_[key]=value;
  //Check if key was a valid one, since the class has fixed number of parameters, 
  //if one was mispelled, now we have one more 
  if (params_.size() != size)
  {
    if (params_["verbosity"]>0)
      print_warn("%*s]\tInvalid key parameter '%s', ignoring...\n", 20,__func__, key.c_str());
    params_.erase(key);
  }
  else if (params_["verbosity"]>1)
    print_info("%*s]\tSetting parameter: %s=%g\n",20,__func__,key.c_str(),value);
  //Recheck how many features we want
  int count(0);
  if (params_["useVFH"]>=1)
    count++;
  if (params_["useESF"]>=1)
    count++;
  if (params_["useCVFH"]>=1)
    count++;
  if (params_["useOURCVFH"]>=1)
    count++;
  feature_count_ = count;
  if (feature_count_ <=0 && params_["verbosity"]>0)
    print_warn("%*s]\tYou disabled all features, pose estimation will not initialize query...\n",20,__func__);
}
/////////////////////////////////////////////////////////////////////////////////////
void PoseEstimation::setParam_ (string key, string& value)
{
  float f;
  try
  {
    f = stof(value);
  }
  catch (const std::invalid_argument& ia)
  {
    if (params_["verbosity"] > 0)
      print_warn("%*s]\tInvalid %s=%s : %s \n",20,__func__, key.c_str(), value.c_str(), ia.what());
  }
  catch (const std::out_of_range& oor)
  {
    if (params_["verbosity"] > 0)
      print_warn("%*s]\tInvalid %s=%s : %s \n", 20,__func__, key.c_str(), value.c_str(), oor.what());
  }
  setParam(key, f); 
}
//////////////////////////////////////////////////////////////////////////////////////
void PoseEstimation::initParams(boost::filesystem::path config_file)
{ 
  if ( exists(config_file) && is_regular_file(config_file))   
  {
    if (extension(config_file)==".conf") 
    {
      ifstream file (config_file.string().c_str());
      string line;
      if (file.is_open())
      {
        while (getline (file, line))
        {
          trim(line); //remove white spaces from line
          if (line.empty())
          {
            //do nothing empty line ...
          }
          else if (line.compare(0,1,"%") == 0 )
          {
            //do nothing comment line ...
          }
          else
          {
            vector<string> vst;
            //split the line to get a key and a token
            split(vst, line, boost::is_any_of("="), boost::token_compress_on);
            if (vst.size()!=2)
            {
              if (params_["verbosity"]>0)
                print_warn("%*s]\tInvalid configuration line (%s), ignoring... Must be [Token]=[Value]\n", 20,__func__, line.c_str());
              continue;
            }
            setParam_(vst.at(0), vst.at(1));
          }
        }//end of config file
      }
      else
        print_error("%*s]\tCannot open config file! (%s)\n", 20,__func__, config_file.string().c_str());
    }  
    else
      print_error("%*s]\tConfig file provided (%s) has no valid extension! (must be .conf)\n", 20,__func__, config_file.string().c_str());
  }
  else
    print_error("%*s]\tPath to Config File is not valid ! (%s)\n", 20,__func__, config_file.string().c_str());
}
////////////////////////////////////////////////////////////////////////////////////////////////////
void PoseEstimation::initParams(boost::shared_ptr<parameters> map)
{
  //set parameters one by one so we can spot wrong key or values
  if (map)
  {
    for (auto &x : *map)
      setParam(x.first,x.second);
  }
  else
    if (params_["verbosity"]>0)
      print_warn("%*s]\tProvided map of parameters looks empty... Ignoring\n");
}
void PoseEstimation::filtering_()
{
  StopWatch timer;
  if (params_["verbosity"] >1)
  {
    print_info("%*s]\tSetting Statistical Outlier Filter to preprocess query cloud...\n",20,__func__);
    print_info("%*s]\tSetting mean K to %g\n",20,__func__, params_["filterMeanK"]);
    print_info("%*s]\tSetting Standard Deviation multiplier to %g\n",20,__func__, params_["filterStdDevMulThresh"]);
    timer.reset();
  }
  PC::Ptr filtered (new PC);
  StatisticalOutlierRemoval<PT> fil;
  fil.setMeanK (params_["filterMeanK"]);
  fil.setStddevMulThresh (params_["filterStdDevMulThresh"]);
  fil.setInputCloud(query_cloud_);
  fil.filter(*filtered);
  if (query_cloud_processed_)
    copyPointCloud(*filtered, *query_cloud_processed_);
  else
    query_cloud_processed_ = filtered;
  if (params_["verbosity"]>1)
  {
    print_info("%*s]\tTotal time elapsed during filter: ",20,__func__);
    print_value("%g", timer.getTime());
    print_info(" ms\n");
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void PoseEstimation::upsampling_()
{
  StopWatch timer;
  if (params_["verbosity"] >1)
  {
    print_info("%*s]\tSetting MLS with Random Uniform Density to preprocess query cloud...\n",20,__func__);
    print_info("%*s]\tSetting polynomial order to %g\n",20,__func__, params_["mlsPolyOrder"]);
    string t = params_["mlsPolyFit"] ? "true" : "false";
    print_info("%*s]\tSetting polynomial fit to %s\n",20,__func__, t.c_str());
    print_info("%*s]\tSetting desired point density to %g\n",20,__func__, params_["mlsPointDensity"]);
    print_info("%*s]\tSetting search radius to %g\n",20,__func__, params_["mlsSearchRadius"]);
    timer.reset();
  }
  PC::Ptr upsampled (new PC);
  search::KdTree<PT>::Ptr tree (new search::KdTree<PT>);
  MovingLeastSquares<PT, PT> mls;
  if (query_cloud_processed_)
    mls.setInputCloud(query_cloud_processed_);
  else
    mls.setInputCloud(query_cloud_);
  mls.setSearchMethod(tree);
  mls.setUpsamplingMethod (MovingLeastSquares<PT, PT>::RANDOM_UNIFORM_DENSITY);
  mls.setComputeNormals (false);
  mls.setPolynomialOrder(params_["mlsPolyOrder"]);
  mls.setPolynomialFit(params_["mlsPolyFit"]);
  mls.setSearchRadius(params_["mlsSearchRadius"]);
  mls.setPointDensity(params_["mlsPointDensity"]);
  mls.process(*upsampled);
  if (query_cloud_processed_)
    copyPointCloud(*upsampled, *query_cloud_processed_);
  else
    query_cloud_processed_ = upsampled;
  if (params_["verbosity"]>1)
  {
    print_info("%*s]\tTotal time elapsed during upsampling: ",20,__func__);
    print_value("%g", timer.getTime());
    print_info(" ms\n");
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
void PoseEstimation::downsampling_()
{
  StopWatch timer;
  if (params_["verbosity"] >1)
  {
    print_info("%*s]\tSetting Voxel Grid to preprocess query cloud...\n",20,__func__);
    print_info("%*s]\tSetting Leaf Size to %g\n",20,__func__, params_["vgridLeafSize"]);
    timer.reset();
  }
  PC::Ptr downsampled (new PC);
  VoxelGrid<PT> vg;
  if (query_cloud_processed_)
    vg.setInputCloud(query_cloud_processed_);
  else
    vg.setInputCloud(query_cloud_);
  vg.setLeafSize (params_["vgridLeafSize"], params_["vgridLeafSize"], params_["vgridLeafSize"]);
  vg.setDownsampleAllData (true);
  vg.filter(*downsampled);
  if (query_cloud_processed_)
    copyPointCloud(*downsampled, *query_cloud_processed_);
  else
    query_cloud_processed_ = downsampled;
  if (params_["verbosity"]>1)
  {
    print_info("%*s]\tTotal time elapsed during downsampling: ",20,__func__);
    print_value("%g", timer.getTime());
    print_info(" ms\n");
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
void PoseEstimation::setQueryViewpoint(float x, float y, float z)
{
  vpx_ = x;
  vpy_ = y;
  vpz_ = z;
  vp_supplied_ = true;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
bool PoseEstimation::computeNormals_()
{
  StopWatch timer;
  if (params_["verbosity"]>1)
  {
    print_info("%*s]\tSetting normal estimation to calculate query normals...\n",20,__func__);
    print_info("%*s]\tSetting a neighborhood radius of %g\n",20,__func__, params_["neRadiusSearch"]);
    timer.reset();
  }
  NormalEstimationOMP<PT, Normal> ne;
  search::KdTree<PT>::Ptr tree (new search::KdTree<PT>);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(params_["neRadiusSearch"]);
  ne.setNumberOfThreads(0); //use pcl autoallocation
  ne.setInputCloud(query_cloud_processed_);
  if (vp_supplied_)
  {
    //A Viewpoint was already supplied by setQueryViewpoint, so we use it
    ne.setViewPoint (vpx_, vpy_, vpz_);
    if (params_["verbosity"] >1)
      print_info("%*s]\tUsing supplied viewpoint: %g, %g, %g\n",20,__func__, vpx_, vpy_, vpz_);
  }
  else if (params_["computeViewpointFromName"])
  {
    //Try to compute viewpoint from query name
    try
    {
      //assume correct naming convection (name_lat_long)
      //If something goes wrong an exception is catched
      vector<string> vst;
      split(vst, query_name_, boost::is_any_of("_"), boost::token_compress_on);
      float vx,vy,vz;
      int lat,lon;
      lat = stoi(vst.at(1));
      lon = stoi(vst.at(2));
      //assume radius of one meter and reference frame in object center
      vx = cos(lat*D2R)*sin(lon*D2R);
      vy = sin(lat*D2R);
      vz = cos(lat*D2R)*cos(lon*D2R);
      setQueryViewpoint(vx,vy,vz);
      if (vp_supplied_)
      {
        if (params_["verbosity"]>1)
          print_info("%*s]\tUsing calculated viewpoint: %g, %g, %g\n",20,__func__, vpx_, vpy_, vpz_);
        ne.setViewPoint(vpx_, vpy_, vpz_);
      }
    }
    catch (...)
    {
      //something went wrong
      print_error("%*s]\tCannot compute Viewpoint from query name... Check naming convention and object reference frame!\n",20,__func__);
      return false;
    }
  }
  else if (params_["useSOasViewpoint"])
  {
    //Use Viewpoint stored in sensor_origin_ of query cloud
    //However we want to save it in class designated spots so it can be used again by 
    //other features
    float vx,vy,vz;
    vx = query_cloud_->sensor_origin_[0];
    vy = query_cloud_->sensor_origin_[1];
    vz = query_cloud_->sensor_origin_[2];
    setQueryViewpoint(vx,vy,vz);
    if (vp_supplied_)
    {
      ne.setViewPoint (vpx_, vpy_, vpz_);
      if (params_["verbosity"]>1)
        print_info("%*s]\tUsing viewpoint from sensor_origin_: %g, %g, %g\n",20,__func__, vpx_, vpy_, vpz_);
    }
    else
    {
      print_error("%*s]\tCannot set viewpoint from sensor_origin_...\n",20,__func__);
      return false;
    }
  }
  else
  {
    print_error("%*s]\tNo Viewpoint supplied!! Cannot continue!!\n",20,__func__);
    print_error("%*s]\tEither use setQueryViewpoint or set 'useSOasViewpoint'/'computeViewpointFromName'\n",20,__func__);
    return false;
  }
  ne.compute(normals_);
  if (params_["verbosity"]>1)
  {
    print_info("%*s]\tTotal time elapsed during normal estimation: ",20,__func__);
    print_value("%g", timer.getTime());
    print_info(" ms\n");
  }
  return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////
void PoseEstimation::computeVFH_()
{
  if(!vp_supplied_)
  {
    //Should never happen, viewpoint was set by normal estimation
    print_error("%*s]\tCannot estimate VFH of query, viewpoint was not set...\n",20,__func__);
    return;
  }
  else
  {
    StopWatch timer;
    if (params_["verbosity"]>1)
    {
      print_info("%*s]\tEstimating VFH feature of query...\n",20,__func__);
      timer.reset();
    }
    VFHEstimation<PT, Normal, VFHSignature308> vfhE;
    search::KdTree<PT>::Ptr tree (new search::KdTree<PT>);
    vfhE.setSearchMethod(tree);
    vfhE.setInputCloud (query_cloud_processed_);
    vfhE.setViewPoint (vpx_, vpy_, vpz_);
    vfhE.setInputNormals (normals_.makeShared());
    vfhE.compute (vfh_);
    if (params_["verbosity"]>1)
    {
      print_info("%*s]\tTotal time elapsed during VFH estimation: ",20,__func__);
      print_value("%g", timer.getTime());
      print_info(" ms\n");
    }
  }
}
////////////////////////////////////////////////////////////////////////////////////
void PoseEstimation::computeESF_()
{
  StopWatch timer;
  if (params_["verbosity"]>1)
  {
    print_info("%*s]\tEstimating ESF feature of query...\n",20,__func__);
    timer.reset();
  }
  ESFEstimation<PT, ESFSignature640> esfE;
  search::KdTree<PT>::Ptr tree (new search::KdTree<PT>);
  esfE.setSearchMethod(tree);
  esfE.setInputCloud (query_cloud_processed_);
  esfE.compute (esf_);
  if (params_["verbosity"]>1)
  {
    print_info("%*s]\tTotal time elapsed during ESF estimation: ",20,__func__);
    print_value("%g", timer.getTime());
    print_info(" ms\n");
  }
}
//////////////////////////////////////////////////////////////////////////////////////
void PoseEstimation::computeCVFH_()
{
  if(!vp_supplied_)
  {
    //Should never happen, viewpoint was set by normal estimation
    print_error("%*s]\tCannot estimate CVFH of query, viewpoint was not set...\n",20,__func__);
    return;
  }
  else
  {
    StopWatch timer;
    if (params_["verbosity"]>1)
    {
      print_info("%*s]\tEstimating CVFH feature of query...\n",20,__func__);
      print_info("%*s]\tUsing Angle Threshold of %g degress for normal deviation\n",20,__func__,params_["cvfhEPSAngThresh"]); 
      print_info("%*s]\tUsing Curvature Threshold of %g\n",20,__func__,params_["cvfhCurvThresh"]); 
      print_info("%*s]\tUsing Cluster Tolerance of %g\n",20,__func__,params_["cvfhClustTol"]); 
      print_info("%*s]\tConsidering a minimum of %g points for a cluster\n",20,__func__,params_["cvfhMinPoints"]); 
      timer.reset();
    }
    CVFHEstimation<PT, Normal, VFHSignature308> cvfhE;
    search::KdTree<PT>::Ptr tree (new search::KdTree<PT>);
    cvfhE.setSearchMethod(tree);
    cvfhE.setInputCloud (query_cloud_processed_);
    cvfhE.setViewPoint (vpx_, vpy_, vpz_);
    cvfhE.setInputNormals (normals_.makeShared());
    cvfhE.setEPSAngleThreshold(params_["cvfhEPSAngThresh"]*D2R); //angle needs to be supplied in radians
    cvfhE.setCurvatureThreshold(params_["cvfhCurvThresh"]);
    cvfhE.setClusterTolerance(params_["cvfhClustTol"]);
    cvfhE.setMinPoints(params_["cvfhMinPoints"]);
    cvfhE.setNormalizeBins(false);
    cvfhE.compute (cvfh_);
    if (params_["verbosity"]>1)
    {
      print_info("%*s]\tTotal of %d clusters were found on query\n",20,__func__, cvfh_.points.size());
      print_info("%*s]\tTotal time elapsed during CVFH estimation: ",20,__func__);
      print_value("%g", timer.getTime());
      print_info(" ms\n");
    }
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void PoseEstimation::computeOURCVFH_()
{
  if(!vp_supplied_)
  {
    //Should never happen, viewpoint was set by normal estimation
    print_error("%*s]\tCannot estimate OURCVFH of query, viewpoint was not set...\n",20,__func__);
    return;
  }
  else
  {
    StopWatch timer;
    if (params_["verbosity"]>1)
    {
      print_info("%*s]\tEstimating OURCVFH feature of query...\n",20,__func__);
      print_info("%*s]\tUsing Angle Threshold of %g degress for normal deviation\n",20,__func__,params_["ourcvfhEPSAngThresh"]); 
      print_info("%*s]\tUsing Curvature Threshold of %g\n",20,__func__,params_["ourcvfhCurvThresh"]); 
      print_info("%*s]\tUsing Cluster Tolerance of %g\n",20,__func__,params_["ourcvfhClustTol"]); 
      print_info("%*s]\tConsidering a minimum of %g points for a cluster\n",20,__func__,params_["ourcvfhMinPoints"]); 
      print_info("%*s]\tUsing Axis Ratio of %g and Min Axis Value of %g during SGURF disambiguation\n",20,__func__,params_["ourcvfhAxisRatio"],params_["ourcvfhMinAxisValue"]); 
      print_info("%*s]\tUsing Refinement Factor of %g for clusters\n",20,__func__,params_["ourcvfhRefineClusters"]); 
      timer.reset();
    }
    //For some reason OURCVFHEstimation is not templated to treat PointXYZRGBA point types...
    //Using PointXYZ...
    OURCVFHEstimation<PointXYZ, Normal, VFHSignature308> ourcvfhE;
    search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    copyPointCloud(*query_cloud_processed_, *cloud);
    ourcvfhE.setSearchMethod(tree);
    ourcvfhE.setInputCloud (cloud);
    ourcvfhE.setViewPoint (vpx_, vpy_, vpz_);
    ourcvfhE.setInputNormals (normals_.makeShared());
    ourcvfhE.setEPSAngleThreshold(params_["ourcvfhEPSAngThresh"]*D2R); //angle needs to be supplied in radians
    ourcvfhE.setCurvatureThreshold(params_["ourcvfhCurvThresh"]);
    ourcvfhE.setClusterTolerance(params_["ourcvfhClustTol"]);
    ourcvfhE.setMinPoints(params_["ourcvfhMinPoints"]);
    ourcvfhE.setAxisRatio(params_["ourcvfhAxisRatio"]);
    ourcvfhE.setMinAxisValue(params_["ourcvfhMinAxisValue"]);
    ourcvfhE.setRefineClusters(params_["ourcvfhRefineClusters"]);
    ourcvfhE.compute (ourcvfh_);
    if (params_["verbosity"]>1)
    {
      print_info("%*s]\tTotal of %d clusters were found on query\n",20,__func__, ourcvfh_.points.size());
      print_info("%*s]\tTotal time elapsed during OURCVFH estimation: ",20,__func__);
      print_value("%g", timer.getTime());
      print_info(" ms\n");
    }
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
bool PoseEstimation::initQuery_()
{
  StopWatch timer;
  if (params_["verbosity"]>1)
    timer.reset();
  if (feature_count_ <=0)
  {
    print_error("%*s]\tCannot initialize query, zero features chosen to estimate\n",20,__func__);
    return false;
  }
  //Check if a filter is needed
  if (params_["filtering"] >= 1)
    filtering_();
  //Check if upsampling is needed
  if (params_["upsampling"] >= 1)
    upsampling_();
  //Check if downsampling is needed
  if (params_["downsampling"] >= 1)
    downsampling_();
  if (! query_cloud_processed_ )
    query_cloud_processed_ = query_cloud_;

  //Check if we need ESF descriptor
  if (params_["useESF"] >= 1)
    computeESF_();
  //Check if we need Normals
  if (params_["useVFH"] >=1 || params_["useCVFH"] >=1 || params_["useOURCVFH"] >=1)
  {
    if (computeNormals_())
    {
      //And consequently other descriptors
      if (params_["useVFH"] >=1)
        computeVFH_();
      if (params_["useCVFH"] >=1)
        computeCVFH_();
      if (params_["useOURCVFH"] >=1)
        computeOURCVFH_();
    }
    else
      return false;
  }
  if (params_["verbosity"]>1)
  {
    print_info("%*s]\tQuery succesfully set and initialized:\n",20,__func__);
    print_info("%*s]\t",20,__func__);
    print_value("%s", query_name_.c_str());
    print_info(" with ");
    print_value("%d", query_cloud_processed_->points.size());
    print_info(" points\n");
    print_info("%*s]\tTotal time elapsed to initialize a query: ",20,__func__);
    print_value("%g", timer.getTime());
    print_info(" ms\n");
  }
  return true;
}
/////////////////////////////////////////////////////////////////////////////////////
void PoseEstimation::setQuery(string str, PC& cl)
{
  query_name_ = str;
  if (query_cloud_)
    copyPointCloud(cl, *query_cloud_);
  else
    query_cloud_ = cl.makeShared();
  if (initQuery_())
    query_set_ = true;
}
/////////////////////////////////////////////////////////////////////////////////////
void PoseEstimation::setQuery(string str, PC::Ptr clp)
{
  query_name_ = str;
  if (query_cloud_)
    copyPointCloud(*clp, *query_cloud_);
  else
    query_cloud_ = clp;
  if (initQuery_())
    query_set_ = true;
}
/////////////////////////////////////////////////////////////////////////////////////////
void PoseEstimation::printParams()
{
  if (params_["verbosity"]>1)
    print_info("%*s]\tPrinting current list of parameters:\n",20,__func__);
  for (auto &x : params_)
    cout<< x.first.c_str() <<"="<< x.second<<endl;
}
void PoseEstimation::setDatabase(PoseDB& database)
{
  if (database.isEmpty())
  {
    print_warn("%*s]\tProvided database is empty or invalid, skipping new database setting...\n",20,__func__);
    return;
  }
  database_.clear();
  db_set_= false;
  StopWatch timer;
  timer.reset();
  database_ = database;
  db_set_ = true;
  if (params_["verbosity"]>1)
  {
    print_info("%*s]\tDatabase set in ",20,__func__);
    print_value("%g", timer.getTime());
    print_info(" ms\n");
    print_info("%*s]\tTotal number of poses found: ",20,__func__);
    print_value("%d\n",database_.names_.size());
  }
}
/////////////////////////////////////////////////////////////////////////////////////
void PoseEstimation::setDatabase(boost::filesystem::path dbPath)
{
  if (!exists(dbPath) || !is_directory(dbPath))
  {
    print_warn("%*s]\t%s is not a directory or does not exists, skipping new database setting...\n",20,__func__);
    return;
  }
  database_.clear();
  db_set_=false;
  StopWatch timer;
  timer.reset();
  if (database_.load(dbPath))
  {
    db_set_ = true;
    if (params_["verbosity"]>1)
    {
      print_info("%*s]\tDatabase loaded and set from location %s in ",20,__func__,dbPath.string().c_str());
      print_value("%g", timer.getTime());
      print_info(" ms\n");
      print_info("%*s]\tTotal number of poses found: ",20,__func__);
      print_value("%d\n",database_.names_.size());
    }
  }
}
bool PoseEstimation::findCandidate_(vector<Candidate>& list, string name, float& dist)
{
  if (list.empty())
  {
    return false;
  }
  for (vector<Candidate>::iterator it=list.begin(); it!=list.end(); ++it)
  {
    if (it->name_.compare(name)==0)
    {
      dist = it->normalized_distance_;
      list.erase(it);
      return true;
    }
  }
  return false;
}
//////////////////////////////////////////////////////////////////////////////////////////
void PoseEstimation::generateLists()
{
  int k = params_["kNeighbors"];
  if (!db_set_)
  {
    print_error("%*s]\tDatabase is not set, set it with setDatabase first! Exiting...\n",20,__func__);
    return;
  }
  if (!query_set_)
  {
    print_error("%*s]\tQuery is not set, set it with setQuery first! Exiting...\n",20,__func__);
    return;
  }
  if (params_["verbosity"]>1)
    print_info("%*s]\tStarting Candidate list(s) generation...\n",20,__func__);
  StopWatch timer,t;
  timer.reset();
  if (params_["useVFH"]>=1)
  {
    if (params_["verbosity"]>1)
      print_info("%*s]\tGenerating List of Candidates, based on VFH... ",20,__func__);
    t.reset();
    try
    {
      vfh_list_.clear();
      flann::Matrix<float> vfh_query (new float[1*308],1,308);
      for (size_t j=0; j < 308; ++j)
        vfh_query[0][j]= vfh_.points[0].histogram[j];
      flann::Matrix<int> match_id (new int[1*k],1,k);
      flann::Matrix<float> match_dist (new float[1*k],1,k);
      database_.vfh_idx_->knnSearch (vfh_query, match_id, match_dist, k, SearchParams(256));
      for (size_t i=0; i<k; ++i)
      {
        string name= database_.names_[match_id[0][i]];
        Candidate c(name,database_.clouds_[match_id[0][i]]);
        c.rank_=i+1;
        c.distance_=match_dist[0][i];
        c.normalized_distance_=(match_dist[0][i] - match_dist[0][0])/(match_dist[0][k-1] - match_dist[0][0]);
        vfh_list_.push_back(c);
      }
    }
    catch (...)
    {
      if (params_["verbosity"]>1)
        print_error("ERROR\n");
      print_error("%*s]\tError Computing VFH list\n",20,__func__);
      return;
    }
    if (params_["verbosity"]>1)
    {
      print_value("%g",t.getTime());
      print_info(" ms elapsed\n");
    }
  }
  if (params_["useESF"]>=1)
  {
    if (params_["verbosity"]>1)
      print_info("%*s]\tGenerating List of Candidates, based on ESF... ",20,__func__);
    t.reset();
    try
    {
      esf_list_.clear();
      flann::Matrix<float> esf_query (new float[1*640],1,640);
      for (size_t j=0; j < 640; ++j)
        esf_query[0][j]= esf_.points[0].histogram[j];
      flann::Matrix<int> match_id (new int[1*k],1,k);
      flann::Matrix<float> match_dist (new float[1*k],1,k);
      database_.esf_idx_->knnSearch (esf_query, match_id, match_dist, k, SearchParams(256) );
      for (size_t i=0; i<k; ++i)
      {
        string name= database_.names_[match_id[0][i]];
        Candidate c(name,database_.clouds_[match_id[0][i]]);
        c.rank_=i+1;
        c.distance_=match_dist[0][i];
        c.normalized_distance_=(match_dist[0][i] - match_dist[0][0])/(match_dist[0][k-1] - match_dist[0][0]);
        esf_list_.push_back(c);
      }
    }
    catch (...)
    {
      if (params_["verbosity"]>1)
        print_error("ERROR\n");
      print_error("%*s]\tError Computing ESF list\n",20,__func__);
      return;
    }
    if (params_["verbosity"]>1)
    {
      print_value("%g",t.getTime());
      print_info(" ms elapsed\n");
    }
  }
  if (params_["useCVFH"]>=1)
  {
    if (params_["verbosity"]>1)
      print_info("%*s]\tGenerating List of Candidates, based on CVFH... ",20,__func__);
    t.reset();
    try
    {
      cvfh_list_.clear();
      for (int i=0; i<database_.names_.size(); ++i)
      {
        Candidate c;
        c.name_ = database_.names_[i];
        c.cloud_ = database_.clouds_[i].makeShared();
        database_.computeDistanceFromClusters_(cvfh_.makeShared() ,i ,"CVFH", c.distance_);
        cvfh_list_.push_back(c);
      }
      if ( cvfh_list_.size() >=k )
      {
        sort(cvfh_list_.begin(), cvfh_list_.end(),
            [](Candidate const &a, Candidate const &b)
            {
            float d,e;
            a.getDistance(d);
            b.getDistance(e);
            return (d < e);
            });
        cvfh_list_.resize(k);
      }
      else
      {
        if (params_["verbosity"]>1)
          print_error("ERROR\n");
        print_error("%*s]\tNot enough candidates to select in CVFH list (kNeighbors is too big)...\n",20,__func__);
        return;
      }
      for (int i=0; i<k; ++i)
      {
        cvfh_list_[i].rank_ = i+1;
        cvfh_list_[i].normalized_distance_=(cvfh_list_[i].distance_ - cvfh_list_[0].distance_)/(cvfh_list_[k-1].distance_ - cvfh_list_[0].distance_);
      }
    }
    catch (...)
    {
      if (params_["verbosity"]>1)
        print_error("ERROR\n");
      print_error("%*s]\tError computing CVFH list\n",20,__func__);
      return;
    }
    if (params_["verbosity"]>1)
    {
      print_value("%g",t.getTime());
      print_info(" ms elapsed\n");
    }
  }
  if (params_["useOURCVFH"]>=1)
  {
    if (params_["verbosity"]>1)
      print_info("%*s]\tGenerating List of Candidates, based on OURCVFH... ",20,__func__);
    t.reset();
    try
    {
      ourcvfh_list_.clear();
      for (int i=0; i<database_.names_.size(); ++i)
      {
        Candidate c;
        c.name_ = database_.names_[i];
        c.cloud_ = database_.clouds_[i].makeShared();
        database_.computeDistanceFromClusters_(ourcvfh_.makeShared() ,i ,"OURCVFH", c.distance_);
        ourcvfh_list_.push_back(c);
      }
      if ( ourcvfh_list_.size() >=k )
      {
        sort(ourcvfh_list_.begin(), ourcvfh_list_.end(),
            [](Candidate const &a, Candidate const &b)
            {
            float d,e;
            a.getDistance(d);
            b.getDistance(e);
            return (d < e);
            });
        ourcvfh_list_.resize(k);
      }
      else
      {
        if (params_["verbosity"]>1)
          print_error("ERROR\n");
        print_error("%*s]\tNot enough candidates to select in OURCVFH list (kNeighbors is too big)...\n",20,__func__);
        return;
      }
      for (int i=0; i<k; ++i)
      {
        ourcvfh_list_[i].rank_ = i+1;
        ourcvfh_list_[i].normalized_distance_=(ourcvfh_list_[i].distance_ - ourcvfh_list_[0].distance_)/(ourcvfh_list_[k-1].distance_ - ourcvfh_list_[0].distance_);
      }
    }
    catch (...)
    {
      if (params_["verbosity"]>1)
        print_error("ERROR\n");
      print_error("%*s]\tError computing OURCVFH list\n",20,__func__);
      return;
    }
    if (params_["verbosity"]>1)
    {
      print_value("%g",t.getTime());
      print_info(" ms elapsed\n");
    }
  }
  //Composite list generation
  if (params_["verbosity"]>1)
    print_info("%*s]\tGenerating Composite List based on previous features... ",20,__func__);
  t.reset();
  composite_list_.clear();
  vector<Candidate> tmp_esf, tmp_cvfh, tmp_ourcvfh, tmp_vfh;
  if(params_["useVFH"]>0)
  {
    if (feature_count_ == 1)
    {
      boost::copy(vfh_list_, back_inserter(composite_list_) );
      candidates_found_ = true;
      if (params_["verbosity"]>1)
      {
        print_value("%g",t.getTime());
        print_info(" ms elapsed\n");
      }
      return;
    }
    boost::copy(vfh_list_, back_inserter(tmp_vfh) );
  }
  if(params_["useESF"]>0)
  {
    if (feature_count_ == 1)
    {
      boost::copy(esf_list_, back_inserter(composite_list_) );
      candidates_found_ = true;
      if (params_["verbosity"]>1)
      {
        print_value("%g",t.getTime());
        print_info(" ms elapsed\n");
      }
      return;
    }
    boost::copy(esf_list_, back_inserter(tmp_esf) );
  }
  if(params_["useCVFH"]>0)
  {
    if (feature_count_ == 1)
    {
      boost::copy(cvfh_list_, back_inserter(composite_list_) );
      candidates_found_ = true;
      if (params_["verbosity"]>1)
      {
        print_value("%g",t.getTime());
        print_info(" ms elapsed\n");
      }
      return;
    }
    boost::copy(cvfh_list_, back_inserter(tmp_cvfh) );
  }
  if(params_["useOURCVFH"]>0)
  {
    if (feature_count_ == 1)
    {
      boost::copy(ourcvfh_list_, back_inserter(composite_list_) );
      candidates_found_ = true;
      if (params_["verbosity"]>1)
      {
        print_value("%g",t.getTime());
        print_info(" ms elapsed\n");
      }
      return;
    }
    boost::copy(ourcvfh_list_, back_inserter(tmp_ourcvfh) );
  }
  if (params_["useVFH"]>0 && !tmp_vfh.empty())
  {
    for (vector<Candidate>::iterator it=tmp_vfh.begin(); it!=tmp_vfh.end(); ++it)
    {
      float d_tmp;
      if (params_["useESF"]>0)
      {
        if (findCandidate_(tmp_esf, it->name_, d_tmp))
          it->normalized_distance_+= d_tmp;
        else
          it->normalized_distance_ += 1;
      }
      if (params_["useCVFH"]>0)
      {
        if (findCandidate_(tmp_cvfh, it->name_, d_tmp))
          it->normalized_distance_+= d_tmp;
        else
          it->normalized_distance_+=1;
      }
      if (params_["useOURCVFH"]>0)
      {
        if (findCandidate_(tmp_ourcvfh, it->name_, d_tmp))
          it->normalized_distance_+= d_tmp;
        else
          it->normalized_distance_+=1;
      }
      it->normalized_distance_ /= feature_count_;
      composite_list_.push_back(*it);
    }
  }
  //Still some candidates in ESF that were not in VFH
  if (params_["useESF"]>0 && !tmp_esf.empty())
  {
    for (vector<Candidate>::iterator it=tmp_esf.begin(); it!=tmp_esf.end(); ++it)
    {
      float d_tmp;
      if (params_["useCVFH"]>0)
      {
        if (findCandidate_(tmp_cvfh, it->name_, d_tmp))
          it->normalized_distance_+= d_tmp;
        else
          it->normalized_distance_+=1;
      }
      if (params_["useOURCVFH"]>0)
      {
        if (findCandidate_(tmp_ourcvfh, it->name_, d_tmp))
          it->normalized_distance_+= d_tmp;
        else
          it->normalized_distance_+=1;
      }
      if (params_["useVFH"]>0)
        it->normalized_distance_ += 1;
      it->normalized_distance_ /= feature_count_;
      composite_list_.push_back(*it);
    }
  }
  //Still some candidates in CVFH that were not in VFH and ESF
  if (params_["useCVFH"]>0 && !tmp_cvfh.empty())
  {
    for (vector<Candidate>::iterator it=tmp_cvfh.begin(); it!=tmp_cvfh.end(); ++it)
    {
      float d_tmp;
      if (params_["useOURCVFH"]>0)
      {
        if (findCandidate_(tmp_ourcvfh, it->name_, d_tmp))
          it->normalized_distance_+= d_tmp;
        else
          it->normalized_distance_+=1;
      }
      if (params_["useVFH"]>0)
        it->normalized_distance_ += 1;
      if (params_["useESF"]>0)
        it->normalized_distance_ += 1;
      it->normalized_distance_ /= feature_count_;
      composite_list_.push_back(*it);
    }
  }
  //Still some candidates in OURCVFH that were not in other lists
  if (params_["useOURCVFH"]>0 && !tmp_ourcvfh.empty())
  {
    for (vector<Candidate>::iterator it=tmp_ourcvfh.begin(); it!=tmp_ourcvfh.end(); ++it)
    {
      if (params_["useVFH"]>0)
        it->normalized_distance_ += 1;
      if (params_["useESF"]>0)
        it->normalized_distance_ += 1;
      if (params_["useCVFH"]>0)
        it->normalized_distance_ += 1;
      it->normalized_distance_ /= feature_count_;
      composite_list_.push_back(*it);
    }
  }
  sort(composite_list_.begin(), composite_list_.end(),
      [](Candidate const & a, Candidate const & b)
      {
      float d,e;
      a.getNormalizedDistance(d);
      b.getNormalizedDistance(e);
      return (d < e);
      });  
  composite_list_.resize(k);
  for (vector<Candidate>::iterator it=composite_list_.begin(); it!=composite_list_.end(); ++it)
    it->rank_ = it - composite_list_.begin() +1; //write the rank of the candidate in the list
  candidates_found_ = true;
  if (params_["verbosity"]>1)
  {
    print_value("%g",t.getTime());
    print_info(" ms elapsed\n");
    print_info("%*s]\tTotal time elapsed to generate list(s) of candidates: ",20,__func__);
    print_value("%g",timer.getTime());
    print_info(" ms\n");
  }
  return;
}
//////////////////////////////////////////////////////////////////////////////////////////
void PoseEstimation::generateLists(boost::filesystem::path dbPath)
{
  if (!query_set_)
  {
    print_error("%*s]\tQuery is not set, set it with setQuery first! Exiting...\n",20,__func__);
    return;
  }
  if (db_set_ && params_["verbosity"]>0)
  {
    print_warn("%*s]\tDatabase was already set, but a new path was specified, loading the new database...\n",20,__func__);
    print_warn("%*s\tUse generateLists() without arguments if you want to keep using the previously loaded database\n",20,__func__);
  }
  setDatabase(dbPath);
  generateLists();
}
////////////////////////////////////////////////77
void PoseEstimation::printCandidates()
{
  if (!candidates_found_)
  {
    if (params_["verbosity"]>0)
      print_warn("%*s]\tList of Candidates are not yet generated, call generateLists before trying to print them!\n",20,__func__); 
    return;
  }
  if (params_["verbosity"]>1)
    print_info("%*s]\tPrinting current list of candidates:\n",20,__func__);
  print_info("%-6s","Rank");
  if (params_["useVFH"] >0)
    print_info("%-30s","VFH");
  if (params_["useESF"] >0)
    print_info("%-30s","ESF");
  if (params_["useCVFH"] >0)
    print_info("%-30s","CVFH");
  if (params_["useOURCVFH"] >0)
    print_info("%-30s","OURCVFH");
  print_info("\n");
  for ( int i=0; i< params_["kNeighbors"]; ++i)
  {
    print_value("%-6d", i+1);
    if (params_["useVFH"] >0)
    {
      print_info("%-15s D:",vfh_list_[i].name_.c_str());
      print_value("%-9g   ",vfh_list_[i].normalized_distance_);
    }
    if (params_["useESF"] >0)
    {
      print_info("%-15s D:",esf_list_[i].name_.c_str());
      print_value("%-9g   ",esf_list_[i].normalized_distance_);
    }
    if (params_["useCVFH"] >0)
    {
      print_info("%-15s D:",cvfh_list_[i].name_.c_str());
      print_value("%-9g   ",cvfh_list_[i].normalized_distance_);
    }
    if (params_["useOURCVFH"] >0)
    {
      print_info("%-15s D:",ourcvfh_list_[i].name_.c_str());
      print_value("%-9g   ",ourcvfh_list_[i].normalized_distance_);
    }
    print_info("\n");
  }
  cout<<endl;
  print_info("%-6s", "Rank");
  print_info("%-30s\n","Composite");
  for (int i=0; i<params_["kNeighbors"]; ++i)
  { 
    print_value("%-6d", (int)composite_list_[i].rank_);
    print_info("%-15s D:", composite_list_[i].name_.c_str());
    print_value("%-9g   ", composite_list_[i].normalized_distance_);
    cout<<endl;
  }
}

void PoseEstimation::refineCandidates()
{
  if (!candidates_found_)
  {
    print_error("%*s]\tList of Candidates are not yet generated, call generateLists first...\n",20,__func__);
    return;
  }
  if (params_["progBisection"]>0)
  {
    //ProgressiveBisection
    if (params_["verbosity"]>1)
      print_info("%*s]\tStarting Progressive Bisection...\n",20,__func__);
    StopWatch timer;
    timer.reset();
    vector<Candidate> list; //make a temporary list to manipulate
    boost::copy(composite_list_, back_inserter(list));
    IterativeClosestPoint<PT, PT> icp;
    icp.setInputTarget(query_cloud_processed_); //query
    icp.setMaximumIterations (params_["progItera"]); //iterations to perform
    icp.setTransformationEpsilon (1e-9); //not using it (difference between consecutive transformations)
    icp.setEuclideanFitnessEpsilon (1e-9); //not using it (sum of euclidean distances between points)
    while (list.size() > 1)
    {
      for (vector<Candidate>::iterator it=list.begin(); it!=list.end(); ++it)
      {
        PC::Ptr aligned (new PC);
        //icp align source over target, result in aligned
        icp.setInputSource(it->cloud_); //the candidate
        icp.align(*aligned);
        it->transformation_ = icp.getFinalTransformation();
        it->rmse_ = sqrt(icp.getFitnessScore());
        if (params_["verbosity"]>1)
        {
          print_info("%*s]\tCandidate: ",20,__func__);
          print_value("%-15s",it->name_.c_str());
          print_info(" just performed %d ICP iterations, its RMSE is: ", (int)params_["progItera"]);
          print_value("%g\n",it->rmse_);
        }
      }
      //now resort list
      sort(list.begin(), list.end(),
          [](Candidate const &a, Candidate const &b)
          {
          float d,e;
          a.getRMSE(d);
          b.getRMSE(e);
          return (d < e);
          });
      //check if candidate falled under rmse threshold, no need to check them all since list is now sorted with
      //minimum rmse on top
      if (list[0].rmse_ < params_["rmseThreshold"] )
      {
        //convergence 
        pose_estimation_.reset();
        pose_estimation_ = boost::make_shared<Candidate>(list[0]);
        refinement_done_=true;
        if (params_["verbosity"]>1)
        {
          print_info("%*s]\tCandidate %s converged with RMSE %g\n",20,__func__,pose_estimation_->name_.c_str(), pose_estimation_->rmse_);
          print_info("%*s]\tFinal transformation is:\n",20,__func__);
          cout<<pose_estimation_->transformation_<<endl;
          print_info("%*s]\tTotal time elapsed in Progressive Bisection: ",20,__func__);
          print_value("%g",timer.getTime());
          print_info(" ms\n");
        }
        return;
      }
      else
      {
        //no convergence, resize list
        int size = list.size();
        size *= params_["progFraction"];
        list.resize(size);
        if (params_["verbosity"]>1)
          print_info("%*s]\tResizing... Keeping %d%% of previous list\n",20,__func__,(int)(params_["progFraction"]*100));
      }
    }
    //only one candidate remained, he wins!
    pose_estimation_.reset();
    pose_estimation_ = boost::make_shared<Candidate>(list[0]);
    refinement_done_=true;
    if (params_["verbosity"]>1)
    {
      print_info("%*s]\tCandidate %s survived progressive bisection with RMSE %g\n",20,__func__,pose_estimation_->name_.c_str(), pose_estimation_->rmse_);
      print_info("%*s]\tFinal transformation is:\n",20,__func__);
      cout<<pose_estimation_->transformation_<<endl;
      print_info("%*s]\tTotal time elapsed in Progressive Bisection: ",20,__func__);
      print_value("%g",timer.getTime());
      print_info(" ms\n");
    }
    return;
  }
  else
  {
    //BruteForce
    if (params_["verbosity"]>1)
      print_info("%*s]\tStarting Brute Force...\n",20,__func__);
    StopWatch timer;
    timer.reset();
    IterativeClosestPoint<PT, PT> icp;
    icp.setInputTarget(query_cloud_processed_); //query
    icp.setMaximumIterations (params_["maxIterations"]); //max iterations to perform
    icp.setTransformationEpsilon (1e-9); //not using it (difference between consecutive transformations)
    icp.setEuclideanFitnessEpsilon (pow(params_["rmseThreshold"],2)); 
    for (vector<Candidate>::iterator it=composite_list_.begin(); it!=composite_list_.end(); ++it)
    {
      PC::Ptr aligned (new PC);
      //icp align source over target, result in aligned
      icp.setInputSource(it->cloud_); //the candidate
      icp.align(*aligned);
      it->transformation_ = icp.getFinalTransformation();
      it->rmse_ = sqrt(icp.getFitnessScore());
      if (params_["verbosity"]>1)
      {
        print_info("%*s]\tCandidate: ",20,__func__);
        print_value("%-15s",it->name_.c_str());
        print_info(" just performed ICP alignment, its RMSE is: ");
        print_value("%g\n",it->rmse_);
      }
      if (it->rmse_ < params_["rmseThreshold"])
      {
        //convergence
        pose_estimation_.reset();
        pose_estimation_ = boost::make_shared<Candidate>(*it);
        refinement_done_=true;
        if (params_["verbosity"]>1)
        {
          print_info("%*s]\tCandidate %s converged with RMSE %g\n",20,__func__,pose_estimation_->name_.c_str(), pose_estimation_->rmse_);
          print_info("%*s]\tFinal transformation is:\n",20,__func__);
          cout<<pose_estimation_->transformation_<<endl;
          print_info("%*s]\tTotal time elapsed in Brute Force: ",20,__func__);
          print_value("%g",timer.getTime());
          print_info(" ms\n");
        }
        return;
      }
    }
    //no candidate converged, pose estimation failed
    pose_estimation_.reset();
    refinement_done_=false;
    if (params_["verbosity"]>0)
      print_warn("%*s]\tCannot find a suitable candidate, try raising the rmse threshold\n",20,__func__);
    if (params_["verbosity"]>1)
    {
      print_info("%*s]\tTotal time elapsed in Brute Force: ",20,__func__);
      print_value("%g",timer.getTime());
      print_info(" ms\n");
    }
    return;
  }
}

void PoseEstimation::estimate(string name, PC& cloud, boost::filesystem::path db_path)
{
  query_set_=candidates_found_=db_set_=refinement_done_ =false;
  setQuery(name, cloud);
  setDatabase(db_path);
  generateLists();
  refineCandidates();
}
void PoseEstimation::estimate(string name, PC::Ptr cloud_pointer, boost::filesystem::path db_path)
{
  query_set_=candidates_found_=db_set_=refinement_done_ =false;
  setQuery(name, cloud_pointer);
  setDatabase(db_path);
  generateLists();
  refineCandidates();
}
void PoseEstimation::estimate(string name, PC& cloud, PoseDB& database)
{
  query_set_=candidates_found_=db_set_=refinement_done_ =false;
  setQuery(name, cloud);
  setDatabase(database);
  generateLists();
  refineCandidates();
}
void PoseEstimation::estimate(string name, PC::Ptr cloud_pointer, PoseDB& database)
{
  query_set_=candidates_found_=db_set_=refinement_done_ =false;
  setQuery(name, cloud_pointer);
  setDatabase(database);
  generateLists();
  refineCandidates();
}
void PoseEstimation::printEstimation()
{
  if (!refinement_done_)
  {
    if (params_["verbosity"]>0)
      print_warn("%*s]\tRefinement is not done yet, or wasn't successful... Printing nothing...\n",20,__func__);
    return;
  }
  if (params_["verbosity"]>1)
    print_info("%*s]\tPrinting current pose estimation information:\n",20,__func__);
  print_info("Query object ");
  print_value("%-15s",query_name_.c_str());
  print_info(" was identified with candidate ");
  print_value("%-15s\n",pose_estimation_->name_.c_str());
  print_info("It was positioned on Rank ");
  print_value("%-3d",pose_estimation_->rank_);
  print_info(" of composite list, with normalized distance of ");
  print_value("%g\n",pose_estimation_->normalized_distance_);
  print_info("Its final RMSE is ");
  print_value("%g\n",pose_estimation_->rmse_);
  print_info("Final Transformation (in candidate reference system):\n");
  cout<<pose_estimation_->transformation_<<endl;
}
boost::shared_ptr<parameters> PoseEstimation::getParameters()
{
  parameters p = {{"key",-1}}; //initialize p with something so that copy assigned is performed instead of move (to preserve param_)
  p = params_;
  boost::shared_ptr<parameters> ptr;
  ptr = boost::make_shared<parameters>(p);
  return ptr;
}
bool PoseEstimation::saveEstimation(path file, bool append)
{
  if (!refinement_done_)
  {
    if (params_["verbosity"]>0)
      print_warn("%*s]\tRefinement is not done yet, or wasn't successful... Saving nothing...\n",20,__func__);
    return false;
  }
  if (exists(file) && is_directory(file)) //existant directory, save to <query_name>.estimation inside it
  {
    try 
    {
      fstream f;
      if (append)
        f.open ( (file.string() + query_name_ + ".estimation").c_str(), fstream::out | fstream::app );
      else
        f.open ( (file.string() + query_name_ + ".estimation").c_str(), fstream::trunc );
      f << "Query object: " << query_name_.c_str() << " is identified with candidate " << pose_estimation_->name_.c_str() <<endl;
      f << "Candidate was on rank " << pose_estimation_->rank_ << " of composite list with normalized distance of " << pose_estimation_->normalized_distance_ <<endl;
      f << "Final RMSE is " << pose_estimation_->rmse_ << endl;
      f << "Final transformation, in candidate reference system: "<<endl;
      f << pose_estimation_ -> transformation_ <<endl<<endl;
      f.close();
    }
    catch (...)
    {
      print_error ("%*s]\tError writing to disk, check if %s is valid path\n",20,__func__,(file.string() + query_name_ + ".estimation").c_str());
      return false;
    }
    if (params_["verbosity"]>1)
      print_info("%*s]\tSaved current pose estimation information in %s\n",20,__func__,(file.string() + query_name_ + ".estimation").c_str());
    return true;
  }
  else if (exists(file) && is_regular_file(file)) //existant file, append or overwrite
  {
    try 
    {
      fstream f;
      if (append)
        f.open ( file.string().c_str(), fstream::out | fstream::app );
      else
        f.open ( file.string().c_str(), fstream::trunc );
      f << "Query object: " << query_name_.c_str() << " is identified with candidate " << pose_estimation_->name_.c_str() <<endl;
      f << "Candidate was on rank " << pose_estimation_->rank_ << " of composite list with normalized distance of " << pose_estimation_->normalized_distance_ <<endl;
      f << "Final RMSE is " << pose_estimation_->rmse_ << endl;
      f << "Final transformation, in candidate reference system: "<<endl;
      f << pose_estimation_ -> transformation_ <<endl<<endl;
      f.close();
    }
    catch (...)
    {
      print_error ("%*s]\tError writing to disk, check if %s is valid path\n",20,__func__,file.string().c_str());
      return false;
    }
    if (params_["verbosity"]>1)
      print_info("%*s]\tSaved current pose estimation information in %s\n",20,__func__,file.string().c_str());
    return true;
  }
  else if (file.filename().has_extension() && !exists(file) ) //non existant file, creating it
  {
    if (file.has_parent_path() && !exists(file.parent_path()) ) //also container directory does not exists
    {
      try
      {
        create_directory(file.parent_path());
      }
      catch (...)
      {
        print_error("%*s]\tError creating directory %s\n",20,__func__,file.parent_path().string().c_str());
        return  false;
      }
    }
    try 
    {
      ofstream f;
      f.open ( file.string().c_str() );
      f << "Query object: " << query_name_.c_str() << " is identified with candidate " << pose_estimation_->name_.c_str() <<endl;
      f << "Candidate was on rank " << pose_estimation_->rank_ << " of composite list with normalized distance of " << pose_estimation_->normalized_distance_ <<endl;
      f << "Final RMSE is " << pose_estimation_->rmse_ << endl;
      f << "Final transformation, in candidate reference system: "<<endl;
      f << pose_estimation_ -> transformation_ <<endl<<endl;
      f.close();
    }
    catch (...)
    {
      print_error ("%*s]\tError writing to disk, check if %s is valid path\n",20,__func__,file.string().c_str());
      return false;
    }
    if (params_["verbosity"]>1)
      print_info("%*s]\tSaved current pose estimation information in %s\n",20,__func__,file.string().c_str());
    return true;
  }
  else if (!file.filename().has_extension() && !exists(file) ) //non existant directory, create then save inside it
  {
    try
    {
      create_directory(file);
      fstream f;
      if (append)
        f.open ( (file.string() + query_name_ + ".estimation").c_str(), fstream::out | fstream::app );
      else
        f.open ( (file.string() + query_name_ + ".estimation").c_str(), fstream::trunc );
      f << "Query object: " << query_name_.c_str() << " is identified with candidate " << pose_estimation_->name_.c_str() <<endl;
      f << "Candidate was on rank " << pose_estimation_->rank_ << " of composite list with normalized distance of " << pose_estimation_->normalized_distance_ <<endl;
      f << "Final RMSE is " << pose_estimation_->rmse_ << endl;
      f << "Final transformation, in candidate reference system: "<<endl;
      f << pose_estimation_ -> transformation_ <<endl<<endl;
      f.close();
    }
    catch (...)
    {
      print_error("%*s]\tError writing to disk, check if %s is valid path\n",20,__func__,(file.string() + query_name_ + ".estimation" ).c_str() );
      return  false;
    }
    if (params_["verbosity"]>1)
      print_info("%*s]\tSaved current pose estimation information in %s\n",20,__func__,(file.string() + query_name_ + ".estimation").c_str());
    return true;
  }
  else //unexpeted path error
  {
    print_error("%*s]\tError parsing path: %s\n",20,__func__,file.string().c_str() );
    return  false;
  }
}
#endif
