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

#include <pel/pose_estimation_base.h>
#include <pcl/common/angles.h>
#include <pel/database/database_io.h>

using namespace pcl::console;

namespace pel
{
  bool
  PoseEstimationBase::generateLists()
  {
    int k = getParam("lists_size");
    int verbosity = getParam("verbosity");
    if (this->isEmpty())
    {
      //Database is Empty
      print_error("%*s]\tDatabase is empty, set it first!\n",20,__func__);
      return false;
    }
    if (target_cloud->empty())
    {
      print_error("%*s]\tTarget is not set, set it first!\n",20,__func__);
      return false;
    }
    if (k > this->names_.size())
    {
      print_error("%*]\tNot enough candidates to select in database, lists_size param is bigger than database size, aborting...\n",20,__func__);
      return false;
    }
    if (verbosity > 1)
      print_info("%*s]\tStarting Candidate lists generation...\n",20,__func__);
    pcl::StopWatch timer,t;
    timer.reset();
    vfh_list.clear();
    esf_list.clear();
    cvfh_list.clear();
    ourcvfh_list.clear();
    composite_list.clear();
    if (getParam("use_vfh")>=1)
    {
      if (verbosity >1)
        print_info("%*s]\tGenerating List of Candidates, based on VFH... ",20,__func__);
      t.reset();
      try
      {
        flann::Matrix<float> vfh_query (new float[1*308],1,308);
        for (size_t j=0; j < 308; ++j)
          vfh_query[0][j]= target_vfh.points[0].histogram[j];
        flann::Matrix<int> match_id (new int[1*k],1,k);
        flann::Matrix<float> match_dist (new float[1*k],1,k);
        vfh_idx_->knnSearch (vfh_query, match_id, match_dist, k, SearchParams(256));
        for (size_t i=0; i<k; ++i)
        {
          std::string name= names_[match_id[0][i]];
          Candidate c(names_[match_id[0][i]],clouds_[match_id[0][i]].makeShared());
          c.setRank(i+1);
          c.setDistance(match_dist[0][i]);
          c.setNormalizedDistance( (match_dist[0][i] - match_dist[0][0])/(match_dist[0][k-1] - match_dist[0][0]));
          vfh_list.push_back(c);
        }
      }
      catch (...)
      {
        print_error("%*s]\tError Computing VFH list\n",20,__func__);
        return false;
      }
      if (verbosity > 1)
      {
        print_value("%g",t.getTime());
        print_info(" ms elapsed\n");
      }
    }
    if (getParam("use_esf")>=1)
    {
      if (verbosity >1)
        print_info("%*s]\tGenerating List of Candidates, based on ESF... ",20,__func__);
      t.reset();
      try
      {
        flann::Matrix<float> esf_query (new float[1*640],1,640);
        for (size_t j=0; j < 640; ++j)
          esf_query[0][j]= target_esf.points[0].histogram[j];
        flann::Matrix<int> match_id (new int[1*k],1,k);
        flann::Matrix<float> match_dist (new float[1*k],1,k);
        esf_idx_->knnSearch (esf_query, match_id, match_dist, k, SearchParams(256) );
        for (size_t i=0; i<k; ++i)
        {
          Candidate c(names_[match_id[0][i]],clouds_[match_id[0][i]].makeShared());
          c.setRank(i+1);
          c.setDistance(match_dist[0][i]);
          c.setNormalizedDistance( (match_dist[0][i] - match_dist[0][0])/(match_dist[0][k-1] - match_dist[0][0]) );
          esf_list.push_back(c);
        }
      }
      catch (...)
      {
        print_error("%*s]\tError Computing ESF list\n",20,__func__);
        return false;
      }
      if (verbosity > 1)
      {
        print_value("%g",t.getTime());
        print_info(" ms elapsed\n");
      }
    }
    if (getParam ("use_cvfh") >=1)
    {
      if (verbosity >1)
        print_info("%*s]\tGenerating List of Candidates, based on CVFH... ",20,__func__);
      t.reset();
      try
      {
        std::vector<std::pair<float, int> > dists;
        if (computeDistFromClusters(target_cvfh.makeShared(), ListType::cvfh, dists))
        {
          std::sort(dists.begin(), dists.end(),
              [](std::pair<float, int> const& a, std::pair<float, int> const& b)
              {
              return (a.first < b.first );
              });
          dists.resize(k);
          for (int i=0; i<k ; ++i)
          {
            Candidate c (names_[dists[i].second], clouds_[dists[i].second].makeShared());
            c.setRank(i+1);
            c.setDistance(dists[i].first);
            c.setNormalizedDistance( (dists[i].first - dists[0].first)/(dists[k-1].first - dists[0].first) );
            cvfh_list.push_back(c);
          }
        }
        else
        {
          print_error("%*s]\tError in computeDistFromClusters while generating CVFH list\n",20,__func__);
          return false;
        }
      }
      catch (...)
      {
        print_error("%*s]\tError computing CVFH list\n",20,__func__);
        return false;
      }
      if (verbosity>1)
      {
        print_value("%g",t.getTime());
        print_info(" ms elapsed\n");
      }
    }
    if (getParam("use_ourcvfh")>=1)
    {
      if (verbosity>1)
        print_info("%*s]\tGenerating List of Candidates, based on OURCVFH... ",20,__func__);
      t.reset();
      try
      {
        std::vector<std::pair<float, int> > dists;
        if (computeDistFromClusters(target_ourcvfh.makeShared(), ListType::ourcvfh, dists) )
        {
          std::sort(dists.begin(), dists.end(),
              [](std::pair<float, int> const& a, std::pair<float, int> const& b)
              {
              return (a.first < b.first );
              });
          dists.resize(k);
          for (int i=0; i<k ; ++i)
          {
            Candidate c (names_[dists[i].second], clouds_[dists[i].second].makeShared() );
            c.setRank(i+1);
            c.setDistance(dists[i].first);
            c.setNormalizedDistance( (dists[i].first - dists[0].first)/(dists[k-1].first - dists[0].first) );
            ourcvfh_list.push_back(c);
          }
        }
        else
        {
          print_error("%*s]\tError in computeDistFromClustersg while enerating OURCVFH list\n",20,__func__);
          return false;
        }
      }
      catch (...)
      {
        print_error("%*s]\tError computing OURCVFH list\n",20,__func__);
        return false;
      }
      if (verbosity >1)
      {
        print_value("%g",t.getTime());
        print_info(" ms elapsed\n");
      }
    }
    //Composite list generation
    if (verbosity>1)
      print_info("%*s]\tGenerating Composite List based on previous features... ",20,__func__);
    t.reset();
    std::vector<Candidate> tmp_esf, tmp_cvfh, tmp_ourcvfh, tmp_vfh;
    if(getParam("use_vfh")>0)
    {
      if (feature_count_ == 1)
      {
        boost::copy(vfh_list, back_inserter(composite_list) );
        if (verbosity>1)
        {
          print_value("%g",t.getTime());
          print_info(" ms elapsed\n");
        }
        return true;
      }
      boost::copy(vfh_list, back_inserter(tmp_vfh) );
    }
    if(getParam("use_esf")>0)
    {
      if (feature_count_ == 1)
      {
        boost::copy(esf_list, back_inserter(composite_list) );
        if (verbosity>1)
        {
          print_value("%g",t.getTime());
          print_info(" ms elapsed\n");
        }
        return true;
      }
      boost::copy(esf_list, back_inserter(tmp_esf) );
    }
    if(getParam("use_cvfh")>0)
    {
      if (feature_count_ == 1)
      {
        boost::copy(cvfh_list, back_inserter(composite_list) );
        if (verbosity>1)
        {
          print_value("%g",t.getTime());
          print_info(" ms elapsed\n");
        }
        return true;
      }
      boost::copy(cvfh_list, back_inserter(tmp_cvfh) );
    }
    if(getParam("use_ourcvfh")>0)
    {
      if (feature_count_ == 1)
      {
        boost::copy(ourcvfh_list, back_inserter(composite_list) );
        if (verbosity >1)
        {
          print_value("%g",t.getTime());
          print_info(" ms elapsed\n");
        }
        return true;
      }
      boost::copy(ourcvfh_list, back_inserter(tmp_ourcvfh) );
    }
    if (getParam("use_vfh") >0 && !tmp_vfh.empty())
    {
      for (auto& x : tmp_vfh)
      {
        float d_tmp;
        float x_dist = x.getNormalizedDistance();
        if (getParam("use_esf") >0)
        {
          if (findAndEraseCandidate(tmp_esf, x.getName(), d_tmp))
            x_dist += d_tmp;
          else
            x_dist += 1;
        }
        if (getParam("use_cvfh")>0)
        {
          if (findAndEraseCandidate(tmp_cvfh, x.getName(), d_tmp))
            x_dist += d_tmp;
          else
            x_dist += 1;
        }
        if (getParam("use_ourcvfh")>0)
        {
          if (findAndEraseCandidate(tmp_ourcvfh, x.getName(), d_tmp))
            x_dist += d_tmp;
          else
            x_dist += 1;
        }
        x.setNormalizedDistance (x_dist / feature_count_);
        composite_list.push_back(x);
      }
    }
    //Still some candidates in ESF that were not in VFH
    if (getParam("use_esf")>0 && !tmp_esf.empty())
    {
      for (auto& x : tmp_esf)
      {
        float d_tmp;
        float x_dist = x.getNormalizedDistance();
        if (getParam("use_cvfh")>0)
        {
          if (findAndEraseCandidate(tmp_cvfh, x.getName(), d_tmp))
            x_dist += d_tmp;
          else
            x_dist += 1;
        }
        if (getParam("use_ourcvfh")>0)
        {
          if (findAndEraseCandidate(tmp_ourcvfh, x.getName(), d_tmp))
            x_dist += d_tmp;
          else
            x_dist += 1;
        }
        if (getParam("use_vfh")>0)
          x_dist += 1;
        x.setNormalizedDistance (x_dist / feature_count_);
        composite_list.push_back(x);
      }
    }
    //Still some candidates in CVFH that were not in VFH and ESF
    if (getParam("use_cvfh")>0 && !tmp_cvfh.empty())
    {
      for (auto& x: tmp_cvfh)
      {
        float d_tmp;
        float x_dist = x.getNormalizedDistance();
        if (getParam("use_ourcvfh")>0)
        {
          if (findAndEraseCandidate(tmp_ourcvfh, x.getName(), d_tmp))
            x_dist += d_tmp;
          else
            x_dist += 1;
        }
        if (getParam("use_vfh")>0)
          x_dist += 1;
        if (getParam("use_esf")>0)
          x_dist += 1;
        x.setNormalizedDistance(x_dist / feature_count_);
        composite_list.push_back(x);
      }
    }
    //Still some candidates in OURCVFH that were not in other lists
    if (getParam("use_ourcvfh")>0 && !tmp_ourcvfh.empty())
    {
      for (auto& x:tmp_ourcvfh)
      {
        float x_dist = x.getNormalizedDistance();
        if (getParam("use_vfh")>0)
          x_dist += 1;
        if (getParam("use_esf")>0)
          x_dist += 1;
        if (getParam("use_cvfh")>0)
          x_dist += 1;
        x.setNormalizedDistance(x_dist / feature_count_);
        composite_list.push_back(x);
      }
    }
    sortListByNormalizedDistance(ListType::composite);
    composite_list.resize(k);
    for (std::vector<Candidate>::iterator it=composite_list.begin(); it!=composite_list.end(); ++it)
      it->setRank(it - composite_list.begin() +1); //write the rank of the candidate in the list
    if (verbosity>1)
    {
      print_value("%g",t.getTime());
      print_info(" ms elapsed\n");
      print_info("%*s]\tTotal time elapsed to generate list(s) of candidates: ",20,__func__);
      print_value("%g",timer.getTime());
      print_info(" ms\n");
    }
    return true;
  }

  bool
  PoseEstimationBase::initTarget()
  {
    if (target_cloud)
    {
      if (target_cloud->empty())
      {
        print_error("%*s]\tError initializing a Target, passed cloud is empty!\n",20,__func__);
        return false;
      }
    }
    else
    {
      print_error("%*s]\tError initializing a Target, uninitialized cloud pointer!\n",20,__func__);
      return false;
    }
    if (getParam("filter")>0)
      removeOutliers();
    else
      copyPointCloud(*target_cloud, *target_cloud_processed);

    if (getParam("upsamp")>0)
      applyUpsampling();

    if (getParam("downsamp")>0)
      applyDownsampling();
    feature_count_ = 0;
    if (getParam("use_esf")>0)
    {
      ++feature_count_;
      computeESF();
    }
    if (getParam("use_vfh")>0 || getParam("use_cvfh")>0 || getParam("use_ourcvfh")>0)
    {
      computeNormals();
      if (getParam("use_vfh")>0)
      {
        ++feature_count_;
        computeVFH();
      }
      if (getParam("use_cvfh")>0)
      {
        ++feature_count_;
        computeCVFH();
      }
      if (getParam("use_ourcvfh")>0)
      {
        ++feature_count_;
        computeOURCVFH();
      }
    }
    if (feature_count_ <= 0)
    {
      print_error("%*s]\tError initializing a Target, zero features chosen to estimate. Enable at least one!\n",20,__func__);
      return false;
    }
    return true;
  }

  void
  PoseEstimationBase::removeOutliers()
  {
    pcl::StopWatch timer;
    if (getParam("verbosity") >1)
    {
      print_info("%*s]\tSetting Statistical Outlier Filter to preprocess target cloud...\n",20,__func__);
      print_info("%*s]\tSetting mean K to %g\n",20,__func__, getParam("filter_mean_k"));
      print_info("%*s]\tSetting Standard Deviation multiplier to %g\n",20,__func__, getParam("filter_std_dev_mul_thresh"));
      timer.reset();
    }
    PtC::Ptr filtered (new PtC);
    pcl::StatisticalOutlierRemoval<Pt> fil;
    fil.setMeanK (getParam("filter_mean_k"));
    fil.setStddevMulThresh (getParam("filter_std_dev_mul_thresh"));
    fil.setInputCloud(target_cloud);
    fil.filter(*filtered);
    copyPointCloud(*filtered, *target_cloud_processed);
    if (getParam("verbosity")>1)
    {
      print_info("%*s]\tTotal time elapsed during filter: ",20,__func__);
      print_value("%g", timer.getTime());
      print_info(" ms\n");
    }
  }

  void
  PoseEstimationBase::applyUpsampling()
  {
    pcl::StopWatch timer;
    float search_radius = getParam("upsamp_search_radius");
    int point_density = getParam("upsamp_point_density");
    std::string poly_fit_str = getParam("upsamp_poly_fit") ? "True" : "False";
    int poly_fit = getParam("upsamp_poly_fit");
    int poly_order = getParam("upsamp_poly_order");
    if (getParam("verbosity") >1)
    {
      print_info("%*s]\tSetting MLS with Random Uniform Density to preprocess target cloud...\n",20,__func__);
      print_info("%*s]\tSetting polynomial order to %d\n",20,__func__, poly_order);
      print_info("%*s]\tSetting polynomial fit to %s\n",20,__func__, poly_fit_str.c_str());
      print_info("%*s]\tSetting desired point density to %d\n",20,__func__, point_density);
      print_info("%*s]\tSetting search radius to %g\n",20,__func__, search_radius);
      timer.reset();
    }
    PtC::Ptr upsampled (new PtC);
    pcl::search::KdTree<Pt>::Ptr tree (new pcl::search::KdTree<Pt>);
    pcl::MovingLeastSquares<Pt, Pt> mls;
    mls.setInputCloud(target_cloud_processed);
    mls.setSearchMethod(tree);
    mls.setUpsamplingMethod (pcl::MovingLeastSquares<Pt, Pt>::RANDOM_UNIFORM_DENSITY);
    mls.setComputeNormals (false);
    mls.setPolynomialOrder(poly_order);
    mls.setPolynomialFit(poly_fit);
    mls.setSearchRadius(search_radius);
    mls.setPointDensity(point_density);
    mls.process(*upsampled);
    copyPointCloud(*upsampled, *target_cloud_processed);
    if (getParam("verbosity") >1)
    {
      print_info("%*s]\tTotal time elapsed during upsampling: ",20,__func__);
      print_value("%g", timer.getTime());
      print_info(" ms\n");
    }
  }

  void
  PoseEstimationBase::applyDownsampling()
  {
    pcl::StopWatch timer;
    float leaf_size = getParam("downsamp_leaf_size");
    if (getParam("verbosity") >1)
    {
      print_info("%*s]\tSetting Voxel Grid Filter to preprocess target cloud...\n",20,__func__);
      print_info("%*s]\tSetting Leaf Size to %g\n",20,__func__, leaf_size);
      timer.reset();
    }
    PtC::Ptr downsampled (new PtC);
    pcl::VoxelGrid<Pt> vg;
    vg.setInputCloud(target_cloud_processed);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.setDownsampleAllData (true);
    vg.filter(*downsampled);
    copyPointCloud(*downsampled, *target_cloud_processed);
    if (getParam("verbosity") >1)
    {
      print_info("%*s]\tTotal time elapsed during downsampling: ",20,__func__);
      print_value("%g", timer.getTime());
      print_info(" ms\n");
    }
  }

  void
  PoseEstimationBase::computeVFH()
  {
    pcl::StopWatch timer;
    if (getParam("verbosity")>1)
    {
      print_info("%*s]\tEstimating VFH feature of target...\n",20,__func__);
      timer.reset();
    }
    pcl::VFHEstimation<Pt, pcl::Normal, pcl::VFHSignature308> vfhE;
    pcl::search::KdTree<Pt>::Ptr tree (new pcl::search::KdTree<Pt>);
    vfhE.setSearchMethod(tree);
    vfhE.setInputCloud (target_cloud_processed);
    vfhE.setViewPoint (target_cloud_processed->sensor_origin_(0), target_cloud_processed->sensor_origin_(1), target_cloud_processed->sensor_origin_(2));
    vfhE.setInputNormals (target_normals.makeShared());
    vfhE.compute (target_vfh);
    if (getParam("verbosity")>1)
    {
      print_info("%*s]\tTotal time elapsed during VFH estimation: ",20,__func__);
      print_value("%g", timer.getTime());
      print_info(" ms\n");
    }
  }

  void
  PoseEstimationBase::computeESF()
  {
    pcl::StopWatch timer;
    if (getParam("verbosity")>1)
    {
      print_info("%*s]\tEstimating ESF feature of target...\n",20,__func__);
      timer.reset();
    }
    pcl::ESFEstimation<Pt, pcl::ESFSignature640> esfE;
    pcl::search::KdTree<Pt>::Ptr tree (new pcl::search::KdTree<Pt>);
    esfE.setSearchMethod(tree);
    esfE.setInputCloud (target_cloud_processed);
    esfE.compute (target_esf);
    if (getParam("verbosity")>1)
    {
      print_info("%*s]\tTotal time elapsed during ESF estimation: ",20,__func__);
      print_value("%g", timer.getTime());
      print_info(" ms\n");
    }
  }

  void
  PoseEstimationBase::computeCVFH()
  {
    pcl::StopWatch timer;
    float ang_thresh = getParam("cvfh_ang_thresh");
    float curv_thresh = getParam("cvfh_curv_thresh");
    float clus_tol = getParam("cvfh_clus_tol");
    int min_points = getParam("cvfh_clus_min_points");
    if (getParam("verbosity")>1)
    {
      print_info("%*s]\tEstimating CVFH feature of target...\n",20,__func__);
      print_info("%*s]\tUsing Angle Threshold of %g degress for normal deviation\n",20,__func__, ang_thresh);
      print_info("%*s]\tUsing Curvature Threshold of %g\n",20,__func__, curv_thresh);
      print_info("%*s]\tUsing Cluster Tolerance of %g\n",20,__func__, clus_tol);
      print_info("%*s]\tConsidering a minimum of %d points for a cluster\n",20,__func__, min_points);
      timer.reset();
    }
    pcl::CVFHEstimation<Pt, pcl::Normal, pcl::VFHSignature308> cvfhE;
    pcl::search::KdTree<Pt>::Ptr tree (new pcl::search::KdTree<Pt>);
    cvfhE.setSearchMethod(tree);
    cvfhE.setInputCloud (target_cloud_processed);
    cvfhE.setViewPoint (target_cloud_processed->sensor_origin_(0), target_cloud_processed->sensor_origin_(1), target_cloud_processed->sensor_origin_(2));
    cvfhE.setInputNormals (target_normals.makeShared());
    cvfhE.setEPSAngleThreshold(pcl::deg2rad(ang_thresh)); //angle needs to be supplied in radians
    cvfhE.setCurvatureThreshold(curv_thresh);
    cvfhE.setClusterTolerance(clus_tol);
    cvfhE.setMinPoints(min_points);
    cvfhE.setNormalizeBins(false);
    cvfhE.compute (target_cvfh);
    if (getParam("verbosity")>1)
    {
      print_info("%*s]\tTotal of %d clusters were found on query\n",20,__func__, target_cvfh.points.size());
      print_info("%*s]\tTotal time elapsed during CVFH estimation: ",20,__func__);
      print_value("%g", timer.getTime());
      print_info(" ms\n");
    }
  }

  void
  PoseEstimationBase::computeOURCVFH()
  {
    pcl::StopWatch timer;
    float ang_thresh = getParam("ourcvfh_ang_thresh");
    float curv_thresh = getParam("ourcvfh_curv_thresh");
    float clus_tol = getParam("ourcvfh_clus_tol");
    int min_points = getParam("ourcvfh_clus_min_points");
    float axis_ratio = getParam("ourcvfh_axis_ratio");
    float min_axis = getParam("ourcvfh_min_axis_value");
    float refine = getParam("ourcvfh_refine_clusters");
    if (getParam("verbosity")>1)
    {
      print_info("%*s]\tEstimating OURCVFH feature of target...\n",20,__func__);
      print_info("%*s]\tUsing Angle Threshold of %g degress for normal deviation\n",20,__func__,ang_thresh);
      print_info("%*s]\tUsing Curvature Threshold of %g\n",20,__func__,curv_thresh);
      print_info("%*s]\tUsing Cluster Tolerance of %g\n",20,__func__,clus_tol);
      print_info("%*s]\tConsidering a minimum of %d points for a cluster\n",20,__func__,min_points);
      print_info("%*s]\tUsing Axis Ratio of %g and Min Axis Value of %g during SGURF disambiguation\n",20,__func__,axis_ratio,min_axis);
      print_info("%*s]\tUsing Refinement Factor of %g for clusters\n",20,__func__,refine);
      timer.reset();
    }
    pcl::OURCVFHEstimation<Pt, pcl::Normal, pcl::VFHSignature308> ourcvfhE;
    pcl::search::KdTree<Pt>::Ptr tree (new pcl::search::KdTree<Pt>);
    //PointCloud<Pt>::Ptr cloud (new PointCloud<Pt>);
    //copyPointCloud(*query_cloud_, *cloud);
    ourcvfhE.setSearchMethod(tree);
    ourcvfhE.setInputCloud (target_cloud_processed);
    ourcvfhE.setViewPoint (target_cloud_processed->sensor_origin_(0), target_cloud_processed->sensor_origin_(1), target_cloud_processed->sensor_origin_(2));
    ourcvfhE.setInputNormals (target_normals.makeShared());
    ourcvfhE.setEPSAngleThreshold(pcl::deg2rad(ang_thresh)); //angle needs to be supplied in radians
    ourcvfhE.setCurvatureThreshold(curv_thresh);
    ourcvfhE.setClusterTolerance(clus_tol);
    ourcvfhE.setMinPoints(min_points);
    ourcvfhE.setAxisRatio(axis_ratio);
    ourcvfhE.setMinAxisValue(min_axis);
    ourcvfhE.setRefineClusters(refine);
    ourcvfhE.compute (target_ourcvfh);
    if (getParam("verbosity")>1)
    {
      print_info("%*s]\tTotal of %d clusters were found on target\n",20,__func__, target_ourcvfh.points.size());
      print_info("%*s]\tTotal time elapsed during OURCVFH estimation: ",20,__func__);
      print_value("%g", timer.getTime());
      print_info(" ms\n");
    }
  }

  void
  PoseEstimationBase::computeNormals()
  {
    pcl::StopWatch timer;
    float search_radius = getParam("normals_radius_search");
    if (getParam("verbosity")>1)
    {
      print_info("%*s]\tSetting normal estimation to calculate target normals...\n",20,__func__);
      print_info("%*s]\tSetting a neighborhood radius of %g\n",20,__func__, search_radius);
      timer.reset();
    }
    pcl::NormalEstimationOMP<Pt, pcl::Normal> ne;
    pcl::search::KdTree<Pt>::Ptr tree (new pcl::search::KdTree<Pt>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(search_radius);
    ne.setNumberOfThreads(0); //use pcl autoallocation
    ne.setInputCloud(target_cloud_processed);
    ne.useSensorOriginAsViewPoint();
    ne.compute(target_normals);
    if (getParam("verbosity")>1)
    {
      print_info("%*s]\tTotal time elapsed during normal estimation: ",20,__func__);
      print_value("%g", timer.getTime());
      print_info(" ms\n");
    }
  }

  bool
  PoseEstimationBase::setTarget(PtC::Ptr target, std::string name)
  {
    if (target)
    {
      target_cloud.reset(new PtC);
      target_cloud_processed.reset(new PtC);
      pcl::copyPointCloud(*target, *target_cloud);
      target_name = name;
    }
    if (getParam("verbosity")>1)
      print_info("%*s]\tSetting Target for Pose Estimation: %s with %d points.\n",20,__func__,name.c_str(),target_cloud->points.size());
    return (initTarget());
  }

  bool
  PoseEstimationBase::loadAndSetDatabase(boost::filesystem::path db_path)
  {
    DatabaseReader loader;
    *this = loader.load(db_path);
    return (!this->isEmpty());
  }

  bool
  PoseEstimationBase::setDatabase (const Database& db)
  {
    if (db.isEmpty())
    {
      print_error("%*s]\tPassed database looks empty, aborting...",20,__func__);
      return false;
    }
    *this = db;
    return (!this->isEmpty());
  }

  PoseEstimationBase&
  PoseEstimationBase::operator= (const Database& other)
  {
    int rows = other.getDatabaseVFH()->rows;
    int cols = other.getDatabaseVFH()->cols;
    histograms vfh (new float[rows * cols], rows, cols);
    for (size_t i=0; i<rows; ++i)
      for (size_t j=0; j<cols; ++j)
        vfh[i][j] = (*other.getDatabaseVFH())[i][j];
    rows = other.getDatabaseESF()->rows;
    cols = other.getDatabaseESF()->cols;
    histograms esf (new float[rows * cols], rows, cols);
    for (size_t i=0; i<rows; ++i)
      for (size_t j=0; j<cols; ++j)
        esf[i][j] = (*other.getDatabaseESF())[i][j];
    rows = other.getDatabaseCVFH()->rows;
    cols = other.getDatabaseCVFH()->cols;
    histograms cvfh (new float[rows * cols], rows, cols);
    for (size_t i=0; i<rows; ++i)
      for (size_t j=0; j<cols; ++j)
        cvfh[i][j] = (*other.getDatabaseCVFH())[i][j];
    rows = other.getDatabaseOURCVFH()->rows;
    cols = other.getDatabaseOURCVFH()->cols;
    histograms ourcvfh (new float[rows * cols], rows, cols);
    for (size_t i=0; i<rows; ++i)
      for (size_t j=0; j<cols; ++j)
        ourcvfh[i][j] = (*other.getDatabaseOURCVFH())[i][j];
    std::vector<std::string> names;
    boost::copy (other.getDatabaseNames(), back_inserter(names));
    std::vector<std::string> names_cvfh;
    boost::copy (other.getDatabaseNamesCVFH(), back_inserter(names_cvfh));
    std::vector<std::string> names_ourcvfh;
    boost::copy (other.getDatabaseNamesOURCVFH(), back_inserter(names_ourcvfh));
    std::vector<PtC> clouds;
    boost::copy (other.getDatabaseClouds(), back_inserter(clouds));
    //only way to copy FLANN indexs that i'm aware of (save it to disk then load it)
    other.getDatabaseIndexVFH()->save(".idx_v_tmp");
    indexVFH idx_vfh (vfh, SavedIndexParams(".idx_v_tmp"));
    boost::filesystem::remove(".idx_v_tmp"); //delete tmp file
    //only way to copy indexs that i'm aware of (save it to disk then load it)
    other.getDatabaseIndexESF()->save(".idx_e_tmp");
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
    this->db_path_ = other.getDatabasePath();
    return *this;
  }

  PoseEstimationBase&
  PoseEstimationBase::operator= (Database&& other)
  {
    this->vfh_ = std::move(other.getDatabaseVFH());
    this->esf_ = std::move(other.getDatabaseESF());
    this->cvfh_ = std::move(other.getDatabaseCVFH());
    this->ourcvfh_ = std::move(other.getDatabaseOURCVFH());
    this->names_ = std::move(other.getDatabaseNames());
    this->names_cvfh_ = std::move(other.getDatabaseNamesCVFH());
    this->names_ourcvfh_ = std::move(other.getDatabaseNamesOURCVFH());
    this->db_path_= std::move(other.getDatabasePath());
    this->clouds_ = std::move(other.getDatabaseClouds());
    this->vfh_idx_ = std::move(other.getDatabaseIndexVFH());
    this->esf_idx_ = std::move(other.getDatabaseIndexESF());
    return *this;
  }
} //End of namespace pel

