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

using namespace pcl::console;

namespace pel
{
  bool
  PoseEstimationBase::generateLists()
  {
    int k = getParam("lists_size");
    int verbosity = getParam("verbosity");
    if (isEmpty())
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
    if (k > names_.size())
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
  PoseEstimationBase::initTarget(std::string name, PtC::Ptr cloud)
  {
    target_name = name;
    if (cloud)
    {
      if (cloud->empty())
      {
        print_error("%*s]\tError initializing a Target, passed cloud is empty!\n",20,__func__);
        return false;
      }
      target_cloud = cloud;
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
//     ///Internal parameter to check if the target was succesfully set and its features estimated
//     bool target_set_;
//     ///Internal parameter to check if list(s) of candidates are successfully generated
//     bool candidates_found_;
//     ///Internal parameter to check if candidate refinement has been done and it was succesfull
//     bool refinement_done_;
//     ///Internal parameter to check if a database was loaded in memory and it's now ready to be used
//     bool db_set_;
//     ///Internal parameter to check if query was supplied in a local refernce system
//     bool local_query_;
//

//     #<{(|*\brief Searches a list for a candidate and eliminates it, saving its distance. Internal use
//      * \param[in] list The list to inspect and modifiy
//      * \param[in] name The name to search in list
//      * \param[out] dist The distace of Candidate found in list (normalized)
//      *
//      * Return true if the candidate is found on the list, false otherwise
//      |)}>#
//     bool findCandidate_(std::vector<Candidate>& list, std::string name, float& dist);
//
//     public:
//     ///Default Empty Constructor that sets default parameters (see them in config file "config/parameters.conf")
//     PoseEstimation();
//
//     #<{(|*\brief Constructor with path to a configuration file containing parameters to set.
//      *\param[in] config_file Path to a config file to use
//      *
//      * Configuration file must have extension .conf and follow certain naming conventions,
//      * look at example .conf file provided for more details, or look at \ref param.
//      * Note: This constructor uses C++11 functionality and will probably not compile without -std=c++11
//      * It delegates construction to empty contructor then calls initParams()
//      * It is the same way as calling empty constructor and then initParams() method
//      |)}>#
//     PoseEstimation(boost::filesystem::path config_file) : PoseEstimation() {this->initParams(config_file);}
//
//     #<{(|*\brief Set the Pose Estimation query (the object to be identified)
//      * \param[in] str The name the query should assume
//      * \param[in] cl  Point cloud containing only the object to be estimated (i.e. already segmented)
//      |)}>#
//     void setQuery (std::string str, PtC& cl);
//
//     #<{(|* \brief Set the Pose Estimation query (the object to be identified)
//      * \param[in] str the name the query should assume
//      * \param[in] clp Shared pointer containing only the pointcloud of the object to be estimated (i.e. already segmented)
//      |)}>#
//     void setQuery (std::string str, PtC::Ptr clp);
//
//     #<{(|* \brief Set a database of known poses to be used for pose estimation procedure.
//      * \param[in] dbPath The path to the directory containing the database
//      *
//      * Note that this method calls PoseDB::load() on the path specified
//      |)}>#
//     void setDatabase(boost::filesystem::path dbPath);
//     #<{(|* \brief Set a database of known poses to be used for pose estimation procedure.
//      * \param[in] database PoseDB object to use as database
//      |)}>#
//     void setDatabase(Database& database);
//
//     #<{(|* \brief Generates list(s) of candidates to the query using the database provided as argument
//      * \param[in] dbPath The path to the directory containing the database of poses, previously generated.
//      * \return _True_ if successful, _false_ otherwise
//      *
//      * Note that this method also calls setDatabase() on the path specified for future lists computations.
//      |)}>#
//     bool generateLists(boost::filesystem::path dbPath);
//
//     #<{(|* \brief Generates list(s) of candidates to the query using the database provided as argument
//      * \param[in] db The PoseDB object containing the database of poses, previously generated or loaded.
//      * \return _True_ if successful, _false_ otherwise
//      *
//      * Note that this method also calls setDatabase() for future lists computations.
//      |)}>#
//     bool generateLists(Database& db);
//
//     #<{(|* \brief Generates list(s) of candidates to the query using previously set database
//      * \return _True_ if successful, _false_ otherwise
//      |)}>#
//     bool generateLists();
//
//     #<{(|*\brief Start the refinement procedure with ICP to obtain a final candidate from the composite list
//      * \return _True_ if refinement is successful and one final candidates is obtained, _false_ otherwise
//      *
//      * Currently two methods for refinemente are implemented: Progressive Bisection (default) and Brute Force
//      * To chose Progressive Bisection set "progBisection" parameter to 1, to chose Brute Force set it to 0
//      - Brute Force:
//      1. Start align rank 1 candidate on composite list with ICP, until "maxIterations" (default 200) are reached or candidate RMSE falls below "rmseThreshold" (default 0.003)
//      2. Align all the candidates until one converges, that one is the final pose estimation
//      3. If no one converges the Pose Estimation will produce no answer. Set an higher "rmseThreshold" parameter
//      - Progressive Bisection:
//      1. Align all candidates on composite list with ICP, but performs at most "progItera" (default 5) iterations
//      2. Resort the list based on minimum RMSE of candidates
//      3. Discard a fraction of the list multiplying its size by "progFraction" (default 0.5), eliminating candidates with worst rmse
//      4. Repeat from step 1 until one candidates falls below the "rmseThreshold" (default 0.003) or only one candidate survives
//      |)}>#
//     bool refineCandidates();
//
//     #<{(|* \brief Undergo the whole process of pose estimation
//      *\param[in] name Name of the new query object to estimate
//      \param[in] cloud Point cloud containing the query object to estimate (segmented)
//      \param[in] db_path Path to a directory containing the database of poses to load
//      \return _True_ if pose estimation was successful in all its parts, _false_ otherwise
//
//      This method calls is sequence setQuery() setDatabase() generateLists() and refineCandidates(), using the
//      already set parameters
//      |)}>#
//     bool estimate(std::string name, PtC& cloud, boost::filesystem::path db_path);
//
//     #<{(|* \brief Undergo the whole process of pose estimation
//      *\param[in] name Name of the new query object to estimate
//      \param[in] cloud_pointer Pointer to a point cloud containing the query object to estimate (segmented)
//      \param[in] db_path Path to a directory containing the database of poses to load
//      \return _True_ if pose estimation was successful in all its parts, _false_ otherwise
//
//      This method calls is sequence setQuery() setDatabase() generateLists() and refineCandidates(), using the
//      already set parameters
//      |)}>#
//     bool estimate(std::string name, PtC::Ptr cloud_pointer, boost::filesystem::path db_path);
//
//     #<{(|* \brief Undergo the whole process of pose estimation
//      *\param[in] name Name of the new query object to estimate
//      \param[in] cloud Point cloud containing the query object to estimate (segmented)
//      \param[in] database PoseDB object to use as database
//      \return _True_ if pose estimation was successful in all its parts, _false_ otherwise
//
//      This method calls is sequence setQuery() setDatabase() generateLists() and refineCandidates(), using the
//      already set parameters
//      |)}>#
//     bool estimate(std::string name, PtC& cloud, Database& database);
//
//     #<{(|* \brief Undergo the whole process of pose estimation
//      *\param[in] name Name of the new query object to estimate
//      \param[in] cloud_pointer Pointer to a point cloud containing the query object to estimate (segmented)
//      \param[in] database PoseDB object to use as database
//      \return _True_ if pose estimation was successful in all its parts, _false_ otherwise
//
//      This method calls is sequence setQuery() setDatabase() generateLists() and refineCandidates(), using the
//      already set parameters
//      |)}>#
//     bool estimate(std::string name, PtC::Ptr cloud_pointer, Database& database);
//
//     #<{(|* \brief Print final estimation informations (such as name, distance, rmse and transformation) on screen
//     |)}>#
//     void printEstimation();
//
//     #<{(|*\brief Save final estimation informations (such as name, distance, rmse and transformation) on a file with path specified
//       \param[in] file Path to a location on disk, where the file will be created or renewed
//       \param[in] append Chose to append to the end of file (true)(default), or truncate its contents (false), if file does not exists this parameter has no effectinal
//       \return _True_ if file is correctly written, _False_ otherwise
//
//       The path can specify a directory or a file, if the former a new text file of name "query_name.estimation" will be created inside the specified directory.
//       If the latter, the specified file will be written.
//       Note on paths: if file path does not exists, a file must be specified with an extension, otherwise it will be treated as a directory.
//       For example "/foo/bar" or "/foo/bar/" will be both treated as directories if they don't exists already.
//       "foo/bar.f" will be treated as file "bar.f" inside relative directory "foo"
//       |)}>#
//     bool saveEstimation(boost::filesystem::path file, bool append = true);
//
//     #<{(|* \brief Get a pointer to a copy of final chosen Candidate, representing the pose estimation
//      * \return A pointer to a copy of the final Candidate
//      |)}>#
//     boost::shared_ptr<Candidate> getEstimation();
//
//     #<{(|* \brief Get a pointer to a copy of final chosen Candidate, representing the pose estimation
//      * \param[out] est A pointer to a copy of the final Candidate chosen by refinement
//      * \return _True_ if est correctly points to final pose estimation, _false_ otherwise
//      |)}>#
//     bool getEstimation(boost::shared_ptr<Candidate> est);
//
//     #<{(|* \brief Get a copy of final poseEstimation transformation
//      * \param[out] t A matrix that will contain the final transformation
//      * \return _True_ if t is correctly initialized, _false_ otherwise
//      |)}>#
//     bool getEstimationTransformation(Eigen::Matrix4f& t);
//
//     #<{(|* \brief Print some information on Query object on screen
//     |)}>#
//     void printQuery();
//
//     #<{(|* \brief Get name and  pointers holding point clouds of query object (before and after eventual preprocessing)
//      * \param[out] name String containing query name
//      * \param[out] clp Shared Pointer to a copy of query point cloud after eventual preprocessing
//      * \return _True_ if operation was successful, _false_ otherwise
//      *
//      * If no preprocessing was done (i.e. downsampling, upsampling and filtering parameters are all zero), clp and clp_pre point the same cloud
//      |)}>#
//     bool getQuery (std::string& name, PtC::Ptr clp);
//
//     #<{(|* \brief Get the estimated features (including normals) from the query
//      * \param[out] vfh Point cloud pointer that will hold the VFH feature
//      * \param[out] cvfh Point cloud pointer that will hold the CVFH feature
//      * \param[out] ourcvfh Point cloud pointer that will hold the OURCVFH feature
//      * \param[out] esf Point cloud pointer that will hold the ESF feature
//      * \param[out] normals Point cloud pointer that will hold the feature normals
//      * \return How many features were currently copied, or (-1) in case of errors
//      *
//      * Note that some features may not be available for the current query, because they were not estimated, check parameter settings
//      |)}>#
//     int getQueryFeatures(pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh, pcl::PointCloud<pcl::VFHSignature308>::Ptr cvfh, pcl::PointCloud<pcl::VFHSignature308>::Ptr ourcvfh, pcl::PointCloud<pcl::ESFSignature640>::Ptr esf, pcl::PointCloud<pcl::Normal>::Ptr normals);
//
//     #<{(|* \brief Visualize the query into a PCL visualizer, this method blocks execution until the visualizer is closed. Query cloud is displayed after preprocessing
//     |)}>#
//     void viewQuery();
//
//     #<{(|* \brief Visualize Pose Estimation into a PCL visualizer, this method blocks execution until the visualizer is closed.
//      *
//      * Query point cloud is displayed and on top of it the aligned candidate in green color. Other informations are displayed in the visualizer window
//      |)}>#
//     void viewEstimation();
//
//     #<{(|* \brief Register pose estimation results into a file for testing purpose
//      *  \param[in] file path to a txt file to write into, if file already exists it will be appended.
//      *  \param[in] gt ground truth reference to check correctness, must be in form <obj>_<lat>_<long>.
//      *  \return _True_ if operation is succesful, _false_ otherwise.
//      *
//      *  Writes into a file pose estimation results for testing purpose. Given the final pose estimation, it saves the ranks of each list
//      *  on which ground truth was found. It also saves if the chosen final candidate matches the ground truth or not. For example a line could be like this:
//      *  \code
//      *  #VFH  #ESF  #CVFH #OURCVFH  #COMP   #FINAL
//      *  2     1     6      4        1       1
//      *  \code
//      *  In this example the ground truth was found on rank 2 in VFH list, rank 1 in ESF, etc...
//      *  Plus the final candidate chosen was correct (1), because it matches the ground truth name.
//      *  Final candidate can be:
//      *    + 0: If ground truth and pose estimation match completely (success)
//      *    + 1: If ground truth and pose estimation match with a tolerance of 10 degrees in latitude and/or longitude (direct neighbor)
//      *    + 2: If ground truth and pose estimation match only in names, hence orientation is wrong, but object was correctly identified (same object)
//      *    + 3: If ground truth and pose estimation do not match, hence the estimation was wrong (false)
//      |)}>#
//     bool saveTestResults(boost::filesystem::path file, std::string gt);
//
//     #<{(|* \brief Register pose estimation results into a file for testing purpose
//      *  \param[in] file path to a txt file to write into, if file already exists it will be appended.
//      *  \return _True_ if operation is succesful, _false_ otherwise.
//      *
//      *  Writes into a file pose estimation results for testing purpose. Given the final pose estimation, it saves the ranks of each list
//      *  on which ground truth was found, ground truth used is query name, that must follow naming convention <obj>_<lat>_<long>. It also saves if the chosen final candidate matches the ground truth or not. For example a line could be like this:
//      *  \code
//      *  #VFH  #ESF  #CVFH #OURCVFH  #COMP   #FINAL
//      *  2     1     6      4        1       1
//      *  \code
//      *  In this example the ground truth was found on rank 2 in VFH list, rank 1 in ESF, etc...
//      *  Plus the final candidate chosen was correct (1), because it matches the ground truth name.
//      *  Final candidate can be:
//      *    + 0: If ground truth and pose estimation match completely (success)
//      *    + 1: If ground truth and pose estimation match with a tolerance of 10 degrees in latitude and/or longitude (direct neighbor)
//      *    + 2: If ground truth and pose estimation match only in names, hence orientation is wrong, but object was correctly identified (same object)
//      *    + 3: If ground truth and pose estimation do not match, hence the estimation was wrong (false)
//      |)}>#
//     bool saveTestResults(boost::filesystem::path file);
//
//     #<{(|* \brief Elaborate test results written with saveTestResults()
//      * \param[in] file test file written by saveTestResults()
//      * \param[in] result path to a file in which write results
//      * \return _True_ if elaboration is successful, _false_ otherwise
//      *
//      * Result file will be truncated, if it already exists
//      |)}>#
//     bool elaborateTests(boost::filesystem::path file, boost::filesystem::path result);
//   };
} //End of namespace pel

