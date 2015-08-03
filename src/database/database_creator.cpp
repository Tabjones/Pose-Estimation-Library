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
#include <pel/database/database_creator.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/features/vfh.h>
#include <pcl/features/esf.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace pcl::console;

namespace pel
{
  Database
  DatabaseCreator::create (boost::filesystem::path path_clouds)
  {
    //Check params correctness
    fixParameters();
    Database created;
    //Start database creation
    if (boost::filesystem::exists(path_clouds) && boost::filesystem::is_directory(path_clouds))
    {
      std::vector<boost::filesystem::path> pvec;
      copy(boost::filesystem::directory_iterator(path_clouds), boost::filesystem::directory_iterator(), back_inserter(pvec));
      sort(pvec.begin(), pvec.end());
      pcl::PointCloud<pcl::VFHSignature308>::Ptr tmp_vfh (new pcl::PointCloud<pcl::VFHSignature308>);
      pcl::PointCloud<pcl::VFHSignature308>::Ptr tmp_cvfh (new pcl::PointCloud<pcl::VFHSignature308>);
      pcl::PointCloud<pcl::VFHSignature308>::Ptr tmp_ourcvfh (new pcl::PointCloud<pcl::VFHSignature308>);
      pcl::PointCloud<pcl::ESFSignature640>::Ptr tmp_esf (new pcl::PointCloud<pcl::ESFSignature640>);
      PtC::Ptr input (new PtC);
      int i(0);
      for (const auto& file : pvec)
      {
        if (is_regular_file (file) && file.extension() == ".pcd")
        {
          if (pcl::io::loadPCDFile(file.c_str(), *input)!=0 ) //loadPCDFile returns 0 if success
          {
            print_warn("%*s]\tError Loading Cloud %s, skipping...\n",20,__func__,file.c_str());
            continue;
          }
        }
        else
        {
          print_warn("%*s]\tLoaded File (%s) is not a pcd, skipping...\n",20,__func__,file.c_str());
          continue;
        }
        std::vector<std::string> vst;
        PtC::Ptr output (new PtC);
        boost::split (vst, file.string(), boost::is_any_of("../\\"), boost::token_compress_on);
        created.names_.push_back(vst.at(vst.size()-2)); //filename without extension and path
        if (this->getParam("filtering") >0)
        {
          pcl::StatisticalOutlierRemoval<Pt> filter;
          filter.setMeanK ( this->getParam("filter_mean_k") );
          filter.setStddevMulThresh ( this->getParam("filter_std_dev_mul_thresh") );
          filter.setInputCloud(input);
          filter.filter(*output); //Process Filtering
          pcl::copyPointCloud(*output, *input);
        }
        if (this->getParam("upsamp") >0)
        {
          pcl::MovingLeastSquares<Pt, Pt> mls;
          pcl::search::KdTree<Pt>::Ptr tree (new pcl::search::KdTree<Pt>);
          mls.setInputCloud (input);
          mls.setSearchMethod (tree);
          mls.setUpsamplingMethod (pcl::MovingLeastSquares<Pt, Pt>::RANDOM_UNIFORM_DENSITY);
          mls.setComputeNormals (false);
          mls.setPolynomialOrder ( this->getParam("upsamp_poly_order") );
          mls.setPolynomialFit ( this->getParam("upsamp_poly_fit") );
          mls.setSearchRadius ( this->getParam("upsamp_search_radius") );
          mls.setPointDensity( this->getParam("upsamp_point_density") );
          mls.process (*output); //Process Upsampling
          copyPointCloud(*output, *input);
        }
        if (this->getParam("downsamp") >0)
        {
          pcl::VoxelGrid <Pt> vgrid;
          vgrid.setInputCloud (input);
          float leaf = this->getParam("downsamp_leaf_size");
          vgrid.setLeafSize (leaf, leaf, leaf);
          vgrid.setDownsampleAllData (true);
          vgrid.filter (*output); //Process Downsampling
          copyPointCloud(*output, *input);
        }
        created.clouds_.push_back(*input); //store processed cloud
        Eigen::Vector3f s_orig (input->sensor_origin_(0), input->sensor_origin_(1), input->sensor_origin_(2) );
        Eigen::Quaternionf s_orie = input->sensor_orientation_;
        input->sensor_origin_.setZero();
        input->sensor_orientation_.setIdentity();
        pcl::transformPointCloud(*input, *output, s_orig, s_orie);
        pcl::copyPointCloud(*output, *input);
        //Normals computation
        pcl::NormalEstimationOMP<Pt, pcl::Normal> ne;
        pcl::search::KdTree<Pt>::Ptr tree (new pcl::search::KdTree<Pt>);
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(this->getParam("normals_radius_search"));
        ne.setNumberOfThreads(0); //use pcl autoallocation
        ne.setInputCloud(input);
        //Use sensor origin stored inside point cloud as viewpoint, should be zero.
        //Because cloud now is in viewpoint reference frame
        ne.useSensorOriginAsViewPoint();
        ne.compute(*normals);
        //VFH
        pcl::VFHEstimation<Pt, pcl::Normal, pcl::VFHSignature308> vfhE;
        pcl::PointCloud<pcl::VFHSignature308> out;
        vfhE.setSearchMethod(tree);
        vfhE.setInputCloud (input);
        vfhE.setViewPoint (0,0,0);
        vfhE.setInputNormals (normals);
        vfhE.compute (out);
        tmp_vfh->push_back(out.points[0]);
        //ESF
        pcl::ESFEstimation<Pt, pcl::ESFSignature640> esfE;
        pcl::PointCloud<pcl::ESFSignature640> out_esf;
        esfE.setSearchMethod(tree);
        esfE.setInputCloud (input);
        esfE.compute (out_esf);
        tmp_esf->push_back(out_esf.points[0]);
        //CVFH
        pcl::CVFHEstimation<Pt, pcl::Normal, pcl::VFHSignature308> cvfhE;
        cvfhE.setSearchMethod(tree);
        cvfhE.setInputCloud (input);
        cvfhE.setViewPoint (0, 0, 0);
        cvfhE.setInputNormals (normals);
        //angle needs to be supplied in radians
        cvfhE.setEPSAngleThreshold(pcl::deg2rad(this->getParam("cvfh_ang_thresh")));
        cvfhE.setCurvatureThreshold(this->getParam("cvfh_curv_thresh"));
        cvfhE.setClusterTolerance(this->getParam("cvfh_clus_tol"));
        cvfhE.setMinPoints(this->getParam("cvfh_clus_min_points"));
        cvfhE.setNormalizeBins(false);
        cvfhE.compute (out);
        for (size_t n=0; n<out.points.size(); ++n)
        {
          created.names_cvfh_.push_back(created.names_[i]);
          tmp_cvfh->push_back(out.points[n]);
        }
        //OURCVFH
        pcl::OURCVFHEstimation<Pt, pcl::Normal, pcl::VFHSignature308> ourcvfhE;
        pcl::search::KdTree<Pt>::Ptr tree2 (new pcl::search::KdTree<Pt>);
        pcl::PointCloud<Pt>::Ptr input2 (new pcl::PointCloud<Pt>);
        copyPointCloud(*input, *input2);
        ourcvfhE.setSearchMethod(tree2);
        ourcvfhE.setInputCloud (input2);
        ourcvfhE.setViewPoint (0,0,0);
        ourcvfhE.setInputNormals (normals);
        ourcvfhE.setEPSAngleThreshold(pcl::deg2rad(this->getParam("ourcvfh_ang_thresh")));
        ourcvfhE.setCurvatureThreshold(this->getParam("ourcvfh_curv_thresh"));
        ourcvfhE.setClusterTolerance(this->getParam("ourcvfh_clus_tol"));
        ourcvfhE.setMinPoints(this->getParam("ourcvf_clus_min_points"));
        ourcvfhE.setAxisRatio(this->getParam("ourcvfh_axis_ratio"));
        ourcvfhE.setMinAxisValue(this->getParam("ourcvfh_min_axis_value"));
        ourcvfhE.setRefineClusters(this->getParam("ourcvfh_refine_clusters"));
        ourcvfhE.compute (out);
        for (size_t n=0; n<out.points.size(); ++n)
        {
          tmp_ourcvfh->push_back(out.points[n]);
          created.names_ourcvfh_.push_back(created.names_[i]);
        }
        print_info("%*s]\t%d clouds processed so far...\r",20,__func__,i+1);
        std::cout<<std::flush;
        ++i;
      }
      std::cout<<std::endl;
      if (tmp_vfh->points.size() == 0) //no clouds loaded
      {
        print_error("%*s]\tNo Histograms created, something went wrong, exiting...\n",20,__func__);
        return (created);
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
      created.vfh_ = boost::make_shared<histograms>(vfh);
      created.esf_ = boost::make_shared<histograms>(esf);
      created.cvfh_ = boost::make_shared<histograms>(cvfh);
      created.ourcvfh_ = boost::make_shared<histograms>(ourcvfh);
      //and indices
      indexVFH vfh_idx (*created.vfh_, flann::KDTreeIndexParams(4));
      created.vfh_idx_ = boost::make_shared<indexVFH>(vfh_idx);
      created.vfh_idx_->buildIndex();
      indexESF esf_idx (*created.esf_, flann::KDTreeIndexParams(4));
      created.esf_idx_ = boost::make_shared<indexESF>(esf_idx);
      created.esf_idx_->buildIndex();
      print_info("%*s]\tDone creating database, total of %d poses stored in memory\n",20,__func__,created.names_.size());
      return (created);
    }
    else
    {
      print_error("%*s]\t%s is not a valid directory...\n",20,__func__,path_clouds.c_str());
      return created;
    }
  }
}//End of namespace
