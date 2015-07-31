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

using namespace pcl::console;

namespace pel
{
  //TODO rework
  Database&
  DatabaseCreator::create (boost::filesystem::path path_clouds)
  {
    //Check params correctness
    fixParameters();
    //Start database creation
    /*
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
      PnC::Ptr input (new PnC);
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
        PnC::Ptr output (new PnC);
        split (vst, it->string(), boost::is_any_of("../\\"), boost::token_compress_on);
        names_.push_back(vst.at(vst.size()-2)); //filename without extension and path
        if (params->at("filtering") >0)
        {
          StatisticalOutlierRemoval<PnT> filter;
          filter.setMeanK ( params->at("filterMeanK") );
          filter.setStddevMulThresh ( params->at("filterStdDevMulThresh") );
          filter.setInputCloud(input);
          filter.filter(*output); //Process Filtering
          copyPointCloud(*output, *input);
        }
        if (params->at("upsampling") >0)
        {
          MovingLeastSquares<PnT, PnT> mls;
          search::KdTree<PnT>::Ptr tree (new search::KdTree<PnT>);
          mls.setInputCloud (input);
          mls.setSearchMethod (tree);
          mls.setUpsamplingMethod (MovingLeastSquares<PnT, PnT>::RANDOM_UNIFORM_DENSITY);
          mls.setComputeNormals (false);
          mls.setPolynomialOrder ( params->at("mlsPolyOrder") );
          mls.setPolynomialFit ( params->at("mlsPolyFit") );
          mls.setSearchRadius ( params->at("mlsSearchRadius") );
          mls.setPointDensity( params->at("mlsPointDensity") );
          mls.process (*output); //Process Upsampling
          copyPointCloud(*output, *input);
        }
        if (params->at("downsampling") >0)
        {
          VoxelGrid <PnT> vgrid;
          vgrid.setInputCloud (input);
          vgrid.setLeafSize ( params->at("vgridLeafSize"), params->at("vgridLeafSize"), params->at("vgridLeafSize"));
          vgrid.setDownsampleAllData (true);
          vgrid.filter (*output); //Process Downsampling
          copyPointCloud(*output, *input);
        }
        clouds_.push_back(*input); //store processed cloud
        Eigen::Vector3f s_orig (input->sensor_origin_(0), input->sensor_origin_(1), input->sensor_origin_(2) );
        Eigen::Quaternionf s_orie = input->sensor_orientation_;
        if (it == pvec.begin())
        {
          //check if clouds are in local or sensor ref frame
          if (s_orig.isZero(1e-5) && s_orie.isApprox(Eigen::Quaternionf::Identity(), 1e-5) )
          {
            this->clouds_in_local_ = false;
          }
          else
          {
            this->clouds_in_local_ = true;
          }
        }
        else
        {
          if (s_orig.isZero(1e-5) && s_orie.isApprox(Eigen::Quaternionf::Identity(), 1e-5) )
          {
            if (this->clouds_in_local_ == true)
              print_warn("%*s]\tLoaded PCD file %s, has sensor reference frame, while others found before were expressed in local object reference frame. Make sure you are loading the correct files!!",20,__func__,vst.at(vst.size()-2).c_str());
          }
          else
          {
            if (this->clouds_in_local_ == false)
              print_warn("%*s]\tLoaded PCD file %s, has local object reference frame, while others found before were expressed in sensor reference frame. Make sure you are loading the correct files!!",20,__func__,vst.at(vst.size()-2).c_str());
          }
        }
        if (this->clouds_in_local_)
        {
          input->sensor_origin_.setZero();
          input->sensor_orientation_.setIdentity();
          transformPointCloud(*input, *output, s_orig, s_orie); //TODO
          copyPointCloud(*output, *input);
        }
        //Normals computation
        NormalEstimationOMP<PnT, Normal> ne;
        search::KdTree<PnT>::Ptr tree (new search::KdTree<PnT>);
        PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(params->at("neRadiusSearch"));
        ne.setNumberOfThreads(0); //use pcl autoallocation
        ne.setInputCloud(input);
        ne.useSensorOriginAsViewPoint(); //use sensor origin stored inside point cloud as viewpoint, assume user has correctly set it
        ne.compute(*normals);
        //VFH
        VFHEstimation<PnT, Normal, VFHSignature308> vfhE;
        PointCloud<VFHSignature308> out;
        vfhE.setSearchMethod(tree);
        vfhE.setInputCloud (input);
        vfhE.setViewPoint (0,0,0);
        vfhE.setInputNormals (normals);
        vfhE.compute (out);
        tmp_vfh->push_back(out.points[0]);
        //ESF
        ESFEstimation<PnT, ESFSignature640> esfE;
        PointCloud<ESFSignature640> out_esf;
        esfE.setSearchMethod(tree);
        esfE.setInputCloud (input);
        esfE.compute (out_esf);
        tmp_esf->push_back(out_esf.points[0]);
        //CVFH
        CVFHEstimation<PnT, Normal, VFHSignature308> cvfhE;
        cvfhE.setSearchMethod(tree);
        cvfhE.setInputCloud (input);
        cvfhE.setViewPoint (0, 0, 0);
        cvfhE.setInputNormals (normals);
        cvfhE.setEPSAngleThreshold(params->at("cvfhEPSAngThresh")*D2R); //angle needs to be supplied in radians
        cvfhE.setCurvatureThreshold(params->at("cvfhCurvThresh"));
        cvfhE.setClusterTolerance(params->at("cvfhClustTol"));
        cvfhE.setMinPoints(params->at("cvfhMinPoints"));
        cvfhE.setNormalizeBins(false);
        cvfhE.compute (out);
        for (size_t n=0; n<out.points.size(); ++n)
        {
          names_cvfh_.push_back(names_[i]);
          tmp_cvfh->push_back(out.points[n]);
        }
        //OURCVFH
        OURCVFHEstimation<PnT, Normal, VFHSignature308> ourcvfhE;
        search::KdTree<PnT>::Ptr tree2 (new search::KdTree<PnT>);
        PointCloud<PnT>::Ptr input2 (new PointCloud<PnT>);
        copyPointCloud(*input, *input2);
        ourcvfhE.setSearchMethod(tree2);
        ourcvfhE.setInputCloud (input2);
        ourcvfhE.setViewPoint (0,0,0);
        ourcvfhE.setInputNormals (normals);
        ourcvfhE.setEPSAngleThreshold(params->at("ourcvfhEPSAngThresh")*D2R); //angle needs to be supplied in radians
        ourcvfhE.setCurvatureThreshold(params->at("ourcvfhCurvThresh"));
        ourcvfhE.setClusterTolerance(params->at("ourcvfhClustTol"));
        ourcvfhE.setMinPoints(params->at("ourcvfhMinPoints"));
        ourcvfhE.setAxisRatio(params->at("ourcvfhAxisRatio"));
        ourcvfhE.setMinAxisValue(params->at("ourcvfhMinAxisValue"));
        ourcvfhE.setRefineClusters(params->at("ourcvfhRefineClusters"));
        ourcvfhE.compute (out);
        for (size_t n=0; n<out.points.size(); ++n)
        {
          tmp_ourcvfh->push_back(out.points[n]);
          names_ourcvfh_.push_back(names_[i]);
        }
        print_info("%*s]\t%d clouds processed so far...\r",20,__func__,i+1);
        cout<<std::flush;
      }
      cout<<endl;
      if (tmp_vfh->points.size() == 0) //no clouds loaded
      {
        print_error("%*s]\tNo Histograms created, something went wrong, exiting...\n",20,__func__);
        return false;
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
      return true;
    }
    else
    {
      print_error("%*s]\t%s is not a valid directory...\n",20,__func__,pathClouds.string().c_str());
      return false;
    }
  }
  //TODO rework
  bool
  Database::create (boost::filesystem::path pathClouds)
  {
    parameters par;
    //setting default parameters for db creation, only relevant ones are created
    par["vgridLeafSize"]=0.005f;
    par["upsampling"]=par["filtering"]=0;
    par["downsampling"]=1;
    par["mlsPolyOrder"]=2;
    par["mlsPointDensity"]=200;
    par["mlsPolyFit"]=1;
    par["mlsSearchRadius"]=0.05f;
    par["filterMeanK"]=50;
    par["filterStdDevMulThresh"]=3;
    par["neRadiusSearch"]=0.02;
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
    return ( this->create(pathClouds, p) );
  */
  }
}//End of namespace
