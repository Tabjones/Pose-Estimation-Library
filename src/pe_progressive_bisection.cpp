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

#include <pel/pe_progressive_bisection.h>

namespace pel
{
  namespace interface
  {
    /*

       bool PoseEstimation::refineCandidates()
       {
       if (!candidates_found_)
       {
       print_error("%*s]\tList of Candidates are not yet generated, call generateLists first...\n",20,__func__);
       return false;
       }
       if (local_query_)
       print_error("%*s]\tQuery in local reference system in not implemented yet, set another query or transform it, exiting...\n",20,__func__); //TODO implement this
       CentroidPoint<Pt> qct;
       for (int i=0; i<query_cloud_->points.size(); ++i)
       qct.add(query_cloud_->points[i]);
       Pt query_centroid;
       qct.get(query_centroid);
       if (params_["progBisection"]>0)
       {
    //ProgressiveBisection
    if (params_["verbosity"]>1)
    print_info("%*s]\tStarting Progressive Bisection...\n",20,__func__);
    StopWatch timer;
    timer.reset();
    vector<Candidate> list; //make a temporary list to manipulate
    boost::copy(composite_list_, back_inserter(list));
    IterativeClosestPoint<Pt, Pt, float> icp;
    icp.setInputTarget(query_cloud_); //query
    icp.setUseReciprocalCorrespondences(params_["icpReciprocal"]);
    icp.setMaximumIterations (params_["progItera"]); //max iterations to perform
    icp.setTransformationEpsilon (1e-9); //difference between consecutive transformations
    icp.setEuclideanFitnessEpsilon (1e-9); //not using it (sum of euclidean distances between points)
    int steps (0);
    while (list.size() > 1)
    {
    for (vector<Candidate>::iterator it=list.begin(); it!=list.end(); ++it)
    {
    PtC::Ptr aligned (new PtC);
    PtC::Ptr candidate (new PtC);
    copyPointCloud(*(it->cloud_), *candidate);
    candidate->sensor_origin_.setZero();
    candidate->sensor_orientation_.setIdentity();
    //icp align source over target, result in aligned
    icp.setInputSource(candidate); //the candidate
    Eigen::Matrix4f guess;
    if (steps >0)
    guess = it->transformation_;
    else
    {
    Eigen::Matrix4f T_kli, T_cen;
    CentroidPoint<Pt> cct;
    Pt candidate_centroid;
    if (this->database_.isLocal())
    { //database is in local frame
    Eigen::Matrix3f R (it->cloud_->sensor_orientation_);
    Eigen::Vector4f t;
    t = it->cloud_->sensor_origin_;
    T_kli << R(0,0), R(0,1), R(0,2), t(0),
    R(1,0), R(1,1), R(1,2), t(1),
    R(2,0), R(2,1), R(2,2), t(2),
    0,      0,      0,      1;
    PtC::Ptr cloud_in_k (new PtC);
    transformPointCloud(*candidate, *cloud_in_k, T_kli);
    for (int i=0; i< cloud_in_k->points.size(); ++i)
    cct.add(cloud_in_k->points[i]);
    cct.get(candidate_centroid);
    T_cen << 1,0,0, (query_centroid.x - candidate_centroid.x),
    0,1,0, (query_centroid.y - candidate_centroid.y),
    0,0,1, (query_centroid.z - candidate_centroid.z),
    0,0,0, 1;
    guess = T_cen*T_kli;
    }
    else
    { //database is in sensor frame, just get centroid
      for (int i=0; i< it->cloud_->points.size(); ++i)
        cct.add(it->cloud_->points[i]);
      cct.get(candidate_centroid);
      T_cen << 1,0,0, (query_centroid.x - candidate_centroid.x),
            0,1,0, (query_centroid.y - candidate_centroid.y),
            0,0,1, (query_centroid.z - candidate_centroid.z),
            0,0,0, 1;
      guess = T_cen;
    }
  }
  icp.align(*aligned, guess); //initial gross estimation
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
  ++steps;
  //now resort list
  sort(list.begin(), list.end(),
      [](Candidate const &a, Candidate const &b)
      {
      return (a.rmse_ < b.rmse_ );
      });
  //check if candidate falled under rmse threshold, no need to check them all since list is now sorted with min rmse on top
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
    return true;
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
  return true;
  }
  else
  {
    //BruteForce
    if (params_["verbosity"]>1)
      print_info("%*s]\tStarting Brute Force...\n",20,__func__);
    StopWatch timer;
    timer.reset();
    IterativeClosestPoint<Pt, Pt, float> icp;
    icp.setInputTarget(query_cloud_); //query
    icp.setUseReciprocalCorrespondences(params_["icpReciprocal"]);
    icp.setMaximumIterations (params_["maxIterations"]); //max iterations to perform
    icp.setTransformationEpsilon (1e-9); //difference between consecutive transformations
    icp.setEuclideanFitnessEpsilon (pow(params_["rmseThreshold"],2));
    for (vector<Candidate>::iterator it=composite_list_.begin(); it!=composite_list_.end(); ++it)
    {
      PtC::Ptr aligned (new PtC);
      PtC::Ptr candidate (new PtC);
      copyPointCloud(*(it->cloud_), *candidate);
      //icp align source over target, result in aligned
      candidate->sensor_origin_.setZero();
      candidate->sensor_orientation_.setIdentity();
      icp.setInputSource(candidate); //the candidate
      Eigen::Matrix4f T_kli, T_cen, guess;
      CentroidPoint<Pt> cct;
      Pt candidate_centroid;
      if (this->database_.isLocal())
      { //database is in local frame
        Eigen::Matrix3f R( it->cloud_->sensor_orientation_ );
        Eigen::Vector4f t;
        t = it->cloud_->sensor_origin_;
        T_kli << R(0,0), R(0,1), R(0,2), t(0),
              R(1,0), R(1,1), R(1,2), t(1),
              R(2,0), R(2,1), R(2,2), t(2),
              0,      0,      0,      1;
        PtC::Ptr cloud_in_k (new PtC);
        transformPointCloud(*(it->cloud_), *cloud_in_k, T_kli);
        for (int i=0; i< cloud_in_k->points.size(); ++i)
          cct.add(cloud_in_k->points[i]);
        cct.get(candidate_centroid);
        T_cen << 1,0,0, (query_centroid.x - candidate_centroid.x),
              0,1,0, (query_centroid.y - candidate_centroid.y),
              0,0,1, (query_centroid.z - candidate_centroid.z),
              0,0,0, 1;
        guess = T_cen*T_kli;
      }
      else
      { //database is in sensor frame, just get centroid
        for (int i=0; i< it->cloud_->points.size(); ++i)
          cct.add(it->cloud_->points[i]);
        cct.get(candidate_centroid);
        T_cen << 1,0,0, (query_centroid.x - candidate_centroid.x),
              0,1,0, (query_centroid.y - candidate_centroid.y),
              0,0,1, (query_centroid.z - candidate_centroid.z),
              0,0,0, 1;
        guess = T_cen;
      }
      icp.align(*aligned, guess);
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
        return true;
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
    return false;
  }
  }

  bool PoseEstimation::estimate(string name, PtC& cloud, boost::filesystem::path db_path)
  {
    query_set_=candidates_found_=db_set_=refinement_done_ =false;
    setQuery(name, cloud);
    if (generateLists(db_path))
      return (refineCandidates());
    else
      return false;
  }
  bool PoseEstimation::estimate(string name, PtC::Ptr cloud_pointer, boost::filesystem::path db_path)
  {
    query_set_=candidates_found_=db_set_=refinement_done_ =false;
    setQuery(name, cloud_pointer);
    if (generateLists(db_path))
      return (refineCandidates());
    else
      return false;
  }
  bool PoseEstimation::estimate(string name, PtC& cloud, PoseDB& database)
  {
    query_set_=candidates_found_=db_set_=refinement_done_ =false;
    setQuery(name, cloud);
    if ( generateLists(database) )
      return ( refineCandidates() );
    else
      return false;
  }
  bool PoseEstimation::estimate(string name, PtC::Ptr cloud_pointer, PoseDB& database)
  {
    query_set_=candidates_found_=db_set_=refinement_done_ =false;
    setQuery(name, cloud_pointer);
    if ( generateLists(database) )
      return ( refineCandidates() );
    else
      return false;
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

  bool PoseEstimation::saveEstimation(path file, bool append)
  {
    if (!refinement_done_)
    {
      if (params_["verbosity"]>0)
        print_warn("%*s]\tRefinement is not done yet, or wasn't successful... Saving nothing...\n",20,__func__);
      return false;
    }
    timestamp t(TIME_NOW);
    if (exists(file) && is_directory(file)) //existant directory, save to <query_name>.estimation inside it
    {
      try
      {
        fstream f;
        if (append)
          f.open ( (file.string() + query_name_ + ".estimation").c_str(), fstream::out | fstream::app );
        else
          f.open ( (file.string() + query_name_ + ".estimation").c_str(), fstream::trunc );
        f << "Pose estimation saved on " << (to_simple_string(t)).c_str() <<endl;
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
        f << "Pose estimation saved on " << (to_simple_string(t)).c_str() <<endl;
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
        f << "Pose estimation saved on " << (to_simple_string(t)).c_str() <<endl;
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
        f << "Pose estimation saved on " << (to_simple_string(t)).c_str() <<endl;
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


  boost::shared_ptr<Candidate> PoseEstimation::getEstimation()
  {
    if (! refinement_done_ )
    {
      if (params_["verbosity"]>0)
        print_warn("%*]\tRefinement has not yet been done, Pose Estimation is not complete, returning empty Candidate\n",20,__func__);
      Candidate c;
      return (boost::make_shared<Candidate> (c) );
    }
    Candidate c;
    c = *pose_estimation_;
    return ( boost::make_shared<Candidate> (c) );
  }

  bool PoseEstimation::getEstimation(boost::shared_ptr<Candidate> est)
  {
    if (! refinement_done_ )
    {
      if (params_["verbosity"]>0)
        print_warn("%*]\tRefinement has not yet been done, Pose Estimation is not complete, returning false\n",20,__func__);
      return false;
    }
    if (!est)
    {
      Candidate c;
      c = *pose_estimation_;
      est = boost::make_shared<Candidate> (c);
    }
    else
    {
      *est = *pose_estimation_;
    }
    return true;
  }

  bool PoseEstimation::getEstimationTransformation(Eigen::Matrix4f& t)
  {
    if (! refinement_done_ )
    {
      if (params_["verbosity"]>0)
        print_warn("%*]\tRefinement has not yet been done, Pose Estimation is not complete, returning false\n",20,__func__);
      return false;
    }
    t = pose_estimation_->transformation_;
    return true;
  }

  bool PoseEstimation::saveCandidates(boost::filesystem::path file)
  {
    if (!candidates_found_)
    {
      if (params_["verbosity"]>0)
        print_warn("%*s]\tList of Candidates are not yet generated, call generateLists before trying to save them!\n",20,__func__);
      return false;
    }
    if (exists(file) && params_["verbosity"]>1)
      print_info("%*s]\tAppending lists of candidates to %s\n",20,__func__,file.string().c_str());
    if (!exists(file) && params_["verbosity"]>1)
      print_info("%*s]\tSaving lists of candidates on %s\n",20,__func__,file.string().c_str());
    timestamp t(TIME_NOW);
    fstream f;
    f.open(file.string().c_str(), fstream::out | fstream::app);
    if (f.is_open())
    {
      f << "List of Candidates generated on "<<to_simple_string(t).c_str()<<" from query object named: "<<query_name_.c_str()<<endl;
      if (params_["useVFH"]>0)
      {
        f << "VFH:"<<endl;
        for (auto c : vfh_list_)
          f << c.name_.c_str() << "\tD: " << c.normalized_distance_<<endl;
      }
      if (params_["useESF"]>0)
      {
        f << "ESF:"<<endl;
        for (auto c : esf_list_)
          f << c.name_.c_str() << "\tD: " << c.normalized_distance_<<endl;
      }
      if (params_["useCVFH"]>0)
      {
        f << "CVFH:"<<endl;
        for (auto c : cvfh_list_)
          f << c.name_.c_str() << "\tD: " << c.normalized_distance_<<endl;
      }
      if (params_["useOURCVFH"]>0)
      {
        f << "OURCVFH:"<<endl;
        for (auto c : ourcvfh_list_)
          f << c.name_.c_str() << "\tD: " << c.normalized_distance_<<endl;
      }
      f << "Composite:"<<endl;
      for (auto c : vfh_list_)
        f << c.name_.c_str() << "\tD: " << c.normalized_distance_<<endl;
      f<<"End of Candidate Lists, saved by PoseEstimation::saveCandidates"<<endl<<endl;
      f.close();
    }
    else
    {
      print_error("%*s]\tError writing to %s...\n",20,__func__,file.string().c_str());
      return false;
    }
    return true;
  }

  void PoseEstimation::printQuery()
  {
    if (!query_set_)
    {
      if (params_["verbosity"]>0)
        print_warn("%*s]\tQuery is not set yet...\n",20,__func__);
      return;
    }
    if (params_["verbosity"]>1)
      print_info("%*s]\tPrinting query informations:\n",20,__func__);
    print_info("Query: ");
    print_value("%s",query_name_.c_str());
    print_info(" with a total of ");
    print_value("%d\n", query_cloud_->points.size());
    print_info("Was preprocessed and now has ");
    print_value("%d",query_cloud_->points.size());
    print_info(" points\n");
    if (vfh_.points.size() != 0)
    {
      print_value("%d ",vfh_.points.size());
      print_info("VFH histogram estimated\n");
    }
    if (esf_.points.size() != 0)
    {
      print_value("%d ",esf_.points.size());
      print_info("ESF histogram estimated\n");
    }
    if (cvfh_.points.size() != 0)
    {
      print_value("%d ",cvfh_.points.size());
      print_info("CVFH histogram(s) estimated\n");
    }
    if (ourcvfh_.points.size() != 0)
    {
      print_value("%d ",ourcvfh_.points.size());
      print_info("OURCVFH histogram(s) estimated\n");
    }
  }

  bool PoseEstimation::getQuery(string& name, PtC::Ptr clp)
  {
    if (!query_set_)
    {
      if (params_["verbosity"]>0)
        print_warn("%*s]\tQuery is not set yet...\n",20,__func__);
      return false;
    }
    name = query_name_;
    if (clp)
      copyPointCloud(*query_cloud_, *clp);
    else
    {
      PtC cl;
      copyPointCloud(*query_cloud_, cl);
      clp = cl.makeShared();
    }
    return true;
  }

  int PoseEstimation::getQueryFeatures(PointCloud<VFHSignature308>::Ptr vfh, PointCloud<VFHSignature308>::Ptr cvfh, PointCloud<VFHSignature308>::Ptr ourcvfh, PointCloud<ESFSignature640>::Ptr esf, PointCloud<Normal>::Ptr normals)
  {
    if (!query_set_)
    {
      if (params_["verbosity"]>0)
        print_warn("%*s]\tQuery is not set yet...\n",20,__func__);
      return 0;
    }
    int count (0);
    try
    {
      if (vfh && vfh_.points.size() != 0)
      {
        copyPointCloud(vfh_, *vfh);
        ++count;
      }
      else if (!vfh && vfh_.points.size() != 0)
      {
        PointCloud<VFHSignature308> cl;
        copyPointCloud(vfh_, cl);
        vfh = cl.makeShared();
        ++count;
      }
      else
      {
        if (params_["verbosity"]>0)
          print_warn("%*s]\tVFH feature was not calculated\n",20,__func__);
      }
      if (cvfh && cvfh_.points.size() != 0)
      {
        copyPointCloud(cvfh_, *cvfh);
        ++count;
      }
      else if (!cvfh && cvfh_.points.size() != 0)
      {
        PointCloud<VFHSignature308> cl;
        copyPointCloud(cvfh_, cl);
        cvfh = cl.makeShared();
        ++count;
      }
      else
      {
        if (params_["verbosity"]>0)
          print_warn("%*s]\tCVFH feature was not calculated\n",20,__func__);
      }
      if (ourcvfh && ourcvfh_.points.size() != 0)
      {
        copyPointCloud(ourcvfh_, *ourcvfh);
        ++count;
      }
      else if (!ourcvfh && ourcvfh_.points.size() != 0)
      {
        PointCloud<VFHSignature308> cl;
        copyPointCloud(ourcvfh_, cl);
        ourcvfh = cl.makeShared();
        ++count;
      }
      else
      {
        if (params_["verbosity"]>0)
          print_warn("%*s]\tOURCVFH feature was not calculated\n",20,__func__);
      }
      if (esf && esf_.points.size() != 0)
      {
        copyPointCloud(esf_, *esf);
        ++count;
      }
      else if (!esf && esf_.points.size() != 0)
      {
        PointCloud<ESFSignature640> cl;
        copyPointCloud(esf_, cl);
        esf = cl.makeShared();
        ++count;
      }
      else
      {
        if (params_["verbosity"]>0)
          print_warn("%*s]\tESF feature was not calculated\n",20,__func__);
      }
      if (normals && normals_.points.size() != 0)
      {
        copyPointCloud(normals_, *normals);
        ++count;
      }
      else if (!normals && normals_.points.size() != 0)
      {
        PointCloud<Normal> cl;
        copyPointCloud(normals_, cl);
        normals = cl.makeShared();
        ++count;
      }
      else
      {
        if (params_["verbosity"]>0)
          print_warn("%*s]\tNormals were not calculated\n",20,__func__);
      }
    }
    catch (...)
    {
      print_error("%*s]\tError copying features...\n",20,__func__);
      return (-1);
    }
    return count;
  }

  void PoseEstimation::viewQuery()
  {
    if (!query_set_)
    {
      if (params_["verbosity"]>0)
        print_warn("%*s]\tQuery is not set yet...\n",20,__func__);
      return;
    }
    visualization::PCLVisualizer viewer;
    visualization::PCLHistogramVisualizer v_vfh, v_esf, v_cvfh, v_ourcvfh;
    if (params_["useVFH"] > 0)
      v_vfh.addFeatureHistogram (vfh_, 308, "vfh");
    if (params_["useESF"] > 0)
      v_esf.addFeatureHistogram (esf_, 640, "esf");
    if (params_["useCVFH"] > 0)
      v_cvfh.addFeatureHistogram (cvfh_, 308, "cvfh");
    if (params_["useOURCVFH"] > 0)
      v_ourcvfh.addFeatureHistogram (ourcvfh_, 308, "ourcvfh");
    viewer.addPointCloud(query_cloud_, "query");
    viewer.addCoordinateSystem(0.2);
    while(!viewer.wasStopped())
    {
      viewer.spinOnce();
      v_vfh.spinOnce();
      v_esf.spinOnce();
      v_cvfh.spinOnce();
      v_ourcvfh.spinOnce();
    }
    viewer.close();
    return;
  }

  void PoseEstimation::viewEstimation()
  {
    if (!refinement_done_)
    {
      if (params_["verbosity"] >0)
        print_warn("%*s]\tEstimation is not done yet...\n",20,__func__);
      return;
    }
    visualization::PCLVisualizer viewer;
    PtC::Ptr pe (new PtC);
    transformPointCloud( *(pose_estimation_->cloud_), *pe, pose_estimation_->transformation_ );
    pe->sensor_origin_.setZero();
    pe->sensor_orientation_.setIdentity();
    visualization::PointCloudColorHandlerCustom<Pt> candidate_color_handler (pe, 0, 255, 0);
    visualization::PointCloudColorHandlerCustom<Pt> query_color_handler (query_cloud_, 255, 0, 0);
    viewer.addPointCloud(query_cloud_, query_color_handler, "query");
    viewer.addCoordinateSystem(0.2);
    viewer.addPointCloud(pe, candidate_color_handler, "estimation");
    while(!viewer.wasStopped())
    {
      viewer.spinOnce();
    }
    viewer.close();
    return;
  }

  bool PoseEstimation::saveTestResults(path file)
  {
    if (!refinement_done_)
    {
      if (params_["verbosity"] >0)
        print_warn("%*s]\tEstimation is not done yet...\n",20,__func__);
      return false;
    }
    if(saveTestResults(file, pose_estimation_->name_))
      return true;
    else
      return false;
  }

  bool PoseEstimation::saveTestResults(path file, string gt)
  {
    if (!refinement_done_)
    {
      if (params_["verbosity"] >0)
        print_warn("%*s]\tEstimation is not done yet...\n",20,__func__);
      return false;
    }
    try
    {
      fstream f;
      f.open ( file.string().c_str(), fstream::out | fstream::app );
      if (vfh_list_.empty())
      {
        f<< -1 <<" ";
      }
      else
      {
        for (int i=0; i<vfh_list_.size();++i)
        {
          if (gt.compare(vfh_list_[i].name_) ==0)
          {
            f << i+1 << " ";
            break;
          }
          if (i==vfh_list_.size()-1)
            f << 0 << " ";
        }
      }
      if (esf_list_.empty())
      {
        f<< -1 <<" ";
      }
      else
      {
        for (int i=0; i<esf_list_.size();++i)
        {
          if (gt.compare(esf_list_[i].name_) ==0)
          {
            f << i+1 << " ";
            break;
          }
          if (i==esf_list_.size()-1)
            f << 0 << " ";
        }
      }
      if (cvfh_list_.empty())
      {
        f<< -1 <<" ";
      }
      else
      {
        for (int i=0; i<cvfh_list_.size();++i)
        {
          if (gt.compare(cvfh_list_[i].name_) ==0)
          {
            f << i+1 << " ";
            break;
          }
          if (i==cvfh_list_.size()-1)
            f << 0 << " ";
        }
      }
      if (ourcvfh_list_.empty())
      {
        f<< -1 <<" ";
      }
      else
      {
        for (int i=0; i<ourcvfh_list_.size();++i)
        {
          if (gt.compare(ourcvfh_list_[i].name_) ==0)
          {
            f << i+1 << " ";
            break;
          }
          if (i==ourcvfh_list_.size()-1)
            f << 0 << " ";
        }
      }
      for (int i=0; i<composite_list_.size();++i)
      {
        if (gt.compare(composite_list_[i].name_) ==0)
        {
          f << i+1 << " ";
          break;
        }
        if (i==composite_list_.size()-1)
          f << 0 << " ";
      }
      if (pose_estimation_->name_.compare(gt) == 0)
      {
        f<< 0 <<endl;
      }
      else
      {
        vector<string> vc, vg;
        split (vc, pose_estimation_->name_, boost::is_any_of("_"), boost::token_compress_on);
        split (vg, gt, boost::is_any_of("_"), boost::token_compress_on);
        if (vc.at(0).compare(vg.at(0)) == 0)
        {
          int lat_c, lat_g, lon_c, lon_g;
          lat_c = stoi (vc.at(1));
          lon_c = stoi (vc.at(2));
          lat_g = stoi (vg.at(1));
          lon_g = stoi (vg.at(2));
          if ( (lat_c >= lat_g-10) && (lat_c <= lat_g+10) && (lon_c >= lon_g-10) && (lon_c <= lon_g+10) )
            f << 1 <<endl;
          else
            f << 2 <<endl;
        }
        else
          f << 3 <<endl;
      }
      f.close();
    }
    catch (...)
    {
      print_error ("%*s]\tError writing to disk, check if %s is valid path and name convetion is respected.\n",20,__func__,file.string().c_str());
      return false;
    }
    if (params_["verbosity"]>1)
      print_info("%*s]\tSaved current pose estimation results in %s\n",20,__func__,file.string().c_str());
    return true;
  }

  bool PoseEstimation::elaborateTests(path file, path result)
  {
    int k = params_["kNeighbors"];
    unsigned int tot_v(0),tot_e(0),tot_c(0),tot_o(0),tot(0);
    vector<int> vfh_r(k+1,0), esf_r(k+1,0), cvfh_r(k+1,0), ourcvfh_r(k+1,0), comp_r(k+1,0), final_c(4,0);
    if (exists(file) && is_regular_file(file))
    {
      ifstream f (file.string().c_str());
      string line;
      if (f.is_open())
      {
        while (getline (f, line))
        {
          try
          {
            vector<string> vst;
            split (vst, line, boost::is_any_of(" "), boost::token_compress_on);
            int vfh,esf,cvfh,ourcvfh,comp,fin_c;
            vfh = stoi(vst.at(0));
            esf = stoi(vst.at(1));
            cvfh = stoi(vst.at(2));
            ourcvfh = stoi(vst.at(3));
            comp = stoi(vst.at(4));
            fin_c = stoi(vst.at(5));
            if (vfh != -1)
            {
              ++vfh_r[vfh];
              ++tot_v;
            }
            if (esf != -1)
            {
              ++esf_r[esf];
              ++tot_e;
            }
            if (cvfh != -1)
            {
              ++cvfh_r[cvfh];
              ++tot_c;
            }
            if (ourcvfh != -1)
            {
              ++ourcvfh_r[ourcvfh];
              ++tot_o;
            }
            ++comp_r[comp];
            ++final_c[fin_c];
            ++tot;
          }
          catch (...)
          {
            print_error ("%*s]\tError reading from test file. Try recreating it.\n",20,__func__);
            return false;
          }
        }//end of file
      }
      else
      {
        print_error ("%*s]\tError opening test file for reading. Try recreating it.\n",20,__func__);
        return false;
      }
      f.close();
    }
    else
    {
      if (params_["verbosity"] >0)
        print_warn("%*s]\tTest file %s does not exists. Nothing to elaborate...\n",20,__func__, file.string().c_str());
      return false;
    }
    if (exists(result) && is_regular_file(result))
    {
      if (params_["verbosity"] >0)
        print_warn("%*s]\tResult file %s already exists. Truncating its contents...\n",20,__func__, result.string().c_str());
    }
    fstream res;
    res.open ( result.string().c_str(), fstream::out | fstream::trunc );
    if (res.is_open())
    {
      timestamp t(TIME_NOW);
      res << "Tests elaborates on "<<to_simple_string(t).c_str()<<endl;
      res <<"Total of "<< tot_v <<" VFH tests: ";
      for (int i=0; i< vfh_r.size(); ++i)
      {
        if (i == 0)
          res << vfh_r[i]/tot_v*100 <<"%% Failed\t";
        else
          res << vfh_r[i]/tot_v*100 <<"%% Rank "<<i<<"\t";
      }
      res<<endl;
      res <<"Total of "<< tot_e <<" ESF tests: ";
      for (int i=0; i< esf_r.size(); ++i)
      {
        if (i == 0)
          res << esf_r[i] <<" failed,\t";
        else
          res << esf_r[i] <<" rank "<<i<<",\t";
      }
      res<<endl;
      res <<"Total of "<< tot_c <<" CVFH tests: ";
      for (int i=0; i< cvfh_r.size(); ++i)
      {
        if (i == 0)
          res << cvfh_r[i]<<" failed,\t";
        else
          res << cvfh_r[i]<<" rank "<<i<<",\t";
      }
      res<<endl;
      res <<"Total of "<< tot_o <<" OURCVFH tests: ";
      for (int i=0; i< ourcvfh_r.size(); ++i)
      {
        if (i == 0)
          res << ourcvfh_r[i]<<" failed,\t";
        else
          res << ourcvfh_r[i]<<" rank "<<i<<",\t";
      }
      res<<endl;
      res <<"Total of "<< tot <<" Composite list tests: ";
      for (int i=0; i< comp_r.size(); ++i)
      {
        if (i == 0)
          res << comp_r[i]<<" failed,\t";
        else
          res << comp_r[i]<<" rank "<<i<<",\t";
      }
      res<<endl<<endl;
      res<<"On a total of "<<tot<<" tests, the final candidate chosen was:"<<endl;
      res<<"the correct pose "<<final_c[0]<<" times"<<endl;
      res<<"a direct neighbor of the correct pose "<<final_c[1]<<" times"<<endl;
      res<<"the same object with a wrong orientation "<<final_c[2]<<" times"<<endl;
      res<<"another object "<<final_c[3]<<" times"<<endl;
    }
    else
    {
      print_error ("%*s]\tError opening result file for writing. Check if %s is a valid path.\n",20,__func__,result.string().c_str());
      return false;
    }
    res.close();
    return true;
  }
  */
  }
}
