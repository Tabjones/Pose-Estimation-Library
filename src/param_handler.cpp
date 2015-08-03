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

#include <pel/param_handler.h>

using namespace pcl::console;

namespace pel
{
  ParamHandler::ParamHandler ()
  {
    params_["verbosity"]= 1;
    params_["use_vfh"]= params_["use_esf"]=params_["use_cvfh"]=params_["use_ourcvfh"]=1;
    params_["downsamp"] = 1;
    params_["downsamp_leaf_size"]=0.005f;
    params_["upsamp"] = params_["filter"]=0;
    params_["lists_size"]=20;
    params_["icp_max_iterations"]=200;
    params_["icp_reciprocal_corr"]=1;
    params_["icp_rmse_thresh"]=0.003f;
    params_["upsamp_poly_order"]=2;
    params_["upsamp_point_density"]=200;
    params_["upsamp_poly_fit"]=1;
    params_["upsamp_search_radius"]=0.05f;
    params_["filter_mean_k"]=50;
    params_["filter_std_dev_mul_thresh"]=3;
    params_["normals_radius_search"]=0.02;
    params_["cvfh_ang_thresh"]=7.5;
    params_["cvfh_curv_thresh"]=0.025;
    params_["cvfh_clus_tol"]=0.01;
    params_["cvfh_clus_min_points"]=50;
    params_["ourcvfh_ang_thresh"]=7.5;
    params_["ourcvfh_curv_thresh"]=0.025;
    params_["ourcvfh_clus_tol"]=0.01;
    params_["ourcvfh_clus_min_points"]=50;
    params_["ourcvfh_axis_ratio"]=0.95;
    params_["ourcvfh_min_axis_value"]=0.01;
    params_["ourcvfh_refine_clusters"]=1;
    size_of_valid_params_ = params_.size();
  }

  void
  ParamHandler::checkAndFixMinParam(std::string key, const float min)
  {
    if (params_.count(key))
      if(params_.at(key) < min)
      {
        if (params_.at("verbosity") > 0 )
          print_warn("%*s]\tParameter '%s' has value %g, which is below minimum allowed of %g, setting it to minimum...\n",20,__func__, key.c_str(),params_.at(key), min);
        params_.at(key) = min;
      }
  }

  void
  ParamHandler::checkAndFixMinMaxParam(std::string key, const float min, const float max)
  {
    checkAndFixMinParam(key,min);
    if (params_.count(key))
      if(params_.at(key) > max)
      {
        if (params_.at("verbosity") > 0 )
          print_warn("%*s]\tParameter '%s' has value %g, which is above maximum allowed of %g, setting it to maximum...\n",20,__func__, key.c_str(),params_.at(key), max);
        params_.at(key) = min;
      }
  }

  void
  ParamHandler::fixParameters ()
  {
    //Check Parameters correctness
    checkAndFixMinMaxParam("verbosity", 0, 2);
    checkAndFixMinMaxParam("downsamp", 0, 1);
    checkAndFixMinParam("downsamp_leaf_size", 0.0001);
    checkAndFixMinMaxParam("upsamp", 0,1);
    checkAndFixMinParam("lists_size", 1);
    checkAndFixMinParam("icp_max_iterations", 1);
    checkAndFixMinMaxParam("icp_reciprocal_corr", 0, 1);
    checkAndFixMinParam("icp_rmse_thresh", 0.000001);
    checkAndFixMinParam("upsamp_poly_order", 1);
    checkAndFixMinParam("upsamp_point_density", 1);
    checkAndFixMinMaxParam("upsamp_poly_fit", 0, 1);
    checkAndFixMinParam("upsamp_search_radius", 0.0001);
    checkAndFixMinParam("filter_mean_k", 1);
    checkAndFixMinParam("filter_std_dev_mul_thresh", 0.00001);
    checkAndFixMinParam("normals_radius_search", 0.0001);
    checkAndFixMinParam("cvfh_ang_thresh", 0.0001);
    checkAndFixMinParam("cvfh_curv_thresh", 0.0001);
    checkAndFixMinParam("cvfh_clus_tol", 0.0001);
    checkAndFixMinParam("cvfh_clus_min_points", 1);
    checkAndFixMinParam("ourcvfh_ang_thresh", 0.0001);
    checkAndFixMinParam("ourcvfh_curv_thresh", 0.0001);
    checkAndFixMinParam("ourcvfh_clus_tol", 0.0001);
    checkAndFixMinParam("ourcvfh_clus_min_points", 1);
    checkAndFixMinParam("ourcvfh_axis_ratio", 0.0001);
    checkAndFixMinParam("ourcvfh_min_axis_value", 0.0001);
    checkAndFixMinMaxParam("ourcvfh_refine_clusters", 0, 1);
  }

  bool
  ParamHandler::setParam (const std::string key, const float value)
  {
    float verb_level = getParam("verbosity");
    if (value < 0)
    {
      if (verb_level > 0)
        print_warn("%*s]\tParameter '%s' has a negative value (%g), ignoring...\n", 20,__func__, key.c_str(), value);
      return false;
    }
    params_[key]=value;
    //Check if key was a valid one, since the class has fixed number of parameters,
    //if one was mispelled, now we have one more
    if (params_.size() != size_of_valid_params_)
    {
      if (verb_level > 0)
        print_warn("%*s]\tInvalid key parameter '%s', ignoring...\n", 20,__func__, key.c_str());
      params_.erase(key);
      return false;
    }
    else if (verb_level > 1)
      print_info("%*s]\tSetting parameter: %s=%g\n",20,__func__,key.c_str(),value);
    return true;
  }

  bool
  ParamHandler::setParam (const std::string key, const std::string value)
  {
    float f;
    float verb_level = getParam("verbosity");
    try
    {
      f = stof(value);
    }
    catch (const std::invalid_argument& ia)
    {
      if ( verb_level > 0)
        print_warn("%*s]\tInvalid %s=%s : %s \n",20,__func__, key.c_str(), value.c_str(), ia.what());
      return false;
    }
    catch (const std::out_of_range& oor)
    {
      if (verb_level > 0)
        print_warn("%*s]\tInvalid %s=%s : %s \n", 20,__func__, key.c_str(), value.c_str(), oor.what());
      return false;
    }
    return (this->setParam(key, f));
  }

  int
  ParamHandler::loadParamsFromFile (const boost::filesystem::path config_file)
  {
    if ( boost::filesystem::exists(config_file) && boost::filesystem::is_regular_file(config_file))
    {
      float verb_level = getParam("verbosity");
      std::ifstream file (config_file.c_str());
      std::string line;
      if (file.is_open())
      {
        int count (0);
        while (getline (file, line))
        {
          boost::trim(line); //remove white spaces from line
          if (line.empty())
          {
            //do nothing empty line ...
            continue;
          }
          else if (line.compare(0,1,"#") == 0 )
          {
            //do nothing comment line ...
            continue;
          }
          else
          {
            std::vector<std::string> vst;
            //split the line to get a key and a token or trailing comments
            boost::split(vst, line, boost::is_any_of("#"), boost::token_compress_on);
            line = vst.at(0);
            boost::split(vst, line, boost::is_any_of(":"), boost::token_compress_on);
            if (vst.size()!=2)
            {
              if (verb_level > 0)
                print_warn("%*s]\tInvalid configuration line (%s), ignoring... Must be [Token]:[Value]\n", 20,__func__, line.c_str());
              continue;
            }
            std::string key (vst.at(0));
            std::string value (vst.at(1));
            boost::trim(key);
            boost::trim(value);
            if ( setParam(key, value) )
            {
              //success
              ++count;
            }
          }
        }//end of config file
        file.close();
        return (count);
      }
      else
      {
        print_error("%*s]\tCannot open config file! (%s)\n", 20,__func__, config_file.c_str());
        return (-1);
      }
    }
    else
    {
      print_error("%*s]\tPath to config_file is not valid, or non existant! (%s)\n", 20,__func__, config_file.c_str());
      return (-1);
    }
  }

  float
  ParamHandler::getParam (const std::string key) const
  {
    if (params_.count(key))
      return ( params_.at(key) );
    else
      return (-1);
  }

  int
  ParamHandler::setParamsFromMap (const parameters& par_map)
  {
    if (!par_map.empty())
    {
      int count (0);
      for (const auto& x: par_map)
        if (this->setParam(x.first, x.second))
          ++count;
      return (count);
    }
    else
    {
      print_error("%*s]\tEmpty map provided, cannot set parameters...\n", 20,__func__);
      return (-1);
    }
  }

  bool
  ParamHandler::dumpParamsToFile (boost::filesystem::path config_file, bool overwrite) const
  {
    float verb_level = getParam("verbosity");
    if (!config_file.has_extension())
    {
      config_file+=".yaml";
    }
    if (boost::filesystem::exists (config_file) && boost::filesystem::is_regular_file (config_file))
    {
      if (overwrite)
      {
        boost::filesystem::remove (config_file);
        if (verb_level > 0)
          print_warn("%*s]\t File %s already exists, overwriting it as requested.\n", 20,__func__, config_file.c_str());
      }
      else
      {
        print_error("%*s]\t File %s already exists, not overwriting it as requested. Aborting...\n", 20,__func__, config_file.c_str());
        return false;
      }
    }
    std::ofstream file;
    file.open(config_file.c_str());
    if (file.is_open())
    {
      try
      {
        for (const auto& x: params_)
          file << x.first << ": " << x.second << std::endl;
      }
      catch (...)
      {
        print_error("%*s]\tError writing into %s file, aborting...\n",20,__func__, config_file.c_str());
        return false;
      }
    }
    else
    {
      print_error("%*s]\tCannot open %s file for writing, aborting...\n",20,__func__, config_file.c_str());
      return false;
    }
    if (verb_level > 1)
      print_info("%*s]\tParameters written into %s succesfully.\n",20,__func__, config_file.c_str());
    return true;
  }
}
