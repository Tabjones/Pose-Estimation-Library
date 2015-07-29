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
    params_["icp_max iterations"]=200;
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
}
