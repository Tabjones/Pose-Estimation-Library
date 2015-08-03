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

#ifndef _INCL_PARAM_HANDLER_H_
#define _INCL_PARAM_HANDLER_H_

#include <pel/common.h>
#include <fstream>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>

namespace pel
{
  /**\brief Bare base interface for parameters handling
   */
  class ParamHandlerBase
  {
    public:
      // Dtor
      virtual ~ParamHandlerBase() =0;
      ///\brief Base interface for setParam
      virtual bool
      setParam (const std::string, const float)=0;
      ///\brief Base interface for getParam
      virtual float
      getParam (const std::string) const =0;
      ///\brief Base interface for dumpParamsToFile
      virtual bool
      dumpParamsToFile (boost::filesystem::path, bool) const =0;
      ///\brief Base interface for loadParamsFromFile
      virtual int
      loadParamsFromFile (const boost::filesystem::path) =0;
  };
  /**\brief Base class for parameters handling
   *
   * Cannot be used directly, but only inherithed
   */
  class ParamHandler : public ParamHandlerBase
  {
    protected:
      ///Protected constructor, we dont want explicit instantiation of the class, only derived, still we dont want it to be pure virtual
      ParamHandler ();
      ///Protected destructor
      virtual ~ParamHandler () {}
      ///Map that stores parameters in key=value fashion
      parameters params_;
      ///How many valid parameters are there
      int size_of_valid_params_;
      ///\brief Set a param from a string value converting it to float. Used internally to read from file.
      bool
      setParam (const std::string key, const std::string value);
      ///\brief Check for parameters value correctness, and adjust them if they are obviously wrong
      void
      fixParameters ();
      ///\brief Check if specified key is bounded between a min and max value and evenutally threshold it
      void
      checkAndFixMinMaxParam (std::string key, const float min, const float max);
      ///\brief Check if specified key is below certain min value, and eventually threshold it
      void
      checkAndFixMinParam (std::string key, const float min);

    public:
      /**\brief Set a param from a float value
       * \param[in] key Valid key that identifies a parameter, look at docs for a valid keys list
       * \param[in] value The value the key will assume
       * \return _True_ if the specified key is correctly set to specified value, _False_ otherwise
       */
      virtual bool
      setParam (const std::string key, const float value);

      /**\brief Load a set of parameters from a configuration file
       * \param[in] config_file File containing parameters in yaml format
       * \returns The number of parameters set correctly, or (-1) in case of errors.
       *
       * Note: Config file can be any file, but must contain a "key: value" on each line (with or without whitespaces). Lists or other
       * YAML contruct are not supported.
       * Note2: config_file could specify a subset of all valid parameters (even only 1). in this case
       * only specified parameters are set, others are left untouched.
       */
      virtual int
      loadParamsFromFile (const boost::filesystem::path config_file);

      /**\brief Set parameters from a previously initialized map
       * \param[in] par_map std::unorderd_map to set parameters from.
       * \returns The number of parameters set correctly, or (-1) in case of errors.
       *
       * Note: par_map could be a subset of all valid parameters (even only 1), in this case only
       * specified parameters are set, others are left untouched.
       */
      int
      setParamsFromMap (const parameters& par_map);

      /**\brief Get a map containing all current parameters
       * \returns The map of parameters
      */
      inline parameters
      getAllParams () const
      {
        return params_;
      }

      /**\brief Get the value of a specified parameter key
       *\param[in] key Valid key that identifies a parameter to read
       *\return The value of requested key, or (-1) in case of errors
       */
      virtual float
      getParam (const std::string key) const;

      /**\brief Save current parameters to a YAML file
       *\param[in] config_file Path on disk to dump parameters
       *\param[in] overwrite Truncate old file, if exists. If _False_ old file will be preserved.
       *\return _True_ if operation is succesful, _False_ otherwise.
       *
       * Note: a .yaml extension gets appended to config_file if it has no extension.
       */
      virtual bool
      dumpParamsToFile (boost::filesystem::path config_file, bool overwrite=false) const;

  };
}
#endif //_INCL_PARAM_HANDLER_H_
