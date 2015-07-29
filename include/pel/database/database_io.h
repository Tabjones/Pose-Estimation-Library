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

#ifndef PEL_DATABASE_DATABASE_IO_H_
#define PEL_DATABASE_DATABASE_IO_H_

#include <pel/common.h>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/algorithm/sort.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>

namespace pel
{
  class Database;
  /**\brief Reads(loads) a Database from disk.
   * Manages Database reading from disk, providing methods to load them.
   * \author Federico Spinelli
   */
  class DatabaseReader
  {
    ///Last succesfully loaded path
    boost::filesystem::path last_loaded_;

    public:
      /**\brief Empty Constructor
      */
      DatabaseReader () {}

      /**\brief Empty Destructor.
      */
      virtual ~DatabaseReader () {}

      /**\brief Load a database from disk.
       * \param[in] path Path to the directory on disk containing database to load
       * \parma[out] target Database object to load into.
       * \returns _True_ if operation is succesful, _False_ otherwise.
       *
       * If path is not valid or database fails to load, target is not touched
       */
      bool
      load (boost::filesystem::path path, Database& target);

      /**\brief Load a database from disk.
       * \param[in] path Path to the directory on disk containing database to load
       * \returns The loaded Database.
       *
       * If path is not valid or database fails to load, returns an empty database
       */
      Database
      load (boost::filesystem::path path);

      /**\brief Reload a database from the last succesfully loaded path
       * \param[out] target Database object to load into.
       * \returns _True_ if operation is succesful, _False_ otherwise
       */
      bool
      reload (Database& target);

      /**\brief Reload a database from the last succesfully loaded path
       * \returns The loaded database, or an empty one if operation fails
       */
      Database
      reload ();
  };

  /**\brief Writes(saves) a Database to disk.
   * Manages Database writing to disk, providing methods to save them.
   * \author Federico Spinelli
   */
  class DatabaseWriter
  {
    ///Last saved location
    boost::filesystem::path last_saved_;

    public:
      /**\brief Empty Constructor
       */
      DatabaseWriter () {}

      /**\brief Empty Destructor
       */
      virtual ~DatabaseWriter () {}

      /**\brief Save a Database to disk.
       * \param[in] path Path to a location on disk to save into.
       * \parma[in] db Database to save.
       * \param[in] overwrite if _True_ save location will get overwritten if needed regardless of its content, if _False_ operation will fail unless target location is an empty directory, or does not exists.
       * \return _True_ if operation is succesful, _False_ otherwise.
       */
      bool
      save (boost::filesystem::path path, const Database& db, bool overwrite=false);
  };
}
#endif //PEL_DATABASE_DATABASE_IO_H_
