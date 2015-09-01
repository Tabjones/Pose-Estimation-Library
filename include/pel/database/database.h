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

#ifndef PEL_DATABASE_DATABASE_H_
#define PEL_DATABASE_DATABASE_H_

#include <pel/common.h>
#include <pel/database/database_io.h>
#include <pel/database/database_creator.h>

namespace pel
{
  /**\brief Stores the database of poses for PoseEstimation.
   *
   * Manages Pose Estimation database.
   * The interface is used internally by Pose Estimation Procedure, however it can be used by the user
   * to manage multiple databases and test Pose Estimation with them, along with helper classes to load and save
   * databases.
   * \note See also DatabaseReader, DatabaseWriter and DatabaseCreator classes.
   *
   * Some examples:
   * \code
   * #include <pel/pe_brute_force.h>
   * #include <pel/database/database_io.h>
   * //...
   * pel::Database db1, db2; //empty databases
   * pel::DatabaseReader reader;
   * db1 = reader.load("/path/to/location"); //load a database from the specified location and stores it in db1
   * db2 = db1; //Both databases contain the same
   * db1 = reader.load("/patho/to/another/location"); //load another database and overwrite db1
   * //...
   * pel::PEBruteForce pose_estimation; //Declare a PoseEstimation object
   * pose_estimation.setTarget(target); //Set its target
   * pose_estimation.setDatabase(db1); //Use the first database
   * pose_estimation.estimate(result1); //estimate the target with first database
   * pose_estimation.setDatabase(db2); //Now use the second database
   * pose_estimation.estimate(result2); //estimate the same target with second database
   * \endcode
   * \author Federico Spinelli
   */
  class Database
  {
    protected:
      ///Shared pointers to database histograms
      boost::shared_ptr<histograms> vfh_, esf_, cvfh_, ourcvfh_;
      ///Names of database clouds
      std::vector<std::string> names_;
      ///Names of clusters of CVFH and OURCVFH clouds
      std::vector<std::string> names_cvfh_, names_ourcvfh_;
      ///Path to database location on disk
      boost::filesystem::path db_path_;
      ///Database of point clouds
      std::vector<PtC> clouds_;
      ///Flann index for vfh
      boost::shared_ptr<indexVFH> vfh_idx_;
      ///Flann index for esf
      boost::shared_ptr<indexESF> esf_idx_;

      /**\brief Calculates unnormalized distance of objects, based on their cluster distances. This is only used
       * for CVFH and OURCVFH, since other features don't have clusters.
       * \param[in] target Pointer to the target histogram
       * \param[in] feat Enum that indicates from which list the histogram belongs (listType::cvfh or listType::ourcvfh only)
       * \param[out] distIdx Vector of unnormalized distances of objects and their relative index
       * \return _True_ if distances are correctly computed, _false_ otherwise
       */
      bool
        computeDistFromClusters (pcl::PointCloud<pcl::VFHSignature308>::Ptr target, ListType feat, std::vector<std::pair<float, int> >& distIdx);

    public:
      /** \brief Default empty Constructor
      */
      Database () {}

      /** \brief Copy constructor
       * \param[in] other Database to copy from
       */
      Database (const Database& other);

      /**\brief Move constructor
       * \param[in] other Database to move from
       */
      Database (Database&& other);

      /** \brief Copy assignment operator
       * \param[in] other Database to copy from
       */
      Database& operator= (const Database& other);

      /** \brief Move assignment operator
       * \param[in] other Database to move from
       */
      Database& operator= (Database&& other);

      /** \brief Erase the entire database. I.E. a call to isEmpty() method will return _True_
       * after this operation
       */
      void
      clear ();

      /** \brief Tell if the database is empty
       *\return _True_ if database is not loaded or empty, _False_ otherwise
       */
      bool
      isEmpty () const;

      /**\brief Get the database of VFH histograms as pointer to n*308 matrix.
       *\return Shared pointer to matrix
       _n_ is the number of poses in Database
      */
      inline boost::shared_ptr<histograms>
      getDatabaseVFH () const
      {
        return (vfh_);
      }
      /**\brief Get the database of ESF histograms as pointer to n*640 matrix
       *\return Shared pointer to matrix
       _n_ is the number of poses in Database
      */
      inline boost::shared_ptr<histograms>
      getDatabaseESF () const
      {
        return (esf_);
      }
      /**\brief Get the database of CVFH histograms as pointer to m*308 matrix
       *\return Shared pointer to matrix
       _m_ is the number of poses in Database plus number of clusters per object, which may vary per object
      */
      inline boost::shared_ptr<histograms>
      getDatabaseCVFH () const
      {
        return (cvfh_);
      }
      /**\brief Get the database of OURCVFH histograms as pointer to p*308 matrix
       *\return Shared pointer to matrix
       _p_ is the number of poses in Database plus number of clusters per object, which may vary per object
      */
      inline boost::shared_ptr<histograms>
      getDatabaseOURCVFH () const
      {
        return (ourcvfh_);
      }
      /**\brief get an _n_ lenght vector containing names of poses in database
       *\return vector of names
       _n_ is the number of poses in Database
       */
      inline std::vector<std::string>
      getDatabaseNames () const
      {
        return (names_);
      }
      /**\brief get an _m_ lenght vector containing names of poses in database for CVFH descriptor
       *\return vector of names
       _m_ is the number of poses in Database plus the number of clusters of each pose.
       */
      inline std::vector<std::string>
      getDatabaseNamesCVFH () const
      {
        return (names_cvfh_);
      }
      /**\brief get an _p_ lenght vector containing names of poses in database for OURCVFH descriptor
       *\return vector of names
       _p_ is the number of poses in Database plus the number of clusters of each pose.
       */
      inline std::vector<std::string>
      getDatabaseNamesOURCVFH () const
      {
        return (names_ourcvfh_);
      }
      /**\brief get a path to Database saved location, if exists.
       *\return path of directory containing Database on disk
       */
      inline boost::filesystem::path
      getDatabasePath () const
      {
        return (db_path_);
      }
      /**\brief get an _n_ lenght vector containing point clouds of poses in database
       *\return vector of point clouds
       _n_ is the number of poses in Database
       */
      inline std::vector<PtC>
      getDatabaseClouds () const
      {
        return (clouds_);
      }
      /**\brief get a pointer to FLANN index for VFH histograms
       *\return shared pointer of FLANN index
       */
      inline boost::shared_ptr<indexVFH>
      getDatabaseIndexVFH () const
      {
        return (vfh_idx_);
      }
      /**\brief get a pointer to FLANN index for ESF histograms
       *\return shared pointer of FLANN index
       */
      inline boost::shared_ptr<indexESF>
      getDatabaseIndexESF () const
      {
        return (esf_idx_);
      }
      ///Friend functions of this class
      friend bool DatabaseReader::load (boost::filesystem::path, Database&);
      friend bool DatabaseWriter::save (boost::filesystem::path, const Database&, bool);
      friend Database DatabaseCreator::create (boost::filesystem::path path_cloud);
  };
}

#endif //PEL_DATABASE_DATABASE_H_
