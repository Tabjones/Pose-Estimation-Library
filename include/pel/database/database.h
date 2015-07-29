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
   * Manages PoseEstimation database, providing methods to load and save them.
   * The interface is used internally by PoseEstimation, however it can be used by the user
   * to manage multiple databases and test PoseEstimation with them.
   * For instance with setDatabase() method from PoseEstimation class.
   *
   * Some examples:
   * \code
   * Database db; //empty database
   * db.load("/path/to/location"); //load a database from the specified location
   * Database another_db ("/another/location"); //create another database and load it from another location
   * PoseEstimation test; //create a pose estimation with default parameters
   * test.setDatabase(db); //tell prova to use the first database
   * //... setting target, parameters, etc...
   * test.estimate;
   * test.setDatabase(another_db); //now use the second database
   * test.estimate; //estimate the same target again with another database
   * \endcode
   * \author Federico Spinelli
   */
  class Database
  {

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
    computeDistFromClusters (pcl::PointCloud<pcl::VFHSignature308>::Ptr target, listType feat,
        std::vector<std::pair<float, int> >& distIdx);

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

    /** \brief Erase the entire database. I.E. a call to isEmpty() method will return _True_ after
     * after this operation
    */
    void
    clear ();

    /** \brief Tell if the database is empty
     *
     * \return _True_ if database is not loaded or empty, _False_ otherwise
     */
    bool
    isEmpty () const;

    friend bool DatabaseReader::load (boost::filesystem::path, Database&);
    friend bool DatabaseWriter::save (boost::filesystem::path, const Database&, bool);
  };
}

#endif //PEL_DATABASE_DATABASE_H_
