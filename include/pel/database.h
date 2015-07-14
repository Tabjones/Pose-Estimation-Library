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

#ifndef PEL_DATABASE_H_
#define PEL_DATABASE_H_

#include "common.h"

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
  class Database{

    //friend class PoseEstimation;

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
    ///unused
    bool clouds_in_local_; //TODO

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

    /** \brief Constructor which loads a database from disk
     * \param[in] pathDB Path to the directory containing the database of poses
     */
    Database (boost::filesystem::path pathDB)
    {
      this->load(pathDB);
    }

    /** \brief Copy constructor from another PoseDB
     * \param[in] db Database object to copy from
     */
    Database (const Database &db);

    /** TODO \brief Check if database contains cloud in local reference system
     * \return _True_ if clouds are expressed in local reference system, _False_ if they are expressed in sensor reference system, i.e. their sensor_origin_ and sensor_orientation_ are left as default (zero, identity).
     */
    bool
    isLocal ()
    {
      return (clouds_in_local_);
    }

    /** \brief Load a database from disk, previously saved with save method
     * \param[in] pathDB Path to the directory containing the database of poses
     * \return _True_ if operation is succesfull, _False_ otherwise
     */
    bool
    load (boost::filesystem::path pathDB);

    /** \brief Save a database to disk
     * \param[in] pathDB Path to a directory on disk, inside which to save the database.
     * \return _True_ if successful, _false_ otherwise
     *
     * pathDB must be a valid path and must not already exists, overwriting is not supported!
     */
    bool
    save (boost::filesystem::path pathDB);

    /** \brief Compute the whole database from scratch and store it in memory.
     * \param[in] pathClouds Path to a directory on disk that contains all the pcd files of object poses
     * \param[in] params Shared pointer to parameters to use during database creation
     * \return _True_ if successful, _false_ otherwise
     *
     * This method uses the provided set of parameters to create the database, erasing any previously loaded databases.
     * Please note that:
     - Constructing a database from scratch can take several minutes at least.
     - In order to use this method, PCD files must be expressed either in the sensor reference frame (i.e the kinect) or in local reference frame (i.e. in the object center). In the latter case sensor_origin_ and sensor_orientation_ of each cloud must be filled with the proper transformation that express where the sensor was during acquisition.
     - PCD files must represent previously segmented objects, no elements of the scene should be present.
     - PCD files must have stored the viewpoint location (coordinates of where the sensor was positioned during acquisition) inside their sensor_origin_ (if not zero) and sensor_orientation_ (if not identity).

     Failure to respect the above can lead to unexpected wrong results.
     */
   //TODO this must be a class of its own         bool create(boost::filesystem::path pathClouds, boost::shared_ptr<parameters> params);

    /** \brief Compute the whole database from scratch and store it in memory.
     * \param[in] pathClouds Path to a directory on disk that contains all the pcd files of object poses
     * \return _True_ if successful, _false_ otherwise
     *
     * This method creates a set of default parameters and creates the database from it, erasing any previously loaded databases.
     * Please note that:
     - Constructing a database from scratch can take several minutes at least.
     - In order to use this method, PCD files must be expressed either in the sensor reference frame (i.e the kinect) or in local reference frame (i.e. in the object center). In the latter case sensor_origin_ and sensor_orientation_ of each cloud must be filled with the proper transformation that express where the sensor was during acquisition.
     - PCD files must represent previously segmented objects, no elements of the scene should be present.
     - PCD files must have stored the viewpoint location (coordinates of where the sensor was positioned during acquisition) inside their sensor_origin_ (if not zero) and sensor_orientation_ (if not identity).

     Failure to respect the above can lead to unexpected wrong results.
     */
    //TODO look above!   bool create(boost::filesystem::path pathClouds);

    /** \brief Copy assignment operator
     * \param[in] db OBject to copy
     */
    //TODO also add other operators
    Database& operator= (const Database& db);

    /** \brief Erase the entire database from memory, leaving it unset
    */
    void
    clear ();

    /** \brief Tell if the database is empty
     *
     * \return _True_ if database is not loaded or empty, _False_ otherwise
     */
    bool
    isEmpty ();

    /** \brief Check if a path contains a valid database
     * \param[in] dbPath Path to directory containing database to check
     * \return _True_ if valid, _False_ otherwise
     *
     * Checks if the directory has a valid database structure, clouds of poses, FLANN matrices of histograms, indexes and name files.
     * A location saved with save method is always a valid path.
     */
    bool
    isValidPath (boost::filesystem::path dbPath);
  };
}

#endif //PEL_DATABASE_H_
