#include <iostream>
#include <pcl/common/norms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <fstream>
#include <pcl/filters/statistical_outlier_removal.h>
#include <boost/algorithm/string/split.hpp>
//#include <Eigen/Dense>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/vfh.h>
#include <pcl/features/esf.h>
#include <pcl/features/cvfh.h>
//#include <pcl/features/fpfh_omp.h>
//#include <pcl/features/gfpfh.h>
#include <pcl/features/our_cvfh.h>
//#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <cmath>
#include <pcl/visualization/pcl_visualizer.h>

#define D2R 0.017453293 //degrees to radians conversion

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std;
using namespace boost;

typedef pair<string, PointCloud<PointXYZRGBA> > labelCloud; 
typedef std::vector<boost::filesystem::path> p_vector; //Vector of Paths
typedef std::vector<std::string> s_vector; //Vector of Strings

vector<labelCloud> orderedClouds; //vector to store the sorted clouds
vector<PointCloud<Normal> > normals; //vector to store cloud normals
//PointCloud<FPFHSignature33>::Ptr fpfh (new PointCloud<FPFHSignature33>); //cloud to store the fpfhs during GFPFH pipeline

/* Program behaviour variables*/
bool visual_debug (false);
bool mls_upsampling (false);
bool filter(false);

////////////////////////////////////////////////
//////////// Show Help Function ////////////////
////////////////////////////////////////////////
void
showHelp (char *progName)
{
  cout<<std::endl;
  print_value( "********************************************************************\n"
               "***           Pose Estimation - Build Database - Help             **\n"
               "********************************************************************\n");
  print_highlight( "Usage %s [TrainingSet Dir] <Options>\n", progName);
  print_info(  "Options:\n"
               "\t  -h:\t Show this help.\n"
               "\t  -v:\t Show some clouds during the steps of database processing\n"
               "\t     \t such as downsampled clouds, normal estimations, etc...\n"
               "\t  -u:\t Apply an Upsampling with MLS random uniform density before\n"
               "\t     \t processing the downsampling. (Greatly increases execution time)\n"
               "\t  -f:\t Apply a Statistical Outliers Removal filter  before\n"
               "\t     \t processing the up/downsampling. (k=50, std=1.6)\n");
  cout<<std::endl;
}

//////////////////////////////////////////////////////////
//////// Parse Command Line Function /////////////////////
//////////////////////////////////////////////////////////
void
parseCommandLine (int argc, char *argv[])
{
  //Show help
  if (find_switch (argc, argv, "-h"))
  {
    showHelp (argv[0]);
    exit (0);
  }
  if (find_switch (argc, argv, "-v"))
  {
    visual_debug = true;
    print_info ("Enabling Visual Mode ...\n");
  }
  if (find_switch (argc, argv, "-f"))
  {
    filter = true;
    print_info ("Enabling Filtering ...\n");
  }
  if (find_switch (argc, argv, "-u"))
  {
    mls_upsampling = true;
    print_info ("Enabling MLS Upsampling ...\n");
  }
  cout<<std::flush;
}


/* MAIN*/  
int
main (int argc, char** argv)
{
  parseCommandLine (argc, argv);
  if (argc < 2)
  {
    print_error ("Need TrainingSet Dir as first argument!\n");
    showHelp (argv[0]);
    return (0);
  }
  if ( !filesystem::exists (argv[1]) && !filesystem::is_directory (argv[1]) )
  {
    print_error ("%s is not a directory! Need TrainingSet Dir as first argument!\n", argv[1]);
    showHelp (argv[0]);
    return (0);
  }
  //Database dir creation
  filesystem::path data_dir("./Database/");
  filesystem::path cdir(data_dir.string() + "Clouds/");
  filesystem::path ndir(data_dir.string() + "Normals/"); 
  if ( !filesystem::exists (data_dir) && !filesystem::is_directory (data_dir) )
    filesystem::create_directory(data_dir);
  if ( !filesystem::exists (cdir) && !filesystem::is_directory (cdir) )
    filesystem::create_directory(cdir);
  if ( !filesystem::exists (ndir) && !filesystem::is_directory (ndir) )
    filesystem::create_directory(ndir);

  /////////////////// Loading Phase ////////////////////
  // 
  // Clouds gets sorted by name then by angle and 
  // finally stored in orderedClouds global vector.
  //
  p_vector pvec;
  s_vector svec;
  copy (filesystem::directory_iterator(argv[1]), filesystem::directory_iterator(), back_inserter(pvec));
  sort (pvec.begin(), pvec.end()); 
  
  for (p_vector::const_iterator it(pvec.begin()); it != pvec.end(); ++it)
  {
    if (is_regular_file( *it ) && it->extension() == ".pcd")
    {
      s_vector vz, vy;
      split (vy, it->string(), boost::is_any_of("/\\.._"), boost::token_compress_on);
      if (!svec.empty())
        split (vz, svec.at(svec.size()-1), boost::is_any_of("/\\.._"), boost::token_compress_on);
      if (svec.empty())
      {
        svec.push_back(it->string());
      }
      else if ( (vz.at(vz.size()-3)).compare(vy.at(vy.size()-3)) == 0)
      {//objects name are the same
        svec.push_back(it->string());
        if (it == (pvec.begin()+pvec.size()-1)) //if im at last element
        {
          sort(svec.begin(), svec.end(),
            [](std::string const & a, std::string const & b)
            { 
              s_vector va,vb;
              split (va, a, boost::is_any_of ("/\\.._"), boost::token_compress_on);
              split (vb, b, boost::is_any_of ("/\\.._"), boost::token_compress_on);
              return ( stoi(va.at(va.size()-2)) < stoi(vb.at(vb.size()-2)) ); 
            });
          for (s_vector::const_iterator s(svec.begin()); s != svec.end(); ++s)
          {
            s_vector sv;
            split (sv, *s, boost::is_any_of ("/\\.."), boost::token_compress_on);
            labelCloud tmp;
            loadPCDFile (s->c_str(), tmp.second);
            tmp.first = sv.at(sv.size()-2);
            orderedClouds.push_back(tmp);
          }
          svec.clear();
        } 
      } 
      else
      {
        sort(svec.begin(), svec.end(),
          [](std::string const & a, std::string const & b)
          { 
            s_vector va,vb;
            split (va, a, boost::is_any_of ("/\\.._"), boost::token_compress_on);
            split (vb, b, boost::is_any_of ("/\\.._"), boost::token_compress_on);
            return ( stoi(va.at(va.size()-2)) < stoi(vb.at(vb.size()-2)) ); 
          });
        for (s_vector::const_iterator s(svec.begin()); s != svec.end(); ++s)
        {
          s_vector sv;
          split (sv, *s, boost::is_any_of ("/\\.."), boost::token_compress_on);
          labelCloud tmp;
          loadPCDFile (s->c_str(), tmp.second);
          tmp.first = sv.at(sv.size()-2);
          orderedClouds.push_back(tmp);
        }
        svec.clear();
        svec.push_back(it->string());
      }
    }
    print_highlight("Loading and sorting clouds... \t\t");
    print_value("%d\r", orderedClouds.size() );
    cout<<std::flush;
  }
  print_value("\t\t\t\t\t\t\t\tOK\n");
  /////////////////////// Outliers Filtering ////////////////////////
  //
  // Clouds get filtered with statistical outliers removal
  // n=50 neighbours selected with std weight to 1.6
  // Only do this step if requested
  //
  if (filter)
  {
    for (size_t i=0; i<orderedClouds.size(); ++i)
    {
      StatisticalOutlierRemoval<PointXYZRGBA> filter;
      filter.setMeanK (50);  //modifies these to render the filter more aggressive, greater number more aggressiveness
      filter.setStddevMulThresh (3); //the smaller is this, the more points are considered outliers and thus removed
      //Create PointClouds for filter
      PointCloud<PointXYZRGBA>::Ptr input (orderedClouds[i].second.makeShared()); 
      PointCloud<PointXYZRGBA>::Ptr output (new PointCloud<PointXYZRGBA>() );
      filter.setInputCloud(input);
      filter.filter(*output);
      copyPointCloud (*output, orderedClouds[i].second);
      print_highlight("Statistical Outliers Removal...\t\t");
      print_value("[%d/%d]\r", i+1, orderedClouds.size());
      cout<<flush;
    }
    print_value("\t\t\t\t\t\t\t\tOK\n");
  }
  /////////////////////// DownSampling Phase ////////////////////////
  //
  //  Clouds get Upsampled then Downsampled to 3mm to
  //  ensure an omogeneous distribution of points
  //
  PCDWriter writer;
  for (size_t i=0; i<orderedClouds.size(); ++i)
  {
    MovingLeastSquares<PointXYZRGBA, PointXYZRGBA> mls;
    search::KdTree<PointXYZRGBA>::Ptr tree (new search::KdTree<PointXYZRGBA>);
    PointCloud<PointXYZRGBA>::Ptr input (orderedClouds[i].second.makeShared());
    PointCloud<PointXYZRGBA>::Ptr output (new PointCloud<PointXYZRGBA>);
    if (mls_upsampling) //Only upsample if requested
    {
      mls.setInputCloud (input);
      mls.setSearchMethod (tree);
      mls.setUpsamplingMethod (MovingLeastSquares<PointXYZRGBA, PointXYZRGBA>::RANDOM_UNIFORM_DENSITY);
      mls.setComputeNormals (false);
      mls.setPolynomialOrder (2);
      mls.setPolynomialFit (true);
      mls.setSearchRadius (0.03); //3cm
//    mls.setUpsamplingRadius (0.002); //5mm    OLD params for method of upsampling using planes
//    mls.setUpsamplingStepSize (0.0025); //2.5mm
      mls.setPointDensity(250);
      mls.process (*output); //Process Upsampling
      copyPointCloud (*output, *input); 
      StatisticalOutlierRemoval<PointXYZRGBA> filter;
      filter.setMeanK (50);  //modifies these to render the filter more aggressive, greater number more aggressiveness
      filter.setStddevMulThresh (4); //the smaller is this, the more points are considered outliers and thus removed
      filter.setInputCloud(input);
      filter.filter(*output);
      copyPointCloud (*output, *input);
    }
    VoxelGrid <PointXYZRGBA> vgrid;
    vgrid.setInputCloud (input);
    vgrid.setLeafSize (0.003, 0.003, 0.003); //Downsample to 3mm
    vgrid.setDownsampleAllData (true);
    vgrid.filter (*output); //Process Downsampling
    if (mls_upsampling)
      print_highlight("MLS upsampling and VoxelGrid downsampling... \t");
    else
      print_highlight("VoxelGrid downsampling...                    \t");
    print_value("[%d/%d]\r", i+1, orderedClouds.size());
    cout<<std::flush;
    copyPointCloud (*output, orderedClouds[i].second);  
    // DEBUG to visualize some clouds
    if (i%35==0 && visual_debug)
    {
      pcl::visualization::PCLVisualizer viewer;
      viewer.addPointCloud (orderedClouds[i].second.makeShared(), "cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
      viewer.setBackgroundColor(0.95,.95,0.95);
      while (!viewer.wasStopped ())
      {
        viewer.spinOnce ();
      }
      viewer.close();
    }  
  }
  print_value("\t\t\t\t\t\t\t\tOK\n");
  ofstream fs_list;
  fs_list.open ((data_dir.string() + "names.list").c_str());
  for (size_t i=0; i<orderedClouds.size(); ++i)
  {
    writer.writeBinaryCompressed(cdir.string() + orderedClouds[i].first + ".pcd", orderedClouds[i].second );
    fs_list << orderedClouds[i].first <<"\n";    
  }
  fs_list.close();

  ////////////////////////// Normals Computation //////////////////////////
  //
  //  Compute all clouds normals, get a viewpoint based on scan
  //  name
  //
  int j=0;
  for (size_t i=0; i<orderedClouds.size(); ++i, j+=10)
  {
    if (j==360)
      j=0;
    float vx,vy,vz;
    vx = cos(22*D2R)*sin(j*D2R);
    vy = sin(22*D2R);
    vz = cos(22*D2R)*cos(j*D2R);
    NormalEstimationOMP<PointXYZRGBA, Normal> ne;
    PointCloud<PointXYZRGBA>::Ptr input (new PointCloud<PointXYZRGBA>);
    copyPointCloud(orderedClouds[i].second, *input);
    PointCloud<Normal>::Ptr output (new PointCloud<Normal>);
    search::KdTree<PointXYZRGBA>::Ptr tree (new search::KdTree<PointXYZRGBA>);
    ne.setNumberOfThreads (0); //auto allocation
    ne.setInputCloud (input);
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.015); //1.5cm
    ne.setViewPoint (vx, vy, vz);
    ne.compute (*output);
    normals.push_back(*output);
    print_highlight("Normal estimation... \t\t\t\t");
    print_value("[%d/%d]\r", i+1, orderedClouds.size());
    cout<<std::flush;
    writer.writeBinaryCompressed(ndir.string() + orderedClouds[i].first + ".pcd", normals[i]);
    // DEBUG to visualize some cloud normals
    if (i%35==0 && visual_debug)
    {
      pcl::visualization::PCLVisualizer viewer;
      viewer.addPointCloud (orderedClouds[i].second.makeShared(), "cloud");
      viewer.addPointCloudNormals<PointXYZRGBA, Normal>(orderedClouds[i].second.makeShared(), normals[i].makeShared(), 3, 0.01f, "normals");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "cloud");
      viewer.setBackgroundColor(0.9,.95,0.95);
      while (!viewer.wasStopped ())
      {
        viewer.spinOnce ();
      }
      viewer.close();
    }  
  }
  print_value("\t\t\t\t\t\t\t\tOK\n");
  //////////////////////// VFH Computation ////////////////////////
  //
  // Compute all VFH histograms, store them in Flann data and
  // save them to disk along with the index
  //
  flann::Matrix<float> vfh (new float[orderedClouds.size() * 308], orderedClouds.size(), 308);
  j=0;
  for (size_t i=0; i<orderedClouds.size(); ++i, j+=10)
  {
    if (j==360)
      j=0;
    float vx,vy,vz;
    vx = cos(22*D2R)*sin(j*D2R);
    vy = sin(22*D2R);
    vz = cos(22*D2R)*cos(j*D2R);
    VFHEstimation<PointXYZRGBA, Normal, VFHSignature308> vfhE;
    PointCloud<PointXYZRGBA>::Ptr input (new PointCloud<PointXYZRGBA>);
    copyPointCloud(orderedClouds[i].second, *input);
    PointCloud<Normal>::Ptr input_N (new PointCloud<Normal>);
    copyPointCloud(normals[i], *input_N);
    PointCloud<VFHSignature308>::Ptr output (new PointCloud<VFHSignature308>);
    search::KdTree<PointXYZRGBA>::Ptr tree (new search::KdTree<PointXYZRGBA>);
    vfhE.setInputCloud (input);
    vfhE.setSearchMethod (tree);
    vfhE.setViewPoint (vx, vy, vz);
    vfhE.setInputNormals (input_N);
    vfhE.compute (*output);
    print_highlight("VFH estimation... \t\t\t\t");
    print_value("[%d/%d]\r", i+1, orderedClouds.size());
    cout<<std::flush;
    for (size_t k=0; k < vfh.cols; ++k)
      vfh[i][k] = output->points[0].histogram[k];
  }
  flann::save_to_file (vfh, data_dir.string() + "vfh.h5", "VFH Histograms");
  flann::Index<flann::ChiSquareDistance<float> > vfh_index (vfh, flann::KDTreeIndexParams(4)); //vfh performs best with chi^2 distance, kdtree default
  vfh_index.buildIndex();
  vfh_index.save (data_dir.string() + "vfh.idx");
  print_value("\t\t\t\t\t\t\t\tOK\n");
  ////////////////////////// ESF Computation //////////////////////////
  //
  // Compute all ESF histograms, store them in Flann data and
  // save them to disk along with the index
  //
  flann::Matrix<float> esf (new float[orderedClouds.size() * 640], orderedClouds.size(), 640);
  for (size_t i=0; i<orderedClouds.size(); ++i)
  {
    ESFEstimation<PointXYZRGBA, ESFSignature640> esfE;
    PointCloud<PointXYZRGBA>::Ptr input (new PointCloud<PointXYZRGBA>);
    copyPointCloud (orderedClouds[i].second, *input);
    PointCloud<ESFSignature640>::Ptr output (new PointCloud<ESFSignature640>);
    search::KdTree<PointXYZRGBA>::Ptr tree (new search::KdTree<PointXYZRGBA>);
    esfE.setInputCloud (input);
    esfE.setSearchMethod (tree);
    esfE.compute (*output);
    print_highlight("ESF estimation... \t\t\t\t");
    print_value("[%d/%d]\r", i+1, orderedClouds.size());
    cout<<std::flush;
    for (size_t k=0; k < esf.cols; ++k)
      esf[i][k] = output->points[0].histogram[k];
  }
  flann::save_to_file (esf, data_dir.string() + "esf.h5", "ESF Histograms");
  flann::Index<flann::L2<float> > esf_index (esf, flann::KDTreeIndexParams(4)); //Esf performs best with L2 norm (source paper), building kdtree with default params
  esf_index.buildIndex();
  esf_index.save (data_dir.string() + "esf.idx");
  print_value("\t\t\t\t\t\t\t\tOK\n");           
  ///////////////////////////// CVFH Computation ////////////////////////
  //
  //  Computes all CVFH descriptors and store them in flann data,
  //  objects may have more than one entry based on how many clusters
  //  are found.
  //
  PointCloud<VFHSignature308> descriptors;
  descriptors.clear();
  ofstream cvfh_list;
  cvfh_list.open((data_dir.string()+ "cvfh.list").c_str(), fstream::trunc); //truncate contents of the file, if it already exists
  cvfh_list.close();
  j=0;
  for (size_t i=0; i<orderedClouds.size(); ++i, j+=10)
  {
    if (j==360)
      j=0;
    float vx,vy,vz;
    vx = cos(22*D2R)*sin(j*D2R);
    vy = sin(22*D2R);
    vz = cos(22*D2R)*cos(j*D2R);
    CVFHEstimation<PointXYZRGBA, Normal, VFHSignature308> cvfhE;
    PointCloud<PointXYZRGBA>::Ptr input (new PointCloud<PointXYZRGBA>);
    copyPointCloud(orderedClouds[i].second, *input);
    PointCloud<Normal>::Ptr input_N (new PointCloud<Normal>);
    copyPointCloud(normals[i], *input_N);
    PointCloud<VFHSignature308>::Ptr output (new PointCloud<VFHSignature308>);
    search::KdTree<PointXYZRGBA>::Ptr tree (new search::KdTree<PointXYZRGBA>);
    cvfhE.setInputCloud (input);
    cvfhE.setSearchMethod (tree);
    cvfhE.setViewPoint (vx, vy, vz);
    cvfhE.setInputNormals (input_N);
    //Set maximum allowable deviation of the normals, in the region segmentation step
    cvfhE.setEPSAngleThreshold(7.5*D2R); // 7.5 degrees according to paper
    //Set maximum disparity between curvatures, in the region segmentation step
    cvfhE.setCurvatureThreshold(0.025f); //this value is from related paper
    //Set to true to normalize the bins of the histogram using the total number of points.
    //Doing this will render cvfh invariant to scale, but the authors encourage the opposite.
    cvfhE.setNormalizeBins(false); //leaving it false for now
    //Clustering parameters
    cvfhE.setClusterTolerance (0.01f); //1 cm all points should have a density of 3 mm
    cvfhE.setMinPoints (50); //at least 50 points to consider a cluster to be valid (as stated in the paper)
    //Compute method
    cvfhE.compute (*output);
    cvfh_list.open ((data_dir.string() + "cvfh.list").c_str(), fstream::out | fstream::app);  //append mode
    cvfh_list << orderedClouds[i].first + "_" + to_string(output->points.size())<<"\n";  //write name of the object and how many histogram it holds 
    cvfh_list.close();
    for (size_t n=0; n < output->points.size(); ++n)
      descriptors.push_back(output->points[n]);
    print_highlight("CVFH estimation... \t\t\t\t");
    print_value("[%d/%d]\r", i+1, orderedClouds.size());
    cout<<std::flush;
  }
  flann::Matrix<float> cvfh (new float[descriptors.points.size() * 308], descriptors.points.size(), 308);
  for (size_t i=0; i< cvfh.rows; ++i)
    for (size_t k=0; k< cvfh.cols; ++k)
      cvfh[i][k]= descriptors.points[i].histogram[k];
  flann::save_to_file (cvfh, data_dir.string() + "cvfh.h5", "CVFH Histograms");
 // flann::Index<CustomDistance<float> > cvfh_index (cvfh, flann::LinearIndexParams() ); //use our newly implemented distance function, with bruteforce search
 // cvfh_index.buildIndex();
 // cvfh_index.save (data_dir.string() + "cvfh.idx");
  print_value("\t\t\t\t\t\t\t\tOK\n");
  
  ////////////////////////// OUR-CVFH Computation ////////////////////////
  //
  //  Computes all OUR-CVFH descriptors and store them in flann data,
  //  objects may have more than one entry based on how many clusters
  //  are found.
  //
  descriptors.clear();
  ofstream ourcvfh_list;
  ourcvfh_list.open((data_dir.string()+ "ourcvfh.list").c_str(), fstream::trunc); //truncate contents of the file, if it already exists
  ourcvfh_list.close();
  j=0;
  for (size_t i=0; i<orderedClouds.size(); ++i, j+=10)
  {
    if (j==360)
      j=0;
    float vx,vy,vz;
    vx = cos(22*D2R)*sin(j*D2R);
    vy = sin(22*D2R);
    vz = cos(22*D2R)*cos(j*D2R);
    OURCVFHEstimation<PointXYZ, Normal, VFHSignature308> ourcvfhE;
    PointCloud<PointXYZ>::Ptr input (new PointCloud<PointXYZ>);
    copyPointCloud (orderedClouds[i].second, *input);
    PointCloud<Normal>::Ptr input_N (new PointCloud<Normal>);
    copyPointCloud (normals[i], *input_N);
    PointCloud<VFHSignature308>::Ptr output (new PointCloud<VFHSignature308>);
    search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);
    ourcvfhE.setInputCloud (input);
    ourcvfhE.setInputNormals (input_N);
    ourcvfhE.setSearchMethod (tree);
    ourcvfhE.setViewPoint (vx, vy, vz);
    //Set to true to normalize the bins of the histogram using the total number of points.
    //Doing this will render ourcvfh invariant to scale, but the authors encourage the opposite.
    ourcvfhE.setNormalizeBins(false); //leaving it false for now
    //ourcvfhE.setRadiusNormals (0.015); //1.5cm used in normal computation step
    //Set maximum allowable deviation of the normals, in the region segmentation step
    ourcvfhE.setEPSAngleThreshold(7.5f*D2R);  
    //Set maximum disparity between curvatures, in the region segmentation step
    ourcvfhE.setCurvatureThreshold(0.025f); //according to paper
    //Clustering parameters
    ourcvfhE.setClusterTolerance (0.01f); //1 cm all points should have a density of 3 mm
    ourcvfhE.setMinPoints (50); //at least 30 points to consider a cluster to be valid
    // Set the minimum axis ratio between the SGURF axes. At the disambiguation phase,
	  // this will decide if additional Reference Frames need to be created, if ambiguous.
  	ourcvfhE.setAxisRatio(0.95);
    //Set the min disambiguation axis value to generate several SGURFs for the cluster when disambiguition is difficult.
    //ourcvfhE.setMinAxisValue(0.01);
    //Set the refinement factor for the clusters
    ourcvfhE.setRefineClusters (1.0); //dont refine clusters
    //Compute method
    ourcvfhE.compute ( *output );
    ourcvfh_list.open ((data_dir.string() + "ourcvfh.list").c_str(), fstream::out | fstream::app);  //append mode
    ourcvfh_list << orderedClouds[i].first + "_" + to_string(output->points.size())<<"\n";    
    ourcvfh_list.close();
    for (size_t n=0; n < output->points.size(); ++n)
      descriptors.push_back(output->points[n]);
    print_highlight("OUR-CVFH estimation... \t\t\t");
    print_value("[%d/%d]\r", i+1, orderedClouds.size());
    cout<<std::flush;
  }
  flann::Matrix<float> ourcvfh (new float[descriptors.points.size() * 308], descriptors.points.size(), 308);
  for (size_t i=0; i< ourcvfh.rows; ++i)
    for (size_t k=0; k< ourcvfh.cols; ++k)
      ourcvfh[i][k]= descriptors.points[i].histogram[k];
  flann::save_to_file (ourcvfh, data_dir.string() + "ourcvfh.h5", "OURCVFH Histograms");
 // flann::Index<CustomDistance<float> > ourcvfh_index (ourcvfh, flann::LinearIndexParams() ); //use the same custom distance as CVFH and also bruteforce search
 // ourcvfh_index.buildIndex();
 // ourcvfh_index.save (data_dir.string() + "ourcvfh.idx");
  print_value("\t\t\t\t\t\t\t\tOK\n");
/*
  /////////////////////////// GFPFH Computation ////////////////////////
  //
  // Classifies points in a cloud based on FPFH descriptors similarity
  // and point proximity, then computes a GlobalFPFH of the cloud.
  // Finally stores them in Flann data to disk.
  //
  flann::Matrix<float> gfpfh (new float[orderedClouds.size() * 16], orderedClouds.size(), 16);
  for (size_t i=0; i<orderedClouds.size(); ++i)
  {
    FPFHEstimationOMP<PointXYZRGBA, Normal, FPFHSignature33> fpfhE;
    PointCloud<PointXYZRGBA>::Ptr input (new PointCloud<PointXYZRGBA>);
    copyPointCloud(orderedClouds[i].second, *input);
    PointCloud<Normal>::Ptr input_N (new PointCloud<Normal>);
    copyPointCloud(normals[i], *input_N);
    search::KdTree<PointXYZRGBA>::Ptr tree (new search::KdTree<PointXYZRGBA>);
    PointCloud<GFPFHSignature16>::Ptr output (new PointCloud<GFPFHSignature16>);
    fpfhE.setInputCloud (input);
    fpfhE.setInputNormals (input_N);
    fpfhE.setSearchMethod (tree);
    fpfhE.setRadiusSearch (0.009); //9mm
    fpfhE.compute (*fpfh);
    PointCloud<PointXYZL>::Ptr input_L (new PointCloud<PointXYZL>);
    copyPointCloud (*input, *input_L);
    for (size_t n=0; n < fpfh->points.size(); ++n)
      input_L->points[n].label = n; //save the index of the fpfh into the point it belongs for later use
    // Start point classification based on CEC
    ConditionalEuclideanClustering<PointXYZL> cec(true);
    cec.setInputCloud (input_L);
    cec.setClusterTolerance (0.01); //1 cm (all points should have a density of 3mm)
    cec.setMinClusterSize (50);
    cec.setMaxClusterSize (input_L->points.size());
    //Points within the tolerance need to validate the condition function to be part of the same cluster
    cec.setConditionFunction (&enforceFPFHSimilarity);
    IndicesClusters clusters;
    IndicesClustersPtr small, big;
    cec.segment (clusters);
    cec.getRemovedClusters(small, big); //also get removed clusters due to size constraints
    // DEBUG to visualize some cloud clusters
    if (i%35==0 && visual_debug)  
    {
      vector<PointCloud<PointXYZ> > clus;
      clus.resize(clusters.size());
      for (int m=0; m < clusters.size(); ++m)
        for (int j=0; j < clusters[m].indices.size(); ++j)
        {
          PointXYZ t;
          t.x = input_L->points[clusters[m].indices[j]].x;
          t.y = input_L->points[clusters[m].indices[j]].y;
          t.z = input_L->points[clusters[m].indices[j]].z;
          clus[m].push_back(t);
        }
      if (!small->empty())
      {
        clus.resize(clus.size()+1);
        for (int m=0; m < small->size(); ++m)
          for (int j=0; j < small->at(m).indices.size(); ++j)
          {
            PointXYZ p;
            p.x = input_L->points[small->at(m).indices[j]].x;
            p.y = input_L->points[small->at(m).indices[j]].y;
            p.z = input_L->points[small->at(m).indices[j]].z;
            clus[clus.size()-1].push_back(p);
          }
      }
      pcl::visualization::PCLVisualizer viewer;
      viewer.addPointCloud (input, "cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");
      for (int k=0; k < clus.size(); ++k)
      {
        int red, blue;
        red = 255 - static_cast<int>(floor(255/clus.size()*k));
        blue = static_cast<int>(floor(255/clus.size()*k));
        pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> ch (clus[k].makeShared(), red,0,blue);
        viewer.addPointCloud (clus[k].makeShared(), ch, to_string(k).c_str());
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, to_string(k).c_str());
      }
      viewer.setBackgroundColor(0.9,.95,0.9);
      while (!viewer.wasStopped ())
      {
        viewer.spinOnce ();
      }
      viewer.close();
    }  
    PointCloud<PointXYZL>::Ptr input_G (new PointCloud<PointXYZL>);
    for (int m=0; m < clusters.size(); ++m)
      for (int j=0; j < clusters[m].indices.size(); ++j)
      {
        PointXYZL p;
        p.x = input->points[clusters[m].indices[j]].x;
        p.y = input->points[clusters[m].indices[j]].y;
        p.z = input->points[clusters[m].indices[j]].z;
        p.label = m; //Labelling Points based on the cluster they belong to
        input_G->push_back(p);
      }
    GFPFHEstimation<PointXYZRGBA, PointXYZL, GFPFHSignature16> gfpfhE;
    if (!small->empty())
    {
      gfpfhE.setNumberOfClasses (clusters.size()+1);  //how many label classes
      for (int m=0; m < small->size(); ++m)
        for (int j=0; j < small->at(m).indices.size(); ++j)
        {
          PointXYZL p;
          p.x = input->points[small->at(m).indices[j]].x;
          p.y = input->points[small->at(m).indices[j]].y;
          p.z = input->points[small->at(m).indices[j]].z;
          p.label = clusters.size(); //small and big clusters get last label, there can't be a too big cluster though 
          input_G->push_back(p);
        }
    }
    else
      gfpfhE.setNumberOfClasses (clusters.size());  //how many label classes
    gfpfhE.setInputCloud(input);
    gfpfhE.setSearchMethod (tree);
    gfpfhE.setInputLabels (input_G);
    gfpfhE.setOctreeLeafSize (0.012); //1.2cm
    gfpfhE.compute(*output);

    print_highlight("GFPFH estimation... \t\t\t\t");
    print_value("[%d/%d]\r", i+1, orderedClouds.size());
    cout<<std::flush;
    for (size_t k=0; k < gfpfh.cols; ++k)
      gfpfh[i][k] = output->points[0].histogram[k];   
  } 
  flann::save_to_file (gfpfh, data_dir.string() + "gfpfh.h5", "GFPFH Histograms");
  flann::Index<ChiSquareDistance<float> > gfpfh_index (gfpfh, flann::KDTreeIndexParams(4)); // norm ?? no information on paper, using chisquare
  gfpfh_index.buildIndex();
  gfpfh_index.save (data_dir.string() + "gfpfh.idx");
  print_value("\t\t\t\t\t\t\t\tOK\n"); 
  */
}
