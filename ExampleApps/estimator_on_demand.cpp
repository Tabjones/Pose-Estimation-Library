#include <pel/pe_progressive_bisection.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <string>
#include <vector>

using namespace pcl;
using namespace pcl::console;
using namespace pel;

bool vis(true);
float thresh(0.005f);
float frac(0.5f);
unsigned int itera(5);
std::string target_filename, target_name;

void
show_help(char* prog_name)
{
  //trim and split program name string
  std::string pn = prog_name;
  boost::trim(pn);
  std::vector<std::string> vst;
  boost::split (vst, pn, boost::is_any_of("/\\.."), boost::token_compress_on);
  pn = vst.at( vst.size() -1);
  print_highlight ("%s takes a pcd file of a Target cloud to run Pose Estimation procedure with Progressive Bisection, using the passed Database.\n", pn.c_str());
  print_highlight ("Usage:\t%s [DatabaseDir] [TargetCloudPCD] [Options]\n", pn.c_str());
  print_highlight ("Options are:\n");
  print_value ("\t-h, --help");
  print_info (":\t\tShow this help screen and quit.\n");
  print_value ("\t--no-vis");
  print_info (":\t\tDisable visualization of Pose Estimation.\n");
  print_value ("\t-t <float>");
  print_info (":\t\tChange current RMSE Threshold to <float> value specified. (Default: 0.005)\n");
  print_value ("\t-f <float>");
  print_info (":\t\tChange current Bisection Fraction to <float> value specified. I.E. the fraction of the list to keep on every iterations. (Default 0.5)\n");
  print_value ("\t-s <uint>");
  print_info(":\t\tChange how many ICP iterations to perform on each step of progressive bisection. (Default 5)\n");
}
void
parse_command_line(int argc, char* argv[])
{
  if (find_switch (argc, argv, "-h") || find_switch (argc, argv, "--help"))
  {
    show_help(argv[0]);
    exit(0);
  }
  if (find_switch (argc, argv, "--no-vis"))
    vis = false;
  parse_argument (argc, argv, "-t", thresh);
  if (thresh <=0)
  {
    print_warn("Invalid negative value for -t option, resetting to default!\n");
    thresh = 0.005f;
  }
  parse_argument (argc, argv, "-f", frac);
  if (frac <=0 || frac >=1)
  {
    print_warn("Invalid value for -f option, resetting to default!\n");
    frac = 0.5f;
  }
  parse_argument (argc, argv, "-s", itera);
  if (itera <=0)
  {
    print_warn("Invalid negative value for -s option, resetting to default!\n");
    itera = 5;
  }
  //find target pcd file
  std::vector<int> file_idx = parse_file_extension_argument (argc, argv, ".pcd");
  if (file_idx.empty())
  {
    print_error ("No pcd file specified for target\n");
    exit (0);
  }
  target_filename = argv[file_idx.at(0)];
  boost::trim (target_filename);
  std::vector<std::string> vst;
  boost::split (vst, target_filename, boost::is_any_of("./\\.."), boost::token_compress_on);
  //get target name without path and extension
  target_name = vst.at( vst.size() -2);
}

////////////////////////////////////////////////////
//////////////////  Main  //////////////////////////
////////////////////////////////////////////////////
int
main (int argc, char *argv[])
{
  //take care of command line parameters
  if (argc <3 )
  {
    print_error("Need at least 2 parameters!\n");
    show_help(argv[0]);
    return (0);
  }
  parse_command_line (argc, argv);
  //create a new point cloud to store loaded Target, with and without color
  PointCloud<PointXYZ>::Ptr target (new PointCloud<PointXYZ>);
  PointCloud<PointXYZRGBA>::Ptr cloud (new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr cloud_raw (new PointCloud<PointXYZRGBA>);
  //Database path (it needs to be first argument)
  std::string dbp(argv[1]);

  //Load target
  if (pcl::io::loadPCDFile(target_filename, *cloud_raw) == 0)
  {
    Eigen::Vector3f offset (cloud_raw->sensor_origin_(0), cloud_raw->sensor_origin_(1), cloud_raw->sensor_origin_(2));
    Eigen::Quaternionf rot (cloud_raw->sensor_orientation_);
    pcl::transformPointCloud(*cloud_raw, *cloud, offset, rot);
    cloud->sensor_origin_.setZero();
    cloud->sensor_orientation_.setIdentity();
    copyPointCloud(*cloud, *target); //Dropping color
    //instantiate a pose estimation object
    pel::interface::PEProgressiveBisection pe;
    //set wanted parameters, others are left as default
    pe.setBisectionFraction(frac);
    pe.setRMSEThreshold(thresh);
    pe.setStepIterations(itera);
    pe.setParam("verbosity", 2);
    //Set target for pose estimation
    if (pe.setTarget(target, target_name))
    {//load and set the database
      if (pe.loadAndSetDatabase(dbp))
      {
        Candidate estimation;
        //perform pose estimation
        pe.estimate(estimation);
        //print result
        print_highlight("Target %s was estimated with %s with RMSE of %g\n",target_name.c_str(), estimation.getName().c_str(), estimation.getRMSE());
        print_highlight("Pose Estimation transformation is:\n");
        std::cout<<estimation.getTransformation();
        if (vis)
        {
          //Proceed to visualization
          pcl::visualization::PCLVisualizer viewer;
          //Transform Candidate with pose estimation transformation, so it aligns over Target
          PointCloud<PointXYZ>::Ptr aligned (new PointCloud<PointXYZ>);
          pcl::transformPointCloud(estimation.getCloud(), *aligned, estimation.getTransformation());
          aligned->sensor_origin_.setZero();
          aligned->sensor_orientation_.setIdentity();
          //make estimation cloud green
          pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> aligned_col_handl (aligned, 0, 255, 0);
          viewer.addPointCloud(cloud, "target");
          viewer.addPointCloud(aligned, aligned_col_handl, "estimation");
          viewer.addCoordinateSystem(0.08);
          viewer.addText("Target in full color, Pose Estimation in green", 20, 20, 22, 0,1,0);
          //Start viewer
          while (!viewer.wasStopped())
            viewer.spinOnce();
          viewer.close();
        }
      }
      else
      {
        print_error("Error loading Database. Database path must be first command line argument!\n");
        exit(0);
      }
    }
    else
    {
      print_error("Error setting a Target.\n");
      exit(0);
    }
  }
  return 1;
}
