#include "PoseEstimation_interface.hpp"

////////////////////////////////////////////////////
//////////////////  Main  //////////////////////////
////////////////////////////////////////////////////
int
main (int argc, char *argv[])
{
  PointCloud<PointXYZRGBA> cloud;
  pcl::io::loadPCDFile(argv[1], cloud);
  PoseEstimation prova;
  string par = "verbosity";
  string name= "funnel_10_20";
  //prova.setParam(par, 2.0f);
  //prova.printParams();
  prova.setQuery(name, cloud);
  return 1;
}
