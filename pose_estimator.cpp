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
  prova.setParam("verbosity", 2);
  //prova.printParams();
  prova.setQueryViewpoint(0,0,1);
  prova.setQuery("funnel_20_30", cloud);
  prova.setDatabase("../../../Objects/Database");
  prova.generateLists();
  prova.printCandidates();
  return 1;
}
