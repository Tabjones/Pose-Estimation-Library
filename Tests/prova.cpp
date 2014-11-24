#include <PoseEstimation_interface.h>

////////////////////////////////////////////////////
//////////////////  Main  //////////////////////////
////////////////////////////////////////////////////
int
main (int argc, char *argv[])
{
  printVersion();
  PointCloud<PointXYZRGBA> cloud;
  pcl::io::loadPCDFile(argv[1], cloud);
  PoseEstimation prova;
  prova.setParam("verbosity", 2);
  prova.setParam("computeViewpointFromName",1);
  prova.setParam("useSOasViewpoint",0);
  PoseDB test;
  //test.create("../../Acquisitions_old/Round1", prova.getParams() );
  //test.save("../../Database_Round1");
  test.load("../../Database_Round1");
  prova.estimate("object_23_50", cloud, test);
  prova.printCandidates();
  prova.printEstimation();
  //prova.saveEstimation("Results/prova.estimation");
  //prova.saveParams("provaconf");
  //prova.saveCandidates("Results/cand.list");
  return 1;
}
