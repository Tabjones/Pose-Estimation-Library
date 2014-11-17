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
  /*
  prova.setQuery("funnel_20_30", cloud);
  prova.setDatabase("../../../Objects/Database");
  prova.generateLists();
  prova.printCandidates();
  prova.refineCandidates();
  prova.setParam("progBisection", 0); //try bruteforce now
  prova.refineCandidates();
  prova.setParam("wrongkey", 3); //check errors on wrong key
  prova.setParam("filtering", -0.5); //check errors on wrong value
  PoseEstimation prova2;
  prova2.setQueryViewpoint(0,0,1);
  prova2.estimate("object_0", cloud, "../../../Objects/Database");
  prova2.printEstimation();
  PoseEstimation prova3("../config/parameters.conf");
  PoseDB db;
  db.load("../../../Objects/Database");
  prova3.setQueryViewpoint(1,1,1);
  prova3.estimate("obj_0", cloud, db);
  prova3.printEstimation();
  */
  //Database tests
  PoseDB test;
  parameters p;
  p["computeViewpointFromName"] = 1;
  p["useSOasViewpoint"]= 0;
  boost::shared_ptr<parameters> par;
  par = boost::make_shared<parameters>(p);
  test.create("../../../Acquisitions_old/Round1", par);
  test.save("../../../Database_Round1/");
  PoseDB test2("../../../Database_Round1/");
  prova.setParam("progBisection", 1);
  prova.setParam("computeViewpointFromName", 1);
  prova.setParam("useSOasViewpoint", 0);
  prova.resetViewpoint();
  prova.estimate("object_23_50", cloud, test2);
  prova.printCandidates();
  prova.printEstimation();
  prova.saveEstimation("Results/prova.estimation");
  return 1;
}
