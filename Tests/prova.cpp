#include <pel.h>

////////////////////////////////////////////////////
//////////////////  Main  //////////////////////////
////////////////////////////////////////////////////
int
main (int argc, char *argv[])
{
  printVersion();
  PointCloud<PointXYZRGBA> cloud;
  boost::filesystem::path db_path ("");
  pcl::io::loadPCDFile(argv[1], cloud);
  PoseEstimation prova;
  prova.setParam("verbosity", 2);
  prova.setParam("computeViewpointFromName",0);
  prova.setParam("useSOasViewpoint",1);
  prova.setParam("progItera",10);
  prova.setDatabase(db_path);
  //test.create("/home/pacman/Dropbox/ObjectDB/Round1", prova.getParams() );
  //test.save("/media/pacman/storage/PointClouds/Database_Round1");
  test.load("/media/pacman/storage/PointClouds/Database_Round1");
  prova.estimate("object_23_50", cloud, test);
  prova.printCandidates();
  prova.printEstimation();
  prova.viewEstimation();
  //prova.saveEstimation("Results/prova.estimation");
  //prova.saveParams("provaconf");
  //prova.saveCandidates("Results/cand.list");
  return 1;
}
