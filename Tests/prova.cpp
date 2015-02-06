#include <pel.h>

////////////////////////////////////////////////////
//////////////////  Main  //////////////////////////
////////////////////////////////////////////////////
int
main (int argc, char *argv[])
{
  printVersion();
  PointCloud<PointXYZRGBA> cloud;
  boost::filesystem::path db_path ("/home/tabjones/ObjectDB/DB_Round1");
  boost::filesystem::path db_path2 ("/home/tabjones/ObjectDB/DB_Round1_cm");
  pcl::io::loadPCDFile(argv[1], cloud);
  VoxelGrid <PointXYZRGBA> vgrid;
  vgrid.setInputCloud(cloud.makeShared());
  vgrid.setLeafSize(0.003, 0.003, 0.003);
  vgrid.setDownsampleAllData(true);
  //vgrid.filter (cloud);
  Eigen::Affine3f RxPi;
  RxPi = Eigen::AngleAxisf(3.145962, Eigen::Vector3f::UnitX() );
  //transformPointCloud(cloud, cloud, RxPi);
  PoseEstimation prova;
  prova.setParam("verbosity", 2);
  prova.setParam("computeViewpointFromName",0);
  prova.setParam("useSOasViewpoint",1);
  prova.setParam("progItera",10);
  prova.setParam("vgridLeafSize",0.003);
  prova.setParam("neRadiusSearch",0.015);
  PoseDB db, db2;
  db.load(db_path);
  db2.load(db_path2);
  //db2.create("/home/tabjones/ObjectDB/Round1", prova.getParams() );
  //db2.save("/home/tabjones/ObjectDB/DB_Round1_cm");
  //test.create("/home/tabjones/ObjectDB/Round1", prova.getParams() );
  //test.save("/home/tabjones/ObjectDB/DB_Round1");
  //test.load("/media/pacman/storage/PointClouds/Database_Round1");
//  prova.estimate("object_23_50", cloud, db);
  prova.estimate("object_23_50", cloud, db);
  prova.printCandidates();
  prova.printEstimation();
  prova.viewEstimation();
  return 1;
}
