#include <pel.h>

////////////////////////////////////////////////////
//////////////////  Main  //////////////////////////
////////////////////////////////////////////////////
int
main (int argc, char *argv[])
{
  printVersion();
  PointCloud<PointXYZRGBA> cloud;
  boost::filesystem::path db_path ("/home/tabjones/ObjectDB/DB_clouds");
  boost::filesystem::path db_path2 ("/home/tabjones/ObjectDB/DB");
  pcl::io::loadPCDFile(argv[1], cloud);
  PoseEstimation prova, prova2;
  prova.setParam("verbosity", 2);
  prova2.setParam("verbosity", 2);
  PoseDB db, db2;
  //db.create(db_path, prova.getParams() );
  //db.save(db_path2);
  db.load(db_path2);
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
  prova2.setParam("progBisection", 0);
  prova2.setParam("rmseThreshold", 0.005);
  prova2.estimate("object_23_50", cloud, db);
  prova2.viewEstimation();
  return 1;
}
