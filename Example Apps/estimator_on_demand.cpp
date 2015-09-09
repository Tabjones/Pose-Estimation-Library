#include <pel/pe_progressive_bisection.h>

using namespace pcl;
using namespace pel;
////////////////////////////////////////////////////
//////////////////  Main  //////////////////////////
////////////////////////////////////////////////////
int
main (int argc, char *argv[])
{
  if (argc !=3 )
  {
    std::cout<<"Need exactly 2 parameters!"<<std::endl<<"Usage: "<<argv[0]<<" [Target Cloud] [Database Dir]"<<std::endl;
    return -1;
  }
  PointCloud<PointXYZRGBA> cloud;
  std::string dbp(argv[2]);
  std::string obj(argv[1]);
  pcl::io::loadPCDFile(obj, cloud);
  cloud.sensor_origin_.setZero();
  cloud.sensor_orientation_.setIdentity();
  std::cout<<"Loaded Cloud"<<std::flush<<std::endl;
  PoseEstimation prova;
  prova.setParam("verbosity", 2);
  PoseDB db;
  //db.create(db_path, prova.getParams() );
  //db.save(db_path2);
  db.load(dbp);
  std::cout<<"Loaded DB"<<std::flush<<std::endl;
  //db2.create("/home/tabjones/ObjectDB/Round1", prova.getParams() );
  //db2.save("/home/tabjones/ObjectDB/DB_Round1_cm");
  //test.create("/home/tabjones/ObjectDB/Round1", prova.getParams() );
  //test.save("/home/tabjones/ObjectDB/DB_Round1");
  //test.load("/media/pacman/storage/PointClouds/Database_Round1");
//  prova.estimate("object_23_50", cloud, db);
  prova.estimate("object", cloud, db);
  std::cout<<"Estimated"<<std::flush<<std::endl;
  prova.printCandidates();
  prova.printEstimation();
  prova.viewEstimation();
  return 1;
}
