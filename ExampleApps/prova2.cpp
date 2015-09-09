#include <pel.h>

////////////////////////////////////////////////////
//////////////////  Main  //////////////////////////
////////////////////////////////////////////////////
int
main (int argc, char *argv[])
{
  printVersion();
  if (argc !=2)
  {
    cout<<"Need at least 1 parameter, the path where the clouds are stored"<<endl;
    return -1;
  }
  string path (argv[1]);
  boost::filesystem::path db_path (path);
  boost::filesystem::path db_path2 (path + "/database");
  PoseEstimation prova;
  prova.setParam("verbosity", 2);
  PoseDB db;
  db.create(db_path, prova.getParams() );
  db.save(db_path2);
  cout<<"Done!!"<<endl<<"Database saved on "<< db_path2.string().c_str() <<endl;
  return 1;
}
