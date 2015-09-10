#include <pel/database_creator.h>
#include <pel/database_io.h>
#include <pel/database.h>
#include <pcl/console/parse.h>
#include <string>
#include <vector>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>

void
show_help(char* prog_name)
{
  //trim and split program name string
  std::string pn = prog_name;
  boost::trim(pn);
  std::vector<std::string> vst;
  boost::split (vst, pn, boost::is_any_of("/\\.."), boost::token_compress_on);
  pn = vst.at( vst.size() -1);
  print_highlight ("%s takes a path containing pcd files of objects viewes to assemble a PEL Database from them.\n", pn.c_str());
  print_highlight ("Usage:\t%s [SourceDir] [OutputDir] [Options]\n", pn.c_str());
  print_highlight ("Options are:\n");
  //TODO
  print_value ("\t-h, --help");
  print_info (":\t\tShow this help screen and quit.\n");
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
