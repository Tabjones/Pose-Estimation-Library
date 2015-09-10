#include <pel/database/database_creator.h>
#include <pel/database/database_io.h>
#include <pel/database/database.h>
#include <pcl/console/parse.h>
#include <string>
#include <vector>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/filesystem/path.hpp>

using namespace pcl::console;

bool load(false), overwrite(false);
boost::filesystem::path in_path, out_path;
boost::filesystem::path p_path;

void
show_help(char* prog_name)
{
  //trim and split program name string
  std::string pn = prog_name;
  boost::trim(pn);
  std::vector<std::string> vst;
  boost::split (vst, pn, boost::is_any_of("/\\.."), boost::token_compress_on);
  pn = vst.at( vst.size() -1);
  print_highlight ("%s takes a path containing pcd files of objects viewes to assemble a PEL Database from them, using default or specified parameters.\n", pn.c_str());
  print_highlight ("Usage:\t%s [SourceDir] [OutputDir] [Options]\n", pn.c_str());
  print_highlight ("Options are:\n");
  print_value ("\t-h, --help");
  print_info (":\t\tShow this help screen and quit.\n");
  print_value ("\t--load <path>");
  print_info (":\t\tLoad a set of configuration parameters from a yaml file in <path>\n");
  print_value ("\t-w");
  print_info (":\t\t\tOverwrite <OutputDir> even if it already exists.\n");
}

void
parse_command_line(int argc, char* argv[])
{
  if (find_switch (argc, argv, "-h") || find_switch (argc, argv, "--help"))
  {
    show_help(argv[0]);
    exit(0);
  }
  if (find_switch (argc, argv, "-w"))
    overwrite = true;
  std::string param_path;
  parse_argument (argc, argv, "--load", param_path);
  p_path = param_path;
  if (!boost::filesystem::exists(p_path) || !boost::filesystem::is_regular_file(p_path))
  {
    print_warn("Invalid path for parameters loading! Ignoring...\n");
    load = false;
  }
  else
    load=true;
  in_path = argv[1];
  out_path = argv[2];
  if (!boost::filesystem::exists(in_path) || !boost::filesystem::is_directory(in_path))
  {
    print_error("Invalid path for source clouds. Cannot continue...\n");
    exit(0);
  }
}

////////////////////////////////////////////////////
//////////////////  Main  //////////////////////////
////////////////////////////////////////////////////
int
main (int argc, char *argv[])
{
  //take care of command line...
  if (argc <3)
  {
    print_error("Need at least 2 parameters: [SourceDir] and [OutputDir], in this order.\n");
    show_help(argv[0]);
    exit(0);
  }
  parse_command_line(argc, argv);

  //to business!
  //Create an empty database and
  pel::Database db;
  //a creator
  pel::DatabaseCreator creator;
  if (load)
  {
    //load provided parameters instead of default ones
    creator.loadParamsFromFile(p_path);
  }
  //be verbose
  creator.setParam("verbosity", 2);
  //inform user what parameters we are going to use
  creator.printAllParams();

  //ok, start Database creation
  db = creator.create(in_path);
  //go get coffee...

  //Everything went fine, lets save the database where the user told us
  //Writer object
  pel::DatabaseWriter writer;
  //write it!
  writer.save(out_path, db, overwrite);

  //bye
  return 1;
}
