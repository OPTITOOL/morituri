/*

 Convert Shapefiles into OSM files.

 */

#include <boost/locale.hpp>
#include <gdal/ogrsf_frmts.h>
#include <getopt.h>
#include <iostream>
#include <memory>
#include <vector>

#include "plugins/base_plugin.hpp"
#include "plugins/dummy/dummy_plugin.hpp"
#include "plugins/navteq/navteq_plugin.hpp"

boost::filesystem::path input_path, output_file;
static int withTrunRestrictions = 0;

void print_help() {
  std::cout << "comm2osm [OPTIONS] [INFILE [OUTFILE]]\n\n"
            << "If INFILE or OUTFILE is not given stdin/stdout is assumed.\n"
            << "File format is autodetected from file name suffix.\n"
            //			<< "Use -t option to force file format.\n"
            << "\nFile format:\n"
            << "  (default)  XML encoding\n"
            << "  pbf        binary PBF encoding\n"
            << "  opl        OPL encoding\n"
            << "\nFile compression\n"
            << "  gz         compressed with gzip\n"
            << "  bz2        compressed with bzip2\n"
            << "\nOptions:\n"
            << "  -h, --help                This help message\n"
            << "  -t, --to-format=FORMAT    Output format\n"
            << "  --turn-restrictions   Convert turn restrictions\n";
}

void check_args_and_setup(int argc, char *argv[]) {
  // options
  static struct option long_options[] = {
      {"turn-restrictions", no_argument, &withTrunRestrictions, 1},
      {"help", no_argument, 0, 'h'},
      {0, 0}};

  while (true) {
    int c = getopt_long(argc, argv, "dhf:t:", long_options, 0);
    if (c == -1) {
      break;
    }

    switch (c) {
    case 0:
      break;
    case 'h':
      print_help();
      exit(0);
    default:
      exit(1);
    }
  }

  int remaining_args = argc - optind;
  if (remaining_args < 2) {
    std::cerr << "Usage: " << argv[0] << " [OPTIONS] [INFILE [OUTFILE]]"
              << std::endl;
    exit(1);
  } else if (remaining_args == 2) {
    input_path = boost::filesystem::path(argv[optind]);
    output_file = boost::filesystem::path(argv[optind + 1]);
  } else if (remaining_args == 1) {
    input_path = boost::filesystem::path(argv[optind]);
  }
}

int main(int argc, char *argv[]) {

  boost::locale::generator gen;
  auto loc = gen("");
  std::locale::global(loc);

  GDALAllRegister();
  check_args_and_setup(argc, argv);

  std::vector<std::unique_ptr<base_plugin>> plugins;

  boost::filesystem::path executable_path(argv[0]);

  // plugins.emplace_back(std::make_unique<dummy_plugin>());
  plugins.emplace_back(std::make_unique<navteq_plugin>(executable_path));

  for (auto &plugin : plugins) {
    if (plugin->check_input(input_path, output_file)) {
      plugin->setWithTurnRestrictions(withTrunRestrictions);
      std::cout << "executing plugin " << plugin->get_name() << std::endl;
      plugin->execute();
    }
  }
}
