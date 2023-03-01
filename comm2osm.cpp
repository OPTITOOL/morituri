/*

 Convert Shapefiles into OSM files.

 */

#include <boost/algorithm/string.hpp>
#include <boost/locale.hpp>
#include <boost/log/trivial.hpp>
#include <boost/program_options.hpp>
#include <ogrsf_frmts.h>
#include <getopt.h>
#include <iostream>
#include <memory>
#include <vector>

#include "plugins/base_plugin.hpp"
#include "plugins/dummy/dummy_plugin.hpp"
#include "plugins/navteq/navteq_plugin.hpp"

boost::filesystem::path input_path, output_file;
static bool withTrunRestrictions = false;
std::vector<double> boundingBox;
std::vector<std::string> countries;
static bool debug = false;

void check_args_and_setup(int argc, char *argv[]) {
  std::string countriesString;
  std::string bb;

  // options
  boost::program_options::options_description desc{"Options"};
  desc.add_options()("help,h", "print usage message") //
      ("turn-restrictions,tr",
       boost::program_options::value<bool>(&withTrunRestrictions)
           ->default_value(false),
       "enable turn restrictions")
      //("bounding-box,bb", boost::program_options::value(&bb), "bounding box")
      ("countries,c",
       boost::program_options::value<std::string>(&countriesString),
       "list of countries (Iso codes seperate by space)") //
      ("debug,d",
       boost::program_options::value<bool>(&debug)->default_value(false),
       "switch to debug mode") //
      ("input-path", boost::program_options::value<std::string>()->required(),
       "input path") //
      ("output-file", boost::program_options::value<std::string>()->required(),
       "output file");

  boost::program_options::positional_options_description p;
  p.add("input-path", 1);
  p.add("output-file", 1);

  boost::program_options::variables_map vm;
  boost::program_options::store(
      boost::program_options::command_line_parser(argc, argv)
          .options(desc)
          .positional(p)
          .run(),
      vm);
  boost::program_options::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    exit(1);
  }

  if (vm.count("input-path")) {
    BOOST_LOG_TRIVIAL(info)
        << "Input paths are: " << vm["input-path"].as<std::string>() << "\n";
    input_path = vm["input-path"].as<std::string>();
  }

  if (vm.count("output-file")) {
    BOOST_LOG_TRIVIAL(info)
        << "Input paths are: " << vm["output-file"].as<std::string>() << "\n";
    output_file = vm["output-file"].as<std::string>();
  }

  if (vm.count("countries")) {
    BOOST_LOG_TRIVIAL(info) << "Countires: " << countriesString << "\n";
    // split country string
    boost::split(countries, countriesString, boost::is_any_of(" "),
                 boost::token_compress_on);
  }

  if (vm.count("bounding-box")) {
    // split bb

    if (boundingBox.size() != 4) {
      BOOST_LOG_TRIVIAL(error)
          << "Wrong size of paramter for the bounding box.";
      exit(1);
    }
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

    if (!countries.empty())
      plugin->setCountries(countries);

    plugin->setDebug(debug);

    if (!boundingBox.empty())
      plugin->setBoundingBox(boundingBox[0], boundingBox[1], boundingBox[2],
                             boundingBox[3]);

    if (plugin->check_input(input_path, output_file)) {
      plugin->setWithTurnRestrictions(withTrunRestrictions);

      BOOST_LOG_TRIVIAL(info) << "executing plugin " << plugin->get_name();
      plugin->execute();
    }
  }
}
