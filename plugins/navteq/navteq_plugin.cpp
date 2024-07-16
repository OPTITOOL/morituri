/*
 * navteq_plugin.cpp
 *
 *  Created on: 11.06.2015
 *      Author: philip
 */

#include <boost/algorithm/string/join.hpp>
#include <boost/log/trivial.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/iterator_range.hpp>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <ogr_api.h>
#include <osmium/io/any_input.hpp>
#include <osmium/io/any_output.hpp>
#include <ranges>

#include "converter/AdminBoundariesConverter.hpp"
#include "converter/BuildingConverter.hpp"
#include "converter/CityConverter.hpp"
#include "converter/HamletConverter.hpp"
#include "converter/LanduseConverter.hpp"
#include "converter/RailwayConverter.hpp"
#include "converter/RestAreaConverter.hpp"
// #include "converter/StreetConverter.hpp"
#include "converter/WaterConverter.hpp"

#include "../comm2osm_exceptions.hpp"
#include "navteq_plugin.hpp"

/*

 Convert navteq data into routable OSM files.

 */

navteq_plugin::navteq_plugin(const std::filesystem::path &executable_path)
    : base_plugin::base_plugin("Navteq Plugin", executable_path) {

  converter.emplace_back(new AdminBoundariesConverter(executable_path));
  //  converter.emplace_back(new StreetConverter(executable_path));
  converter.emplace_back(new LanduseConverter(executable_path));
  converter.emplace_back(new CityConverter(executable_path));
  converter.emplace_back(new HamletConverter(executable_path));
  converter.emplace_back(new BuildingConverter(executable_path));
  converter.emplace_back(new RestAreaConverter(executable_path));
  converter.emplace_back(new RailwayConverter(executable_path));
  converter.emplace_back(new WaterConverter(executable_path));
}

navteq_plugin::~navteq_plugin() {}

bool navteq_plugin::is_valid_format(std::string filename) {
  if (filename.length() > 3)
    filename = filename.substr(filename.length() - 3);
  for (auto i : filename)
    i = std::tolower(i);
  if (filename == "pbf")
    return true;
  if (filename == "osm")
    return true;
  return false;
}

std::optional<std::filesystem::path>
navteq_plugin::check_files(const std::filesystem::path &dir) {

  bool isHereDatatDir = false;
  bool hasPackedData = false;
  std::filesystem::path tarFile;
  for (const auto &entry : std::filesystem::directory_iterator(dir) |
                               std::views::filter([](auto &entry) {
                                 return entry.is_regular_file();
                               })) {

    if (entry.path().filename().string().ends_with(".PROD.csv")) {
      isHereDatatDir = true;
    }

    if (entry.path().filename().string().ends_with(".tar.gz")) {
      hasPackedData = true;
      tarFile = entry.path();
    }
  }

  if (!isHereDatatDir)
    return std::nullopt;

  if (hasPackedData) {
    return std::optional<std::filesystem::path>(tarFile);
  }

  // check if the PROD.csv file exists

  // check HERE-Contrycodes

  // if there is a tar.gz --> use vsitar to extract the files

  // otherwise check id there are unpacked files

  return std::nullopt;
}

bool navteq_plugin::check_input(const std::filesystem::path &input_path,
                                const std::filesystem::path &output_file) {
  if (!std::filesystem::is_directory(input_path))
    throw(std::runtime_error("directory " + input_path.string() +
                             " does not exist"));

  if (!output_file.empty()) {
    std::filesystem::path output_path = output_file.parent_path();
    if (!std::filesystem::is_directory(output_path))
      throw(std::runtime_error("output directory " + output_path.string() +
                               " does not exist"));
    if (!is_valid_format(output_file.string()))
      throw(format_error("unknown format for outputfile: " +
                         output_file.string()));
  }

  for (const auto &entry :
       std::filesystem::recursive_directory_iterator(input_path) |
           std::views::filter(
               [](auto &entry) { return entry.is_directory(); })) {

    // add path to dataDirs
    check_files(entry.path())
        .and_then([&](auto path) -> std::optional<std::filesystem::path> {
          dataDirs.push_back(path);
          return std::nullopt;
        });
  }

  if (!foundCountries.empty()) {
    BOOST_LOG_TRIVIAL(info)
        << "Countries found :" << boost::join(foundCountries, " ");
  }

  if (dataDirs.empty())
    return false;

  BOOST_LOG_TRIVIAL(info) << "dirs: ";
  for (auto &dir : dataDirs)
    BOOST_LOG_TRIVIAL(info) << dir;

  this->plugin_setup(input_path, output_file);

  return true;
}

void navteq_plugin::write_output() {}

void navteq_plugin::execute() {
  if (output_path.empty())
    throw std::exception();

  osmium::io::File outfile(
      output_path.parent_path().append("tmp.pbf").string());
  osmium::io::Header hdr;
  hdr.set("generator", "osmium");
  hdr.set("xml_josm_upload", "false");
  osmium::io::Writer writer(outfile, hdr, osmium::io::overwrite::allow);

  // run converters
  for (auto &c : converter)
    c->convert(dataDirs, writer);

  writer.close();

  BOOST_LOG_TRIVIAL(info) << "Start sorting PBF ";
  sortPBF();

  BOOST_LOG_TRIVIAL(info) << std::endl << "fin";
}

void navteq_plugin::sortPBF() {

  osmium::io::File outfile(output_path.string());
  osmium::io::Header hdr;
  hdr.set("generator", "morituri");
  hdr.set("xml_josm_upload", "false");

  auto now =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::stringstream ss;
  ss << std::put_time(std::gmtime(&now), "%FT%TZ");

  hdr.set("osmosis_replication_timestamp", ss.str());
  osmium::io::Writer writer(outfile, hdr, osmium::io::overwrite::allow);

  osmium::io::File tmpfile(
      output_path.parent_path().append("tmp.pbf").string());

  copyType(writer, tmpfile, osmium::osm_entity_bits::node);
  copyType(writer, tmpfile, osmium::osm_entity_bits::way);
  copyType(writer, tmpfile, osmium::osm_entity_bits::relation);

  writer.close();

  std::filesystem::remove(output_path.parent_path().append("tmp.pbf"));
}

void navteq_plugin::copyType(osmium::io::Writer &writer, osmium::io::File &file,
                             osmium::osm_entity_bits::type bits) {
  osmium::io::Reader reader(file, bits);
  while (osmium::memory::Buffer buffer = reader.read()) {
    writer(std::move(buffer));
  }
  reader.close();
}

void navteq_plugin::setBoundingBox(double minX, double minY, double maxX,
                                   double maxY) {
  boundingBox.MinX = minX;
  boundingBox.MinY = minY;
  boundingBox.MaxX = maxX;
  boundingBox.MaxY = maxY;
}

void navteq_plugin::setCountries(const std::vector<std::string> &countries) {
  countriesToConvert = countries;
}

void navteq_plugin::setDebug(bool _debug) { debug = _debug; }