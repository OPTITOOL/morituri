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
#include <concurrencpp/concurrencpp.h>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <ogr_api.h>
#include <osmium/io/any_input.hpp>
#include <osmium/io/any_output.hpp>
#include <osmium/io/output_iterator.hpp>
#include <osmium/io/reader_with_progress_bar.hpp>
#include <osmium/object_pointer_collection.hpp>
#include <osmium/osm/object_comparisons.hpp>
#include <ranges>

#include "converter/AdminBoundariesConverter.hpp"
#include "converter/BuildingConverter.hpp"
#include "converter/CityConverter.hpp"
#include "converter/HamletConverter.hpp"
#include "converter/HouseNumberConverter.hpp"
#include "converter/LanduseConverter.hpp"
#include "converter/RailwayConverter.hpp"
#include "converter/RestAreaConverter.hpp"
#include "converter/StreetConverter.hpp"
#include "converter/WaterConverter.hpp"

#include "../comm2osm_exceptions.hpp"
#include "navteq_plugin.hpp"

/*

 Convert navteq data into routable OSM files.

 */

navteq_plugin::navteq_plugin(const std::filesystem::path &executable_path)
    : base_plugin::base_plugin("Navteq Plugin", executable_path) {}

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

  // check HERE-Contrycodes
  if (!countriesToConvert.empty()) {
    // get the 2 leading chars of the directory
    std::string country = dir.filename().string().substr(0, 2);
    auto isoContryCode = HERE_REGION_ISO_MAPPING.find(country);
    if (isoContryCode == HERE_REGION_ISO_MAPPING.end()) {
      BOOST_LOG_TRIVIAL(error)
          << "Country code not found in mapping: " << country;
      return std::nullopt;
    }

    if (countriesToConvert.contains(isoContryCode->second)) {
      foundCountries.insert(isoContryCode->second);
    } else {
      return std::nullopt;
    }
  }

  if (hasPackedData) {
    return std::optional<std::filesystem::path>(tarFile);
  } else {
    return std::optional<std::filesystem::path>(dir);
  }
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

  // init the langfile of the converter
  Converter::parse_lang_code_file(executable_path);

  concurrencpp::runtime runtime;

  std::vector<concurrencpp::result<void>> results;

  auto executor = runtime.thread_pool_executor();

  BOOST_LOG_TRIVIAL(info) << "Max threads: "
                          << executor->max_concurrency_level();

  // run converters
  for (auto &dir : dataDirs) {

    results.emplace_back(executor->submit([this, &dir, &writer]() {
      BOOST_LOG_TRIVIAL(debug)
          << "Start converting AdminBoundariesConverter " << dir;
      AdminBoundariesConverter(executable_path).convert(dir, writer);
    }));
    results.emplace_back(executor->submit([this, &dir, &writer]() {
      BOOST_LOG_TRIVIAL(debug) << "Start converting StreetConverter " << dir;
      StreetConverter(executable_path).convert(dir, writer);
    }));
    results.emplace_back(executor->submit([this, &dir, &writer]() {
      BOOST_LOG_TRIVIAL(debug) << "Start converting LanduseConverter " << dir;
      LanduseConverter(executable_path).convert(dir, writer);
    }));
    results.emplace_back(executor->submit([this, &dir, &writer]() {
      BOOST_LOG_TRIVIAL(debug) << "Start converting CityConverter " << dir;
      CityConverter(executable_path).convert(dir, writer);
    }));
    results.emplace_back(executor->submit([this, &dir, &writer]() {
      BOOST_LOG_TRIVIAL(debug) << "Start converting HamletConverter " << dir;
      HamletConverter(executable_path).convert(dir, writer);
    }));
    results.emplace_back(executor->submit([this, &dir, &writer]() {
      BOOST_LOG_TRIVIAL(debug) << "Start converting BuildingConverter " << dir;
      BuildingConverter(executable_path).convert(dir, writer);
    }));
    results.emplace_back(executor->submit([this, &dir, &writer]() {
      BOOST_LOG_TRIVIAL(debug) << "Start converting RestAreaConverter " << dir;
      RestAreaConverter(executable_path).convert(dir, writer);
    }));
    results.emplace_back(executor->submit([this, &dir, &writer]() {
      BOOST_LOG_TRIVIAL(debug) << "Start converting RailwayConverter " << dir;
      RailwayConverter(executable_path).convert(dir, writer);
    }));
    results.emplace_back(executor->submit([this, &dir, &writer]() {
      BOOST_LOG_TRIVIAL(debug) << "Start converting WaterConverter " << dir;
      WaterConverter(executable_path).convert(dir, writer);
    }));
    results.emplace_back(executor->submit([this, &dir, &writer]() {
      BOOST_LOG_TRIVIAL(debug)
          << "Start converting HouseNumberConverter " << dir;
      HouseNumberConverter(executable_path).convert(dir, writer);
    }));
  }

  auto allDone =
      concurrencpp::when_all(executor, results.begin(), results.end()).run();

  allDone.get();

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

  for (const auto entity :
       {osmium::osm_entity_bits::node, osmium::osm_entity_bits::way,
        osmium::osm_entity_bits::relation}) {
    copyType(writer, tmpfile, entity);
  }

  writer.close();

  std::filesystem::remove(output_path.parent_path().append("tmp.pbf"));
}

void navteq_plugin::copyType(osmium::io::Writer &writer, osmium::io::File &file,
                             osmium::osm_entity_bits::type bits) {
  std::vector<osmium::memory::Buffer> data;
  osmium::ObjectPointerCollection objects;

  osmium::io::ReaderWithProgressBar reader(true, file, bits);
  while (osmium::memory::Buffer buffer = reader.read()) {
    osmium::apply(buffer, objects);
    data.push_back(std::move(buffer));
  }
  reader.close();

  objects.sort(osmium::object_order_type_id_version());

  auto out = osmium::io::make_output_iterator(writer);
  std::copy(objects.begin(), objects.end(), out);
}

void navteq_plugin::setBoundingBox(double minX, double minY, double maxX,
                                   double maxY) {
  boundingBox.MinX = minX;
  boundingBox.MinY = minY;
  boundingBox.MaxX = maxX;
  boundingBox.MaxY = maxY;
}

void navteq_plugin::setCountries(const std::set<std::string> &countries) {
  countriesToConvert = countries;
}

void navteq_plugin::setDebug(bool _debug) { debug = _debug; }