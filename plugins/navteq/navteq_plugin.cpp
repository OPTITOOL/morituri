/*
 * navteq_plugin.cpp
 *
 *  Created on: 11.06.2015
 *      Author: philip
 */

#include <boost/algorithm/string/join.hpp>
#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/iterator_range.hpp>
#include <exception>
#include <iomanip>
#include <ogr_api.h>
#include <osmium/io/any_input.hpp>
#include <osmium/io/any_output.hpp>

#include "converter/RailwayConverter.hpp"
#include "converter/WaterConverter.hpp"

#include "navteq.hpp"
#include "navteq_plugin.hpp"
#include "navteq_util.hpp"

/*

 Convert navteq data into routable OSM files.

 */

navteq_plugin::navteq_plugin(const boost::filesystem::path &executable_path)
    : base_plugin::base_plugin("Navteq Plugin", executable_path) {
  // setting executable_path in navteq2osm_tag_parser.hpp for reading ISO-file
  g_executable_path = this->executable_path;

  converter.emplace_back(new RailwayConverter());
  converter.emplace_back(new WaterConverter());
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

bool navteq_plugin::check_files(const boost::filesystem::path &dir) {
  if (!shp_file_exists(dir / STREETS_SHP))
    return false;

  if (!dbf_file_exists(dir / MTD_AREA_DBF))
    return false;
  if (!dbf_file_exists(dir / RDMS_DBF))
    return false;
  if (!dbf_file_exists(dir / CDMS_DBF))
    return false;
  if (!dbf_file_exists(dir / ZLEVELS_DBF))
    return false;
  if (!dbf_file_exists(dir / MAJ_HWYS_DBF))
    return false;
  if (!dbf_file_exists(dir / SEC_HWYS_DBF))
    return false;
  if (!dbf_file_exists(dir / ALT_STREETS_DBF))
    return false;
  if (!shp_file_exists(dir / POINT_ADDRESS_SHP))
    BOOST_LOG_TRIVIAL(warning) << "  point addresses are missing";

  if (!shp_file_exists(dir / NAMED_PLC_SHP))
    BOOST_LOG_TRIVIAL(warning) << "  named places are missing";
  if (!shp_file_exists(dir / ADMINBNDY_1_SHP))
    BOOST_LOG_TRIVIAL(warning)
        << "  administrative boundaries level 1 are missing";
  if (!shp_file_exists(dir / ADMINBNDY_2_SHP))
    BOOST_LOG_TRIVIAL(warning)
        << "  administrative boundaries level 2 are missing";

  boost::filesystem::path bbCheckFile;

  if (!shp_file_exists(dir / ADMINBNDY_3_SHP)) {
    BOOST_LOG_TRIVIAL(warning)
        << "  administrative boundaries level 3 are missing";
  } else {
    bbCheckFile = dir / ADMINBNDY_3_SHP;
  }

  if (!shp_file_exists(dir / ADMINBNDY_4_SHP)) {
    BOOST_LOG_TRIVIAL(warning)
        << "  administrative boundaries level 4 are missing";
  } else {
    bbCheckFile = dir / ADMINBNDY_4_SHP;
  }

  if (!shp_file_exists(dir / ADMINBNDY_5_SHP)) {
    BOOST_LOG_TRIVIAL(warning)
        << "  administrative boundaries level 5 are missing";
  } else {
    bbCheckFile = dir / ADMINBNDY_5_SHP;
  }

  if (!shp_file_exists(dir / ADMINLINE_1_SHP))
    BOOST_LOG_TRIVIAL(warning) << "  administrative lines level 1 are missing";

  // check boundingbox
  if (!checkInBoundingBox(boundingBox, bbCheckFile)) {
    BOOST_LOG_TRIVIAL(warning) << dir.string() << " out of boundingbox";
    return false;
  }

  if (!checkCountryCode(dir)) {
    BOOST_LOG_TRIVIAL(info) << dir.string() << " skip country";
    return false;
  }

  return true;
}

/**
 * \brief Checks wether there is a subdirectory containinig valid data.
 * \param dir directory from which to start recursion
 * \param recur if set non-directories within the root directory are ignored
 * \return Existance of valid data in a subdirectory.
 */

void navteq_plugin::recurse_dir(const boost::filesystem::path &dir) {
  if (check_files(dir))
    dataDirs.push_back(dir);

  for (auto &itr : boost::make_iterator_range(
           boost::filesystem::directory_iterator(dir), {})) {
    if (boost::filesystem::is_directory(itr)) {
      recurse_dir(itr);
    }
  }
}

bool navteq_plugin::check_input(const boost::filesystem::path &input_path,
                                const boost::filesystem::path &output_file) {
  if (!boost::filesystem::is_directory(input_path))
    throw(std::runtime_error("directory " + input_path.string() +
                             " does not exist"));

  if (!output_file.empty()) {
    boost::filesystem::path output_path = output_file.parent_path();
    if (!boost::filesystem::is_directory(output_path))
      throw(std::runtime_error("output directory " + output_path.string() +
                               " does not exist"));
    if (!is_valid_format(output_file.string()))
      throw(format_error("unknown format for outputfile: " +
                         output_file.string()));
  }

  recurse_dir(input_path);

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

void navteq_plugin::add_administrative_boundaries(
    const std::vector<boost::filesystem::path> &dirs,
    osmium::io::Writer &writer) {
  // todo admin-levels only apply to the US => more generic for all countries

  g_way_end_points_map.clear();
  addLevel1Boundaries(dirs, writer);

  for (auto dir : dirs) {
    if (shp_file_exists(dir / ADMINBNDY_2_SHP))
      addLevelNBoundaries(dir / ADMINBNDY_2_SHP, writer, 2);
    if (shp_file_exists(dir / ADMINBNDY_3_SHP))
      addLevelNBoundaries(dir / ADMINBNDY_3_SHP, writer, 3);
    if (shp_file_exists(dir / ADMINBNDY_4_SHP))
      addLevelNBoundaries(dir / ADMINBNDY_4_SHP, writer, 4);
    if (shp_file_exists(dir / ADMINBNDY_5_SHP))
      addLevelNBoundaries(dir / ADMINBNDY_5_SHP, writer, 5);
  }

  // build relations for the admin line
  g_mtd_area_map.clear();

  g_way_end_points_map.clear();
}

void navteq_plugin::add_railways(
    const std::vector<boost::filesystem::path> &dirs,
    osmium::io::Writer &writer) {
  for (auto dir : dirs) {
    if (shp_file_exists(dir / RAILWAYS_POLY_SHP))
      add_railways_shape(dir / RAILWAYS_POLY_SHP, writer);
  }
}

void navteq_plugin::add_buildings(
    const std::vector<boost::filesystem::path> &dirs,
    osmium::io::Writer &writer) {
  g_way_end_points_map.clear();
  for (auto dir : dirs) {
    if (shp_file_exists(dir / LANDMARK_SHP))
      add_building_shape(dir / LANDMARK_SHP, writer);
  }
  g_way_end_points_map.clear();
}

void navteq_plugin::add_landuse(
    const std::vector<boost::filesystem::path> &dirs,
    osmium::io::Writer &writer) {
  for (auto dir : dirs) {
    if (shp_file_exists(dir / LAND_USE_A_SHP))
      add_landuse_shape(dir / LAND_USE_A_SHP, writer);
    if (shp_file_exists(dir / LAND_USE_B_SHP))
      add_landuse_shape(dir / LAND_USE_B_SHP, writer);
  }
}

void navteq_plugin::execute() {
  if (output_path.empty())
    throw std::exception();

  osmium::io::File outfile(
      output_path.parent_path().append("tmp.pbf").string());
  osmium::io::Header hdr;
  hdr.set("generator", "osmium");
  hdr.set("xml_josm_upload", "false");
  osmium::io::Writer writer(outfile, hdr, osmium::io::overwrite::allow);

  BOOST_LOG_TRIVIAL(info) << "Procesing Meta areas";
  preprocess_meta_areas(dataDirs);

  BOOST_LOG_TRIVIAL(info) << "Procesing alt street rout types";
  process_alt_steets_route_types(dataDirs);

  BOOST_LOG_TRIVIAL(info) << "Add street shapes";
  add_street_shapes(dataDirs, writer);

  BOOST_LOG_TRIVIAL(info) << "Add administrative boundaries";
  add_administrative_boundaries(dataDirs, writer);

  BOOST_LOG_TRIVIAL(info) << "Add landuse";
  add_landuse(dataDirs, writer);

  BOOST_LOG_TRIVIAL(info) << "Add city nodes";
  add_city_nodes(dataDirs, writer);

  BOOST_LOG_TRIVIAL(info) << "Add hamlet nodes";
  add_hamlet_nodes(dataDirs, writer);

  BOOST_LOG_TRIVIAL(info) << "Add buildings";
  add_buildings(dataDirs, writer);

  BOOST_LOG_TRIVIAL(info) << "Add rest areas";
  add_rest_area_nodes(dataDirs, writer);

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

  boost::filesystem::remove(output_path.parent_path().append("tmp.pbf"));
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

bool navteq_plugin::checkCountryCode(const boost::filesystem::path &dir) {
  DBFHandle handle = read_dbf_file(dir / MTD_CNTRY_REF_DBF);

  if (countriesToConvert.empty())
    return true;

  for (int i = 0; i < DBFGetRecordCount(handle); i++) {
    std::string countryCode = dbf_get_string_by_field(handle, i, ISO_CODE);

    foundCountries.insert(countryCode);
    auto found = std::find(countriesToConvert.cbegin(),
                           countriesToConvert.cend(), countryCode);
    if (found != countriesToConvert.end()) {
      DBFClose(handle);
      return true;
    }
  }
  DBFClose(handle);

  return false;
}

void navteq_plugin::setDebug(bool debug) { debugMode = debug; }