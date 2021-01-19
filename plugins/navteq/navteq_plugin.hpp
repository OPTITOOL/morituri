/*
 * navteq_plugin.hpp
 *
 *  Created on: 11.06.2015
 *      Author: philip
 */

#ifndef NAVTEQPLUGIN_HPP_
#define NAVTEQPLUGIN_HPP_

#include "../base_plugin.hpp"
#include "navteq_types.hpp"
#include <boost/filesystem/path.hpp>
#include <osmium/osm/entity_bits.hpp>
#include <string>
namespace osmium {
namespace io {
class Writer;
class File;
} // namespace io
} // namespace osmium

class navteq_plugin : public base_plugin {
private:
  bool is_valid_format(std::string format);
  void recurse_dir(const boost::filesystem::path &dir);
  bool check_files(const boost::filesystem::path &dir);
  void write_output();
  void add_administrative_boundaries(osmium::io::Writer &writer);
  void add_water(osmium::io::Writer &writer);
  void add_landuse(osmium::io::Writer &writer);
  void add_railways(osmium::io::Writer &writer);
  void add_buildings(osmium::io::Writer &writer);

  void sortPBF();
  void copyType(osmium::io::Writer &writer, osmium::io::File &file,
                osmium::osm_entity_bits::type bits);

  std::vector<boost::filesystem::path> dirs;

  OGREnvelope boundingBox;

  std::vector<std::string> countriesToConvert;
  std::set<std::string> foundCountries;

  bool checkCountryCode(const boost::filesystem::path &dir);

public:
  navteq_plugin(const boost::filesystem::path &executable_path);
  virtual ~navteq_plugin();

  bool check_input(
      const boost::filesystem::path &input_path,
      const boost::filesystem::path &output_path = boost::filesystem::path());
  void execute();

  void setWithTurnRestrictions(bool withTurnRestrictions);

  void setBoundingBox(double minX, double minY, double maxX, double maxY);

  void setCountries(const std::vector<std::string> &_countries);
};

#endif /* NAVTEQPLUGIN_HPP_ */
