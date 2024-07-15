/*
 * navteq_plugin.hpp
 *
 *  Created on: 11.06.2015
 *      Author: philip
 */

#ifndef NAVTEQPLUGIN_HPP_
#define NAVTEQPLUGIN_HPP_

#include "../base_plugin.hpp"
#include "converter/Converter.hpp"
#include <filesystem>
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
  std::optional<std::filesystem::path>
  check_files(const std::filesystem::path &dir);
  void write_output();

  void sortPBF();
  void copyType(osmium::io::Writer &writer, osmium::io::File &file,
                osmium::osm_entity_bits::type bits);

  std::vector<std::filesystem::path> dataDirs;

  std::vector<std::unique_ptr<Converter>> converter;

  OGREnvelope boundingBox;

  std::vector<std::string> countriesToConvert;
  std::set<std::string> foundCountries;
  bool debug;

public:
  navteq_plugin(const std::filesystem::path &executable_path);
  virtual ~navteq_plugin();

  bool check_input(
      const std::filesystem::path &input_path,
      const std::filesystem::path &output_file = std::filesystem::path());
  void execute();

  void setBoundingBox(double minX, double minY, double maxX, double maxY);

  void setCountries(const std::vector<std::string> &_countries);

  void setDebug(bool debug);
};

#endif /* NAVTEQPLUGIN_HPP_ */
