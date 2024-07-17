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
#include <map>
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

  std::set<std::string> countriesToConvert;
  std::set<std::string> foundCountries;
  bool debug;

  const std::map<std::string, std::string> HERE_REGION_ISO_MAPPING = {
      {"A1", "AUT"},                // Austria
      {"A9", "GRC"}, {"GK", "GRC"}, // Greece
      {"T1", "TUR"}, {"T3", "TUR"}, {"T4", "TUR"}, {"T5", "TUR"},
      {"T6", "TUR"}, {"T7", "TUR"}, // Turkey
      {"AB", "AZE"},                // Azerbaijan
      {"BB", "BEL"},                // Belgium
      {"C1", "CZE"}, {"C4", "CZE"}, // Czechia
      {"CA", "CHE"},                // Switzerland
      {"E7", "IRL"},                // Ireland
      {"E1", "GBR"}, {"E2", "GBR"}, {"E3", "GBR"}, {"E4", "GBR"},
      {"E5", "GBR"}, {"E6", "GBR"}, // Great britain
      {"F1", "FRA"}, {"F2", "FRA"}, {"F3", "FRA"}, {"F4", "FRA"},
      {"F5", "FRA"}, {"F6", "FRA"},                               // France
      {"SP", "FIN"}, {"SQ", "FIN"}, {"SU", "FIN"}, {"SV", "FIN"}, // Finland
      {"G1", "DEU"}, {"G2", "DEU"}, {"G3", "DEU"}, {"G4", "DEU"},
      {"G5", "DEU"}, {"G6", "DEU"}, {"G7", "DEU"}, {"G8", "DEU"}, // Germany
      {"H1", "HUN"},                                              // Hungary
      {"H2", "HRV"},                                              // Croatia
      {"H3", "SVN"},                                              // Slovenia
      {"H4", "EST"},                                              // Estonia
      {"H5", "LVA"},                                              // Latvia
      {"H6", "LTU"},                                              // Lithuania
      {"H7", "BGR"},                                              // Bulgaria
      {"H8", "ROU"},                                              // Rumania
      {"H9", "MDA"},                                              // Moldova
      {"I1", "ITA"}, {"I2", "ITA"}, {"I3", "ITA"}, {"I4", "ITA"},
      {"I5", "ITA"},                               // Italy
      {"K4", "PRT"}, {"K6", "PRT"}, {"K8", "PRT"}, // Portugal
      {"K1", "ESP"}, {"K2", "ESP"}, {"K3", "ESP"}, {"K5", "ESP"},
      {"K7", "ESP"}, {"K9", "ESP"},                // Spain
      {"KT", "SRB"},                               // Kosovo
      {"C5", "CYP"}, {"KQ", "CYP"},                // cyprus
      {"LU", "LUX"},                               // Luxembourg
      {"MK", "MNE"},                               // Montenegro
      {"M2", "SVK"},                               // Slovakia
      {"M3", "ALB"},                               // Albania
      {"M4", "MKD"},                               // Macedonia
      {"M5", "SRB"},                               // Serbia
      {"M6", "BIH"},                               // Bosnia and Herzegovina
      {"M7", "BLR"},                               // Belarus
      {"M9", "MLT"},                               // Malta
      {"N1", "NOR"},                               // Norway
      {"N2", "ISL"},                               // Iceland
      {"N7", "NLD"}, {"NL", "NLD"},                // Netherland
      {"P1", "POL"}, {"P2", "POL"}, {"P3", "POL"}, // Poland
      {"S1", "SWE"}, {"S2", "SWE"}, {"S3", "SWE"}, // Sweden
      {"S4", "DNK"},                               // Denmark
      {"U1", "UKR"}, {"U2", "UKR"}, {"U3", "UKR"}, {"U4", "UKR"},
      {"U6", "UKR"}, {"UR", "UKR"}, // Ukraine
      {"4A", "RUS"}, {"4B", "RUS"}, {"4C", "RUS"}, {"4D", "RUS"},
      {"4E", "RUS"}, {"4F", "RUS"}, {"4G", "RUS"}, {"4H", "RUS"},
      {"4J", "RUS"}, {"R1", "RUS"}, {"R2", "RUS"}, {"R3", "RUS"},
      {"R4", "RUS"}, {"R5", "RUS"}, {"R6", "RUS"}, {"R7", "RUS"},
      {"R8", "RUS"}, {"R9", "RUS"}, {"RF", "RUS"}, {"RG", "RUS"},
      {"RH", "RUS"}, {"RK", "RUS"}, {"RZ", "RUS"} // Russia
  };

public:
  navteq_plugin(const std::filesystem::path &executable_path);
  virtual ~navteq_plugin();

  bool check_input(
      const std::filesystem::path &input_path,
      const std::filesystem::path &output_file = std::filesystem::path());
  void execute();

  void setBoundingBox(double minX, double minY, double maxX, double maxY);

  void setCountries(const std::set<std::string> &_countries);

  void setDebug(bool debug);
};

#endif /* NAVTEQPLUGIN_HPP_ */
