/*
 * This file is part of the Morituri project.
 * Morituri is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Morituri is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Morituri.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef HOUSENUMBERCONVERTER_HPP
#define HOUSENUMBERCONVERTER_HPP

#include "Converter.hpp"

class HouseNumberConverter : public Converter {

public:
  HouseNumberConverter(const std::filesystem::path &executable_path);
  virtual ~HouseNumberConverter();

  virtual void convert(const std::filesystem::path &dir,
                       osmium::io::Writer &writer) override;

private:
  void create_house_numbers(const OGRFeatureUniquePtr &feat,
                            const OGRLineString *ogr_ls, bool left,
                            osmium::memory::Buffer &node_buffer,
                            osmium::memory::Buffer &way_buffer);

  void create_house_numbers(const OGRFeatureUniquePtr &feat,
                            const OGRLineString *ogr_ls,
                            osmium::memory::Buffer &node_buffer,
                            osmium::memory::Buffer &way_buffer);

  void create_premium_house_numbers(
      const OGRFeatureUniquePtr &feat,
      const std::vector<std::pair<osmium::Location, std::string>> &addressList,
      int linkId, osmium::memory::Buffer &node_buffer);

  void process_house_numbers(const std::filesystem::path &dirs,
                             osmium::io::Writer &writer);

  void process_house_numbers(
      const OGRFeatureUniquePtr &feat,
      const std::map<uint64_t,
                     std::vector<std::pair<osmium::Location, std::string>>>
          &pointAddresses,
      int linkId, osmium::memory::Buffer &node_buffer,
      osmium::memory::Buffer &way_buffer);

  std::map<uint64_t, std::vector<std::pair<osmium::Location, std::string>>>
  createPointAddressMapList(const std::filesystem::path &dir);

  const char *parse_house_number_schema(const char *schema);

  const std::filesystem::path STREETS_SHP = "Streets.shp";

  static constexpr std::string_view ST_NAME = "ST_NAME";

  static constexpr std::string_view ADDR_TYPE = "ADDR_TYPE";

  static constexpr std::string_view L_REFADDR = "L_REFADDR";
  const char *L_NREFADDR = "L_NREFADDR";
  const char *L_ADDRSCH = "L_ADDRSCH";
  // const char *L_ADDRFORM = "L_ADDRFORM";
  const char *R_REFADDR = "R_REFADDR";
  const char *R_NREFADDR = "R_NREFADDR";
  const char *R_ADDRSCH = "R_ADDRSCH";

  const double HOUSENUMBER_CURVE_OFFSET = 0.00005;
};

#endif // HOUSENUMBERCONVERTER_HPP