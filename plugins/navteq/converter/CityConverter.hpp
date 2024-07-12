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

#ifndef CITYCONVERTER_HPP
#define CITYCONVERTER_HPP

#include "Converter.hpp"

#include <ogrsf_frmts.h>

class CityConverter : public Converter {

public:
  CityConverter();
  virtual ~CityConverter();

  virtual void convert(const std::vector<boost::filesystem::path> &dirs,
                       osmium::io::Writer &writer);

private:
  void add_city_shape(boost::filesystem::path city_shape_file,
                      osmium::io::Writer &writer);

  void process_city(const OGRFeatureUniquePtr &feat, uint fac_type,
                    osmium::memory::Buffer &node_buffer,
                    const std::map<std::string, std::string> &translations);

  std::string get_place_value(uint population, uint capital);

  static constexpr std::string_view POI_LANGCD = "POI_LANGCD";
  static constexpr std::string_view POI_ID = "POI_ID";
  static constexpr std::string_view POPULATION = "POPULATION";
  static constexpr std::string_view CAPITAL = "CAPITAL";
};

#endif // CITYCONVERTER_HPP
