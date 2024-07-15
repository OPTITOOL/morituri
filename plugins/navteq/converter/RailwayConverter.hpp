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

#ifndef RAILWAYCONVERTER_HPP
#define RAILWAYCONVERTER_HPP

#include "Converter.hpp"

#include <ogrsf_frmts.h>

class RailwayConverter : public Converter {

public:
  RailwayConverter(const boost::filesystem::path &executable_path);
  virtual ~RailwayConverter();

  virtual void convert(const std::vector<boost::filesystem::path> &dirs,
                       osmium::io::Writer &writer);

private:
  void add_railways_shape(boost::filesystem::path railway_shape_file,
                          osmium::io::Writer &writer);

  void
  process_railways(const OGRFeatureUniquePtr &feat,
                   std::map<osmium::Location, osmium::unsigned_object_id_type>
                       &g_way_end_points_map,
                   osmium::memory::Buffer &node_buffer,
                   osmium::memory::Buffer &way_buffer);

  static constexpr std::string_view BRIDGE = "BRIDGE";
  static constexpr std::string_view TUNNEL = "TUNNEL";
};

#endif // RAILWAYCONVERTER_HPP
