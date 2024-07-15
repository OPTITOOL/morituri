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

#ifndef BUILDINGCONVERTER_HPP
#define BUILDINGCONVERTER_HPP

#include "Converter.hpp"

#include <ogrsf_frmts.h>

class BuildingConverter : public Converter {

public:
  BuildingConverter(const std::filesystem::path &executable_path);
  virtual ~BuildingConverter();

  virtual void convert(const std::vector<std::filesystem::path> &dirs,
                       osmium::io::Writer &writer);

private:
  void add_building_shape(std::filesystem::path landmark_shape_file,
                          osmium::io::Writer &writer);

  void
  process_building(const OGRFeatureUniquePtr &feat,
                   std::map<osmium::Location, osmium::unsigned_object_id_type>
                       &g_way_end_points_map,
                   osmium::memory::Buffer &node_buffer,
                   osmium::memory::Buffer &way_buffer);

  void build_building_poly_taglist(osmium::builder::WayBuilder &builder,
                                   const OGRFeatureUniquePtr &feat);
};

#endif // BUILDINGCONVERTER_HPP
