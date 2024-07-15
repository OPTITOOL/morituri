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

#ifndef LANDUSECONVERTER_HPP
#define LANDUSECONVERTER_HPP

#include "Converter.hpp"

#include <ogrsf_frmts.h>

class LanduseConverter : public Converter {

public:
  LanduseConverter(const std::filesystem::path &executable_path);
  virtual ~LanduseConverter();

  virtual void convert(const std::vector<std::filesystem::path> &dirs,
                       osmium::io::Writer &writer);

private:
  void add_landuse_shape(std::filesystem::path landuse_shape_file,
                         osmium::io::Writer &writer);

  void
  process_landuse(const OGRFeatureUniquePtr &feat,
                  std::map<osmium::Location, osmium::unsigned_object_id_type>
                      &g_way_end_points_map,
                  osmium::memory::Buffer &node_buffer,
                  osmium::memory::Buffer &way_buffer,
                  osmium::memory::Buffer &rel_buffer);

  osmium::unsigned_object_id_type build_landuse_relation_with_tags(
      const OGRFeatureUniquePtr &feat,
      std::vector<osmium::unsigned_object_id_type> &ext_osm_way_ids,
      std::vector<osmium::unsigned_object_id_type> &int_osm_way_ids,
      osmium::memory::Buffer &rel_buffer);

  void build_landuse_taglist(osmium::builder::RelationBuilder &builder,
                             const OGRFeatureUniquePtr &feat);
};

#endif // LANDUSECONVERTER_HPP