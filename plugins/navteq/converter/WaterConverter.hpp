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

#ifndef WATERCONVERTER_HPP
#define WATERCONVERTER_HPP

#include "Converter.hpp"

#include <map>
#include <ogrsf_frmts.h>
#include <osmium/osm/location.hpp>
#include <osmium/osm/types.hpp>

namespace osmium {
namespace memory {
class Buffer;
} // namespace memory
namespace builder {
class WayBuilder;
class RelationBuilder;
} // namespace builder
} // namespace osmium

class OGRLineString;

class WaterConverter : public Converter {

public:
  WaterConverter();
  virtual ~WaterConverter();

  virtual void convert(const std::vector<boost::filesystem::path> &dirs,
                       osmium::io::Writer &writer);

private:
  void add_water_shape(boost::filesystem::path water_shape_file,
                       osmium::io::Writer &writer);

  void process_water(const OGRFeatureUniquePtr &feat,
                     std::map<osmium::Location, osmium::unsigned_object_id_type>
                         &g_way_end_points_map,
                     osmium::memory::Buffer &node_buffer,
                     osmium::memory::Buffer &way_buffer,
                     osmium::memory::Buffer &rel_buffer);

  std::vector<osmium::unsigned_object_id_type> build_water_ways_with_tagList(
      const OGRFeatureUniquePtr &feat, OGRLineString *line,
      std::map<osmium::Location, osmium::unsigned_object_id_type>
          &g_way_end_points_map,
      osmium::memory::Buffer &node_buffer, osmium::memory::Buffer &way_buffer);

  void build_water_way_taglist(osmium::builder::WayBuilder &builder,
                               const OGRFeatureUniquePtr &feat);

  osmium::unsigned_object_id_type build_water_relation_with_tags(
      const OGRFeatureUniquePtr &feat,
      std::vector<osmium::unsigned_object_id_type> ext_osm_way_ids,
      std::vector<osmium::unsigned_object_id_type> int_osm_way_ids,
      osmium::memory::Buffer &rel_buffer);

  void build_water_poly_taglist(osmium::builder::RelationBuilder &builder,
                                const OGRFeatureUniquePtr &feat);
};

#endif // WATERCONVERTER_HPP