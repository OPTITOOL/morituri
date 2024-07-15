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

#ifndef ADMINBOUNDARIESCONVERTER_HPP
#define ADMINBOUNDARIESCONVERTER_HPP

#include "Converter.hpp"

#include <ogrsf_frmts.h>

namespace osmium {
namespace memory {
class Buffer;
} // namespace memory
namespace builder {
class Builder;
} // namespace builder
} // namespace osmium

class AdminBoundariesConverter : public Converter {

public:
  AdminBoundariesConverter(const std::filesystem::path &executable_path);
  virtual ~AdminBoundariesConverter();

  virtual void convert(const std::vector<std::filesystem::path> &dirs,
                       osmium::io::Writer &writer);

private:
  void addLevel1Boundaries(
      const std::vector<std::filesystem::path> &dirs,
      std::map<osmium::Location, osmium::unsigned_object_id_type>
          &g_way_end_points_map,
      osmium::io::Writer &writer);

  void add_admin_shape(
      std::filesystem::path admin_shape_file,
      std::map<osmium::Location, osmium::unsigned_object_id_type>
          &g_way_end_points_map,
      osmium::io::Writer &writer,
      std::map<int, std::pair<std::vector<osmium::unsigned_object_id_type>,
                              std::vector<osmium::unsigned_object_id_type>>>
          &adminLineMap);

  std::map<int, std::vector<osmium::unsigned_object_id_type>>
  add_admin_lines(std::filesystem::path admin_line_shape_file,
                  std::map<osmium::Location, osmium::unsigned_object_id_type>
                      &g_way_end_points_map,
                  osmium::io::Writer &writer);

  void process_admin_boundary(
      const OGRFeatureUniquePtr &feat,
      std::map<osmium::Location, osmium::unsigned_object_id_type>
          &g_way_end_points_map,
      osmium::memory::Buffer &node_buffer, osmium::memory::Buffer &way_buffer,
      osmium::memory::Buffer &rel_buffer,
      std::map<int, std::pair<std::vector<osmium::unsigned_object_id_type>,
                              std::vector<osmium::unsigned_object_id_type>>>
          &adminLineMap);

  std::vector<osmium::unsigned_object_id_type>
  build_admin_line(OGRFeatureUniquePtr &feat,
                   std::map<osmium::Location, osmium::unsigned_object_id_type>
                       &g_way_end_points_map,
                   osmium::memory::Buffer &node_buffer,
                   osmium::memory::Buffer &way_buffer);

  osmium::unsigned_object_id_type build_admin_boundary_relation_with_tags(
      osmium::unsigned_object_id_type area_id,
      const std::vector<osmium::unsigned_object_id_type> &ext_osm_way_ids,
      const std::vector<osmium::unsigned_object_id_type> &int_osm_way_ids,
      osmium::memory::Buffer &rel_buffer, uint level);

  osmium::unsigned_object_id_type build_admin_boundary_relation_with_tags(
      const OGRFeatureUniquePtr &feat,
      const std::vector<osmium::unsigned_object_id_type> &ext_osm_way_ids,
      const std::vector<osmium::unsigned_object_id_type> &int_osm_way_ids,
      osmium::memory::Buffer &rel_buffer);

  void addLevelNBoundaries(
      std::filesystem::path dir,
      std::map<osmium::Location, osmium::unsigned_object_id_type>
          &g_way_end_points_map,
      osmium::io::Writer &writer, uint level);

  void build_admin_boundary_taglist(osmium::builder::Builder &builder,
                                    osmium::unsigned_object_id_type area_id,
                                    uint level);

  void build_admin_boundary_taglist(osmium::builder::Builder &builder,
                                    const OGRFeatureUniquePtr &feat);

  static constexpr std::string_view AREA_ID = "AREA_ID";
};

#endif // ADMINBOUNDARIESCONVERTER_HPP