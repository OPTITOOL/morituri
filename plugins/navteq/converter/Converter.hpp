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

#ifndef CONVERTER_HPP
#define CONVERTER_HPP

#include <boost/filesystem/path.hpp>
#include <osmium/osm/types.hpp>
#include <string_view>
#include <vector>

namespace osmium {
class OSMObject;
class Location;

namespace io {
class Writer;
} // namespace io

namespace builder {
class RelationBuilder;
class NodeBuilder;
} // namespace builder

namespace memory {
class Buffer;
} // namespace memory
} // namespace osmium

class OGRMultiPolygon;
class OGRPolygon;
class OGRLineString;
class OGRLinearRing;

class Converter {

public:
  Converter() {}
  virtual ~Converter() {}

  virtual void convert(const std::vector<boost::filesystem::path> &dirs,
                       osmium::io::Writer &writer) = 0;

  void set_dummy_osm_object_attributes(osmium::OSMObject &obj);

protected:
  void build_relation_members(
      osmium::builder::RelationBuilder &builder,
      const std::vector<osmium::unsigned_object_id_type> &ext_osm_way_ids,
      const std::vector<osmium::unsigned_object_id_type> &int_osm_way_ids);

  void create_multi_polygon(
      OGRMultiPolygon *mp,
      std::vector<osmium::unsigned_object_id_type> &mp_ext_ring_osm_ids,
      std::vector<osmium::unsigned_object_id_type> &mp_int_ring_osm_ids,
      osmium::memory::Buffer &node_buffer, osmium::memory::Buffer &way_buffer);

  void
  create_polygon(const OGRPolygon *poly,
                 std::vector<osmium::unsigned_object_id_type> &exterior_way_ids,
                 std::vector<osmium::unsigned_object_id_type> &interior_way_ids,
                 osmium::memory::Buffer &node_buffer,
                 osmium::memory::Buffer &way_buffer);

  std::vector<std::pair<osmium::Location, osmium::unsigned_object_id_type>>
  create_open_way_nodes(const OGRLineString *line,
                        osmium::memory::Buffer &node_buffer);

  std::vector<osmium::unsigned_object_id_type>
  build_closed_ways(const OGRLinearRing *ring,
                    osmium::memory::Buffer &node_buffer,
                    osmium::memory::Buffer &way_buffer);

  std::vector<std::pair<osmium::Location, osmium::unsigned_object_id_type>>
  create_closed_way_nodes(const OGRLinearRing *ring,
                          osmium::memory::Buffer &node_buffer);

  osmium::unsigned_object_id_type
  build_node(const osmium::Location &location,
             osmium::memory::Buffer &node_buffer);

  osmium::unsigned_object_id_type
  build_node(const osmium::Location &location,
             osmium::builder::NodeBuilder *builder);

  static constexpr int BUFFER_SIZE = 10 * 1000 * 1000;

  static constexpr int OSM_MAX_WAY_NODES = 1000;

  static constexpr std::string_view USER = "import";
  static constexpr std::string_view VERSION = "1";
  static constexpr std::string_view CHANGESET = "1";
  static constexpr std::string_view USERID = "1";
  static constexpr int TIMESTAMP = 1;

  static osmium::unsigned_object_id_type g_osm_id;
};

#endif // CONVERTER_HPP
