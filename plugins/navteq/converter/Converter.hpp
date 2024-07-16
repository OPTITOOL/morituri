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

#include <filesystem>
#include <map>
#include <ogrsf_frmts.h>
#include <optional>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/osm/types.hpp>
#include <string_view>
#include <vector>

#include "../../comm2osm_exceptions.hpp"

namespace osmium {
class OSMObject;
class Location;

namespace io {
class Writer;
} // namespace io

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
  Converter(const std::filesystem::path &_executable_path)
      : executable_path(_executable_path) {}
  virtual ~Converter() {}

  virtual void convert(const std::vector<std::filesystem::path> &dirs,
                       osmium::io::Writer &writer) = 0;

  void set_dummy_osm_object_attributes(osmium::OSMObject &obj);

  struct mtd_area_dataset {
    osmium::unsigned_object_id_type area_id;
    std::string admin_lvl;
    uint area_code_1;
    std::string name;
    std::string short_name;
    std::vector<std::pair<std::string, std::string>> lang_code_2_area_name;
  };

protected:
  // data structure to store admin boundary tags

  void build_relation_members(
      osmium::builder::RelationBuilder &builder,
      const std::vector<osmium::unsigned_object_id_type> &ext_osm_way_ids,
      const std::vector<osmium::unsigned_object_id_type> &int_osm_way_ids);

  void create_multi_polygon(
      OGRMultiPolygon *mp,
      std::vector<osmium::unsigned_object_id_type> &mp_ext_ring_osm_ids,
      std::vector<osmium::unsigned_object_id_type> &mp_int_ring_osm_ids,
      std::map<osmium::Location, osmium::unsigned_object_id_type>
          &g_way_end_points_map,
      osmium::memory::Buffer &node_buffer, osmium::memory::Buffer &way_buffer);

  void
  create_polygon(const OGRPolygon *poly,
                 std::vector<osmium::unsigned_object_id_type> &exterior_way_ids,
                 std::vector<osmium::unsigned_object_id_type> &interior_way_ids,
                 std::map<osmium::Location, osmium::unsigned_object_id_type>
                     &g_way_end_points_map,
                 osmium::memory::Buffer &node_buffer,
                 osmium::memory::Buffer &way_buffer);

  std::vector<std::pair<osmium::Location, osmium::unsigned_object_id_type>>
  create_open_way_nodes(
      const OGRLineString *line,
      std::map<osmium::Location, osmium::unsigned_object_id_type>
          &g_way_end_points_map,
      osmium::memory::Buffer &node_buffer);

  std::vector<osmium::unsigned_object_id_type>
  build_closed_ways(const OGRLinearRing *ring,
                    std::map<osmium::Location, osmium::unsigned_object_id_type>
                        &g_way_end_points_map,
                    osmium::memory::Buffer &node_buffer,
                    osmium::memory::Buffer &way_buffer);

  std::vector<std::pair<osmium::Location, osmium::unsigned_object_id_type>>
  create_closed_way_nodes(
      const OGRLinearRing *ring,
      std::map<osmium::Location, osmium::unsigned_object_id_type>
          &g_way_end_points_map,
      osmium::memory::Buffer &node_buffer);

  osmium::unsigned_object_id_type
  build_node(const osmium::Location &location,
             osmium::memory::Buffer &node_buffer);

  osmium::unsigned_object_id_type
  build_node(const osmium::Location &location,
             osmium::builder::NodeBuilder &builder);

  template <typename T> void setObjectProperties(T &builder) {
    builder.object().set_id(g_osm_id++);
    set_dummy_osm_object_attributes(builder.object());
    builder.set_user(USER.data());
  }

  std::string navteq_2_osm_admin_lvl(uint navteq_admin_lvl_int);

  std::string navteq_2_osm_admin_lvl(std::string navteq_admin_lvl);

  std::map<osmium::unsigned_object_id_type, mtd_area_dataset>
  process_meta_areas(std::filesystem::path dir);

  uint get_area_code_l(uint64_t l_area_id, uint64_t r_area_id,
                       const std::map<osmium::unsigned_object_id_type,
                                      mtd_area_dataset> &mtd_area_map);

  uint get_area_code_l(const OGRFeatureUniquePtr &f,
                       const std::map<osmium::unsigned_object_id_type,
                                      mtd_area_dataset> &mtd_area_map);

  void parse_lang_code_file();

  std::string parse_lang_code(std::string lang_code);

  GDALDatasetUniquePtr openDataSource(const std::filesystem::path &shape_file);

  std::string to_camel_case_with_spaces(const char *camel);

  std::string to_camel_case_with_spaces(const std::string &camel);

  void add_uint_tag(osmium::builder::TagListBuilder &tl_builder,
                    const char *tag_key, uint uint_tag_val);

  bool parse_bool(const char *value);

  const char *get_field_from_feature(const OGRFeatureUniquePtr &feat,
                                     const std::string_view &field);

  uint64_t get_uint_from_feature(const OGRFeatureUniquePtr &feat,
                                 const std::string_view &field);

  bool string_is_unsigned_integer(const std::string &s);
  bool string_is_not_unsigned_integer(const std::string &s);

  static constexpr int BUFFER_SIZE = 10 * 1000 * 1000;

  static constexpr int OSM_MAX_WAY_NODES = 1000;

  static constexpr std::string_view USER = "import";
  static constexpr std::string_view VERSION = "1";
  static constexpr std::string_view CHANGESET = "1";
  static constexpr std::string_view USERID = "1";
  static constexpr int TIMESTAMP = 1;

  static constexpr std::string_view YES = "yes";
  static constexpr std::string_view NO = "no";

  static osmium::unsigned_object_id_type g_osm_id;

  static constexpr std::string_view FEAT_COD = "FEAT_COD";
  static constexpr std::string_view POLYGON_NM = "POLYGON_NM";

  static constexpr std::string_view FAC_TYPE = "FAC_TYPE";
  static constexpr std::string_view POI_NMTYPE = "POI_NMTYPE";
  static constexpr std::string_view POI_NAME = "POI_NAME";

  static constexpr std::string_view LINK_ID = "LINK_ID";

  // MTD_AREA_DBF columns
  static constexpr std::string_view LANG_CODE = "LANG_CODE";
  static constexpr std::string_view AREA_NAME = "AREA_NAME";
  static constexpr std::string_view AREA_CODE_1 = "AREACODE_1";
  static constexpr std::string_view L_AREA_ID = "L_AREA_ID";
  static constexpr std::string_view R_AREA_ID = "R_AREA_ID";
  static constexpr std::string_view ADMIN_LVL = "ADMIN_LVL";
  static constexpr std::string_view AREA_ID = "AREA_ID";

  static constexpr int NAVTEQ_ADMIN_LVL_MIN = 1;
  static constexpr int NAVTEQ_ADMIN_LVL_MAX = 7;

private:
  static std::map<std::string, std::string> lang_code_map;
  std::filesystem::path executable_path;
};

#endif // CONVERTER_HPP
