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

#ifndef STREETCONVERTER_HPP
#define STREETCONVERTER_HPP

#include "Converter.hpp"

#include <osmium/builder/osm_object_builder.hpp>

class StreetConverter : public Converter {

public:
  StreetConverter(const boost::filesystem::path &executable_path);
  virtual ~StreetConverter();

  virtual void convert(const std::vector<boost::filesystem::path> &dirs,
                       osmium::io::Writer &writer);

private:
  struct z_lvl_index_type_t {
    ushort index;
    short z_level;
  };

  struct mod_group_type {
    std::string lang_code;
    uint64_t mod_type;
    uint64_t mod_val;
    mod_group_type(uint64_t mod_type, uint64_t mod_val,
                   const std::string &lang_code) {
      this->lang_code = lang_code;
      this->mod_type = mod_type;
      this->mod_val = mod_val;
    }
  };

  struct cntry_ref_type {
    std::string unit_measure;
    std::string speed_limit_unit;
    std::string iso_code;
    cntry_ref_type() {}
    cntry_ref_type(const std::string &unit_measure,
                   const std::string &speed_limit_unit,
                   const std::string &iso_code) {
      this->unit_measure = unit_measure;
      this->speed_limit_unit = speed_limit_unit;
      this->iso_code = iso_code;
    }
    bool operator==(cntry_ref_type rhs) {
      if (this->unit_measure != rhs.unit_measure)
        return false;
      if (this->speed_limit_unit != rhs.speed_limit_unit)
        return false;
      if (this->iso_code != rhs.iso_code)
        return false;
      return true;
    }
    bool operator!=(cntry_ref_type rhs) { return !(*this == rhs); }
  };

  std::map<uint64_t, ushort> process_alt_steets_route_types(
      const std::vector<boost::filesystem::path> &dirs);

  void add_street_shapes(const std::vector<boost::filesystem::path> &dirs,
                         osmium::io::Writer &writer);

  std::map<uint64_t, std::vector<z_lvl_index_type_t>>
  process_z_levels(const std::vector<boost::filesystem::path> &dirs);

  void init_z_level_map(
      boost::filesystem::path dir,
      std::map<uint64_t, std::vector<z_lvl_index_type_t>> &z_level_map);

  void init_conditional_modifications(const boost::filesystem::path &dir);
  void init_g_cnd_mod_map(const boost::filesystem::path &dir);
  void init_g_cdms_map(const boost::filesystem::path &dir);

  void init_country_reference(const boost::filesystem::path &dir);
  void init_g_area_to_govt_code_map(const boost::filesystem::path &dir);
  void init_g_cntry_ref_map(const boost::filesystem::path &dir);

  // process end nodes
  void process_way_end_nodes(
      const std::vector<boost::filesystem::path> &dirs,
      const std::map<uint64_t, std::vector<z_lvl_index_type_t>> &z_level_map,
      osmium::io::Writer &writer);
  void process_way_end_nodes(OGRFeatureUniquePtr &feat,
                             osmium::memory::Buffer &node_buffer);
  void process_way_end_node(const osmium::Location &location,
                            osmium::memory::Buffer &node_buffer);

  // ramp names
  void init_ramp_names(const boost::filesystem::path &dir);
  std::map<uint64_t, std::string>
  read_junction_names(const boost::filesystem::path &dbf_file);
  void parse_ramp_names(const boost::filesystem::path &shp_file,
                        const std::map<uint64_t, std::string> &junctionNames);

  void process_way(
      OGRFeatureUniquePtr &feat,
      const std::map<uint64_t, std::vector<z_lvl_index_type_t>> &z_level_map,
      osmium::memory::Buffer &node_buffer, osmium::memory::Buffer &way_buffer);
  void middle_points_preparation(
      OGRLineString *ogr_ls,
      std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
      osmium::memory::Buffer &node_buffer);

  osmium::unsigned_object_id_type build_way(
      OGRFeatureUniquePtr &feat, OGRLineString *ogr_ls,
      std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
      osmium::memory::Buffer &way_buffer, bool is_sub_linestring, short z_lvl);

  uint64_t build_tag_list(OGRFeatureUniquePtr &feat,
                          osmium::builder::Builder *builder,
                          osmium::memory::Buffer &buf, short z_level);

  uint64_t parse_street_tags(
      osmium::builder::TagListBuilder *builder, OGRFeatureUniquePtr &f,
      cdms_map_type *cdms_map, cnd_mod_map_type *cnd_mod_map,
      area_id_govt_code_map_type *area_govt_map, cntry_ref_map_type *cntry_map,
      mtd_area_map_type *mtd_area_map, link_id_route_type_map *route_type_map,
      link_id_to_names_map *names_map,
      const std::set<link_id_type> &construction_set, bool debugMode);

  void add_additional_restrictions(osmium::builder::TagListBuilder *builder,
                                   uint64_t link_id, uint64_t l_area_id,
                                   uint64_t r_area_id, cdms_map_type *cdms_map,
                                   cnd_mod_map_type *cnd_mod_map,
                                   area_id_govt_code_map_type *area_govt_map,
                                   cntry_ref_map_type *cntry_map,
                                   mtd_area_map_type *mtd_area_map);

  void process_end_point(
      bool first, short z_lvl, OGRLineString *ogr_ls,
      std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
      osmium::memory::Buffer &node_buffer);

  void split_way_by_z_level(
      OGRFeatureUniquePtr &feat, OGRLineString *ogr_ls,
      const std::vector<z_lvl_index_type_t> &node_z_level_vector,
      std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
      uint64_t link_id, osmium::memory::Buffer &way_buffer);

  void build_sub_way_by_index(
      OGRFeatureUniquePtr &feat, OGRLineString *ogr_ls, ushort start_index,
      ushort end_index,
      std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
      osmium::memory::Buffer &way_buffer, short z_lvl);

  ushort create_continuing_sub_ways(
      OGRFeatureUniquePtr &feat, OGRLineString *ogr_ls, ushort first_index,
      ushort start_index, ushort last_index, uint link_id,
      const std::vector<z_lvl_index_type_t> &node_z_level_vector,
      std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
      osmium::memory::Buffer &way_buffer);

  // CndMod types (CM)
  static constexpr std::string_view CM_MOD_TYPE = "MOD_TYPE";
  static constexpr std::string_view CM_MOD_VAL = "MOD_VAL";

  // ZLEVELS_DBF columns
  static constexpr std::string_view Z_LEVEL = "Z_LEVEL";
  static constexpr std::string_view POINT_NUM = "POINT_NUM";

  // MTD_AREA_DBF columns
  static constexpr std::string_view AREA_ID = "AREA_ID";
  static constexpr std::string_view LANG_CODE = "LANG_CODE";
  static constexpr std::string_view AREA_NAME = "AREA_NAME";
  static constexpr std::string_view AREA_CODE_1 = "AREACODE_1";
  static constexpr std::string_view ADMIN_LVL = "ADMIN_LVL";
  static constexpr std::string_view GOVT_CODE = "GOVT_CODE";

  // RDMS_DBF columns
  static constexpr std::string_view COND_ID = "COND_ID";

  static constexpr std::string_view ST_NM_BASE = "ST_NM_BASE";
  static constexpr std::string_view EXITNAME = "EXITNAME";
  static constexpr std::string_view JUNCTIONNM = "JUNCTIONNM";

  // MTD_CNTRY_REF columns
  static constexpr std::string_view UNTMEASURE = "UNTMEASURE";
  // const char* MAX_ADMINLEVEL = "MAX_ADMINLEVEL";
  static constexpr std::string_view SPEEDLIMITUNIT = "SPDLIMUNIT";
  static constexpr std::string_view ISO_CODE = "ISO_CODE";
};

#endif // STREETCONVERTER_HPP
