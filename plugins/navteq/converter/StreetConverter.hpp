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
  StreetConverter(const std::filesystem::path &executable_path);
  virtual ~StreetConverter();

  virtual void convert(const std::filesystem::path &dir,
                       osmium::io::Writer &writer) override;

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

  struct dateTimeMod {
    bool hasDateTimeMod = false;
  };

  struct cond_type {
    uint64_t cond_id_type;
    uint64_t cond_type_type;
    bool only_hgv;
    std::map<uint64_t, StreetConverter::mod_group_type> mod_group_map;
    dateTimeMod dt_mod;
  };

  struct cntry_ref_type {
    std::string unit_measure;
    std::string speed_limit_unit;
    std::string iso_code;
    uint64_t area_code_1 = 0;
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
      if (this->area_code_1 != rhs.area_code_1)
        return false;
      return true;
    }
    bool operator!=(cntry_ref_type rhs) { return !(*this == rhs); }
  };

  struct TagData {
    std::map<uint64_t, ushort> route_type_map;

    std::multimap<uint64_t, StreetConverter::cond_type> cdms_map;
    std::map<uint64_t, uint64_t> area_govt_map;
    std::map<uint64_t, cntry_ref_type> cntry_map;
    std::map<osmium::Location, std::tuple<std::string, std::string>> ramp_names;
    std::map<uint64_t, std::map<uint, std::string>> highway_names;
  };

private:
  std::map<uint64_t, ushort>
  process_alt_steets_route_types(const std::filesystem::path &dir);

  std::map<uint64_t, std::vector<StreetConverter::z_lvl_index_type_t>>
  init_z_level_map(const std::filesystem::path &dir);

  void
  init_cnd_mod(std::multimap<uint64_t, StreetConverter::cond_type> &cdms_map,
               const std::filesystem::path &dir);

  void
  init_cnd_dt_mod(std::multimap<uint64_t, StreetConverter::cond_type> &cdms_map,
                  const std::filesystem::path &dir);

  std::multimap<uint64_t, StreetConverter::cond_type>
  init_cdms_map(const std::filesystem::path &dir);

  void update_region_connecting_points(
      std::map<osmium::Location, osmium::unsigned_object_id_type>
          &regionConnectingPoints,
      const StreetConverter::TagData &data, const std::filesystem::path &dir,
      osmium::io::Writer &writer);

  std::map<uint64_t, uint64_t> init_g_area_to_govt_code_map(
      std::map<uint64_t, StreetConverter::cntry_ref_type> &cntry_ref_map,
      const std::filesystem::path &dir);

  std::map<uint64_t, StreetConverter::cntry_ref_type>
  init_g_cntry_ref_map(const std::filesystem::path &dir);

  std::map<uint64_t, std::map<uint, std::string>>
  init_highway_names(const std::filesystem::path &dir);

  // ramp names
  std::map<osmium::Location, std::tuple<std::string, std::string>>
  init_ramp_names(const std::filesystem::path &dir);
  std::map<uint64_t, std::string>
  read_junction_names(const std::filesystem::path &dbf_file);

  void parse_ramp_names(
      const std::filesystem::path &shp_file,
      std::map<osmium::Location, std::tuple<std::string, std::string>>
          &ramps_ref_map,
      const std::map<uint64_t, std::string> &junctionNames);

  // process end nodes
  std::map<osmium::Location, osmium::unsigned_object_id_type>
  process_way_end_nodes(
      const std::filesystem::path &dir, const StreetConverter::TagData &data,
      const std::map<osmium::Location, osmium::unsigned_object_id_type>
          &regionConnectingPoints,
      std::map<uint64_t, std::vector<StreetConverter::z_lvl_index_type_t>>
          &z_level_map,
      osmium::io::Writer &writer);

  void process_way_end_nodes(
      OGRFeatureUniquePtr &feat, const StreetConverter::TagData &data,
      std::map<osmium::Location, osmium::unsigned_object_id_type>
          &way_end_points_map,
      osmium::memory::Buffer &node_buffer);

  void process_way_end_node(
      const osmium::Location &location, const StreetConverter::TagData &data,
      std::map<osmium::Location, osmium::unsigned_object_id_type>
          &way_end_points_map,
      osmium::memory::Buffer &node_buffer);

  void
  process_way(const std::filesystem::path &dir,
              const StreetConverter::TagData &data,
              std::map<osmium::Location, osmium::unsigned_object_id_type>
                  &way_end_points_map,
              std::map<uint64_t, std::vector<z_lvl_index_type_t>> &z_level_map,
              osmium::io::Writer &writer);
  void
  process_way(OGRFeatureUniquePtr &feat, const StreetConverter::TagData &data,
              std::map<osmium::Location, osmium::unsigned_object_id_type>
                  &way_end_points_map,
              std::map<std::pair<osmium::Location, short>,
                       osmium::unsigned_object_id_type> &z_lvl_nodes_map,
              std::map<uint64_t, std::vector<z_lvl_index_type_t>> &z_level_map,
              osmium::memory::Buffer &node_buffer,
              osmium::memory::Buffer &way_buffer);

  void middle_points_preparation(
      OGRLineString *ogr_ls,
      std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
      osmium::memory::Buffer &node_buffer);

  osmium::unsigned_object_id_type build_way(
      OGRFeatureUniquePtr &feat, OGRLineString *ogr_ls,
      std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
      const StreetConverter::TagData &data,
      std::map<osmium::Location, osmium::unsigned_object_id_type>
          &way_end_points_map,
      osmium::memory::Buffer &way_buffer, bool is_sub_linestring, short z_lvl);

  uint64_t build_tag_list(OGRFeatureUniquePtr &feat, const TagData &data,
                          osmium::builder::Builder &builder, short z_level);

  uint64_t parse_street_tags(
      osmium::builder::TagListBuilder &builder, OGRFeatureUniquePtr &f,
      const std::multimap<uint64_t, cond_type> &cdms_map,
      const std::map<uint64_t, uint64_t> &area_govt_map,
      const std::map<uint64_t, cntry_ref_type> &cntry_map,
      const std::map<uint64_t, ushort> &route_type_map,
      const std::map<uint64_t, std::map<uint, std::string>> &names_map);

  void set_ferry_z_lvls_to_zero(const OGRFeatureUniquePtr &feat,
                                std::vector<z_lvl_index_type_t> &z_lvl_vec);

  void test__z_lvl_range(short z_lvl);

  void parse_highway_names(
      const std::filesystem::path &dbf_file,
      std::map<uint64_t, std::map<uint, std::string>> &hwys_ref_map,
      bool isStreetLayer);

  void add_additional_restrictions(
      osmium::builder::TagListBuilder &builder, uint64_t link_id,
      const std::multimap<uint64_t, cond_type> &cdms_map,
      const cntry_ref_type &cntry_ref);

  void addSpecialSpeedSituation(
      std::multimap<uint64_t, StreetConverter::cond_type>::const_iterator &it,
      uint64_t link_id, const StreetConverter::cntry_ref_type &cntry_ref,
      osmium::builder::TagListBuilder &builder, bool imperial_units);

  void addTransportAccessRestriction(
      std::multimap<uint64_t, StreetConverter::cond_type>::const_iterator &it,
      osmium::builder::TagListBuilder &builder, uint64_t link_id,
      const StreetConverter::cntry_ref_type &cntry_ref, bool imperial_units);

  void process_end_point(
      bool first, short z_lvl, OGRLineString *ogr_ls,
      std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
      std::map<osmium::Location, osmium::unsigned_object_id_type>
          &way_end_points_map,
      std::map<std::pair<osmium::Location, short>,
               osmium::unsigned_object_id_type> &z_lvl_nodes_map,
      osmium::memory::Buffer &node_buffer);

  void split_way_by_z_level(
      OGRFeatureUniquePtr &feat, OGRLineString *ogr_ls,
      const std::vector<z_lvl_index_type_t> &node_z_level_vector,
      std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
      const StreetConverter::TagData &data,
      std::map<osmium::Location, osmium::unsigned_object_id_type>
          &way_end_points_map,
      uint64_t link_id, osmium::memory::Buffer &way_buffer);

  void build_sub_way_by_index(
      OGRFeatureUniquePtr &feat, OGRLineString *ogr_ls, ushort start_index,
      ushort end_index,
      std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
      const StreetConverter::TagData &data,
      std::map<osmium::Location, osmium::unsigned_object_id_type>
          &way_end_points_map,
      osmium::memory::Buffer &way_buffer, short z_lvl);

  ushort create_continuing_sub_ways(
      OGRFeatureUniquePtr &feat, OGRLineString *ogr_ls, ushort first_index,
      ushort start_index, ushort last_index, uint link_id,
      const std::vector<z_lvl_index_type_t> &node_z_level_vector,
      std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
      const StreetConverter::TagData &data,
      std::map<osmium::Location, osmium::unsigned_object_id_type>
          &way_end_points_map,
      osmium::memory::Buffer &way_buffer);

  StreetConverter::cntry_ref_type
  get_cntry_ref(const OGRFeatureUniquePtr &f,
                const std::map<uint64_t, uint64_t> &area_to_areacode1_map,
                const std::map<uint64_t, cntry_ref_type> &cntry_map);

  bool is_imperial(const cntry_ref_type &cntry_ref);

  bool is_motorized_allowed(const OGRFeatureUniquePtr &f);

  bool is_hgv_only(const OGRFeatureUniquePtr &f);

  bool need_to_consider(const OGRFeatureUniquePtr &f);

  void add_highway_tags(osmium::builder::TagListBuilder &builder,
                        const OGRFeatureUniquePtr &f, ushort route_type,
                        const cntry_ref_type &cntry_ref,
                        const std::string &ref_name, bool underConstruction);

  void add_highway_tag(osmium::builder::TagListBuilder &builder,
                       const OGRFeatureUniquePtr &f, ushort route_type,
                       ushort func_class, const cntry_ref_type &cntry_ref,
                       const std::string &ref_name, bool underConstruction);

  std::string_view get_hwy_value(ushort route_type, ushort func_class,
                                 uint area_code_1, const std::string &ref_name,
                                 bool urban);

  void add_hazmat_tag(osmium::builder::TagListBuilder &builder,
                      uint64_t mod_val);

  std::string add_highway_name_tags(
      osmium::builder::TagListBuilder &builder,
      const std::map<uint64_t, std::map<uint, std::string>> &names_map,
      uint64_t link_id, bool ramp);

  void add_ferry_tag(osmium::builder::TagListBuilder &builder,
                     const OGRFeatureUniquePtr &f);

  void add_one_way_tag(osmium::builder::TagListBuilder &builder,
                       const char *value);

  void add_access_tags(osmium::builder::TagListBuilder &builder,
                       const OGRFeatureUniquePtr &f);

  void add_maxspeed_tags(osmium::builder::TagListBuilder &builder,
                         const OGRFeatureUniquePtr &f,
                         const cntry_ref_type &cntry_ref);

  void add_lanes_tag(osmium::builder::TagListBuilder &builder,
                     const OGRFeatureUniquePtr &f);

  void add_postcode_tag(osmium::builder::TagListBuilder &builder,
                        const OGRFeatureUniquePtr &f);

  bool only_pedestrians(const OGRFeatureUniquePtr &f);

  void add_here_speed_cat_tag(osmium::builder::TagListBuilder &builder,
                              const OGRFeatureUniquePtr &f);

  bool is_ferry(const char *value);

  bool is_superior_or_equal(short superior, short than);

  bool is_superior(short superior, short than);

  uint get_number_after(const std::string &str, const char *start_str);

  std::string lbs_to_metric_ton(double lbs);

  std::string inch_to_feet(unsigned int inches);

  template <class T> std::string kg_to_t(T kilo) {
    return std::to_string(kilo / 1000.0f);
  }

  template <class T> std::string cm_to_m(T meter) {
    return std::to_string(meter / 100.0f);
  }

  void addRestrictionTag(osmium::builder::TagListBuilder &builder,
                         const std::string &restriction, int direction,
                         bool is_imperial_units, uint64_t max_value);

  // CndMod types (CM)
  static constexpr std::string_view CM_MOD_TYPE = "MOD_TYPE";
  static constexpr std::string_view CM_MOD_VAL = "MOD_VAL";

  // ZLEVELS_DBF columns
  static constexpr std::string_view Z_LEVEL = "Z_LEVEL";
  static constexpr std::string_view POINT_NUM = "POINT_NUM";
  static constexpr std::string_view ALIGNED = "ALIGNED";

  static constexpr std::string_view GOVT_CODE = "GOVT_CODE";
  static constexpr std::string_view AREACODE_1 = "AREACODE_1";

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

  static constexpr std::string_view ROUTE = "ROUTE_TYPE";
  static constexpr std::string_view FUNC_CLASS = "FUNC_CLASS";
  static constexpr std::string_view SPEED_CAT = "SPEED_CAT";
  static constexpr std::string_view FR_SPEED_LIMIT = "FR_SPD_LIM";
  static constexpr std::string_view TO_SPEED_LIMIT = "TO_SPD_LIM";
  static constexpr std::string_view DIR_TRAVEL = "DIR_TRAVEL";
  static constexpr std::string_view AR_AUTO = "AR_AUTO";
  static constexpr std::string_view AR_BUS = "AR_BUS";
  static constexpr std::string_view AR_TAXIS = "AR_TAXIS";
  // static constexpr std::string_view AR_CARPOOL = "AR_CARPOOL";
  static constexpr std::string_view AR_PEDESTRIANS = "AR_PEDEST";
  static constexpr std::string_view AR_TRUCKS = "AR_TRUCKS";
  static constexpr std::string_view AR_DELIV = "AR_DELIV";
  static constexpr std::string_view AR_EMERVEH = "AR_EMERVEH";
  static constexpr std::string_view AR_MOTORCYCLES = "AR_MOTOR";
  static constexpr std::string_view AR_THROUGH_TRAFFIC = "AR_TRAFF";
  static constexpr std::string_view PAVED = "PAVED";
  static constexpr std::string_view PRIVATE = "PRIVATE";
  static constexpr std::string_view BRIDGE = "BRIDGE";
  static constexpr std::string_view TUNNEL = "TUNNEL";
  static constexpr std::string_view TOLLWAY = "TOLLWAY";
  static constexpr std::string_view CONTRACC = "CONTRACC";
  static constexpr std::string_view ROUNDABOUT = "ROUNDABOUT";
  static constexpr std::string_view FERRY = "FERRY_TYPE";
  static constexpr std::string_view URBAN = "URBAN";

  static constexpr std::string_view FOURWHLDR = "FOURWHLDR";
  static constexpr std::string_view PHYS_LANES = "PHYS_LANES";
  static constexpr std::string_view PUB_ACCESS = "PUB_ACCESS";

  static constexpr std::string_view L_POSTCODE = "L_POSTCODE";
  static constexpr std::string_view R_POSTCODE = "R_POSTCODE";
  static constexpr std::string_view RAMP = "RAMP";

  // highway OSM tags

  static constexpr std::string_view HIGHWAY = "highway";

  static constexpr std::string_view HIGHWAY_NM = "HIGHWAY_NM";
  static constexpr std::string_view ST_NAME = "ST_NAME";
  static constexpr std::string_view COND_TYPE = "COND_TYPE";

  const std::filesystem::path STREETS_SHP = "Streets.shp";

  const std::filesystem::path ALT_STREETS_DBF = "AltStreets.dbf";

  // condition types (CT)
  const ushort CT_CONSTRUCTION_STATUS_CLOSED = 3;
  const ushort CT_TRANSPORT_ACCESS_RESTRICTION = 23;
  const ushort CT_TRANSPORT_SPECIAL_SPEED_SITUATION = 25;

  // should be global for connectivity between regions
  static std::map<osmium::Location, osmium::unsigned_object_id_type>
      g_regionConnectingPoints;

  static std::mutex g_regionConnectingPoints_mutex;
};

#endif // STREETCONVERTER_HPP
