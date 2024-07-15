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

  struct cond_type {
    uint64_t cond_id_type;
    uint64_t cond_type_type;
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
  std::map<osmium::Location, std::map<uint, std::string>>
  init_ramp_names(const boost::filesystem::path &dir);
  std::map<uint64_t, std::string>
  read_junction_names(const boost::filesystem::path &dbf_file);
  void parse_ramp_names(
      const boost::filesystem::path &shp_file,
      std::map<osmium::Location, std::map<uint, std::string>> &ramps_ref_map,
      const std::map<uint64_t, std::string> &junctionNames);

  void process_way(
      const std::vector<boost::filesystem::path> &dirs,
      const std::map<uint64_t, std::vector<z_lvl_index_type_t>> &z_level_map,
      osmium::io::Writer &writer);
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
      osmium::builder::TagListBuilder &builder, OGRFeatureUniquePtr &f,
      const std::multimap<uint64_t, cond_type> &cdms_map,
      const std::unordered_map<uint64_t, std::vector<mod_group_type>>
          &cnd_mod_map,
      const std::map<uint64_t, uint64_t> &area_govt_map,
      const std::map<uint64_t, cntry_ref_type> &cntry_map,
      const std::map<osmium::unsigned_object_id_type,
                     Converter::mtd_area_dataset> &mtd_area_map,
      const std::map<uint64_t, ushort> &route_type_map,
      const std::map<uint64_t, std::map<uint, std::string>> &names_map,
      const std::set<uint64_t> &construction_set, bool debugMode);

  std::map<uint64_t, std::map<uint, std::string>>
  init_highway_names(const boost::filesystem::path &dir);

  void parse_highway_names(
      const boost::filesystem::path &dbf_file,
      std::map<uint64_t, std::map<uint, std::string>> &hwys_ref_map,
      bool isStreetLayer);
  void add_additional_restrictions(
      osmium::builder::TagListBuilder &builder, uint64_t link_id,
      uint64_t l_area_id, uint64_t r_area_id,
      const std::multimap<uint64_t, cond_type> &cdms_map,
      const std::unordered_map<uint64_t, std::vector<mod_group_type>>
          &cnd_mod_map,
      const std::map<uint64_t, uint64_t> &area_govt_map,
      const std::map<uint64_t, cntry_ref_type> &cntry_map,
      const std::map<osmium::unsigned_object_id_type,
                     Converter::mtd_area_dataset> &mtd_area_map);

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

  bool is_imperial(uint64_t l_area_id, uint64_t r_area_id,
                   const std::map<uint64_t, uint64_t> &area_govt_map,
                   const std::map<uint64_t, cntry_ref_type> &cntry_map);

  bool is_imperial(uint64_t area_id,
                   const std::map<uint64_t, uint64_t> &area_govt_map,
                   const std::map<uint64_t, cntry_ref_type> &cntry_map);

  bool is_motorized_allowed(const OGRFeatureUniquePtr &f);

  void
  add_highway_tags(osmium::builder::TagListBuilder &builder,
                   const OGRFeatureUniquePtr &f, ushort route_type,
                   const std::map<osmium::unsigned_object_id_type,
                                  Converter::mtd_area_dataset> &mtd_area_map,
                   const std::string &ref_name, bool underConstruction);

  void
  add_highway_tag(osmium::builder::TagListBuilder &builder,
                  const OGRFeatureUniquePtr &f, ushort route_type,
                  ushort func_class,
                  const std::map<osmium::unsigned_object_id_type,
                                 Converter::mtd_area_dataset> &mtd_area_map,
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
                         const OGRFeatureUniquePtr &f);

  void add_lanes_tag(osmium::builder::TagListBuilder &builder,
                     const OGRFeatureUniquePtr &f);

  void add_postcode_tag(osmium::builder::TagListBuilder &builder,
                        const OGRFeatureUniquePtr &f);

  bool only_pedestrians(const OGRFeatureUniquePtr &f);

  void init_under_construction(const boost::filesystem::path &dir);

  void add_here_speed_cat_tag(osmium::builder::TagListBuilder &builder,
                              const OGRFeatureUniquePtr &f);

  void create_house_numbers(const OGRFeatureUniquePtr &feat,
                            const OGRLineString *ogr_ls, bool left,
                            osmium::memory::Buffer &node_buffer,
                            osmium::memory::Buffer &way_buffer);

  void create_house_numbers(const OGRFeatureUniquePtr &feat,
                            const OGRLineString *ogr_ls,
                            osmium::memory::Buffer &node_buffer,
                            osmium::memory::Buffer &way_buffer);

  void create_premium_house_numbers(
      const OGRFeatureUniquePtr &feat,
      const std::vector<std::pair<osmium::Location, std::string>> &addressList,
      int linkId, osmium::memory::Buffer &node_buffer);

  void process_house_numbers(const std::vector<boost::filesystem::path> &dirs,
                             osmium::io::Writer &writer);

  void process_house_numbers(
      const OGRFeatureUniquePtr &feat,
      const std::map<uint64_t,
                     std::vector<std::pair<osmium::Location, std::string>>>
          &pointAddresses,
      int linkId, osmium::memory::Buffer &node_buffer,
      osmium::memory::Buffer &way_buffer);

  std::map<uint64_t, std::vector<std::pair<osmium::Location, std::string>>> *
  createPointAddressMapList(const boost::filesystem::path &dir);

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

  static constexpr std::string_view L_AREA_ID = "L_AREA_ID";
  static constexpr std::string_view R_AREA_ID = "R_AREA_ID";

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
  static constexpr std::string_view MOTORWAY = "motorway";
  static constexpr std::string_view MOTORWAY_LINK = "motorway_link";
  static constexpr std::string_view TRUNK = "trunk";
  static constexpr std::string_view PRIMARY = "primary";
  static constexpr std::string_view PRIMARY_LINK = "primary_link";
  static constexpr std::string_view SECONDARY = "secondary";
  static constexpr std::string_view SECONDARY_LINK = "secondary_link";
  static constexpr std::string_view TERTIARY = "tertiary";
  static constexpr std::string_view UNCLASSIFIED = "unclassified";
  static constexpr std::string_view RESIDENTIAL = "residential";
  static constexpr std::string_view TRACK = "track";
  static constexpr std::string_view PATH = "path";
  static constexpr std::string_view FOOTWAY = "footway";
  static constexpr std::string_view HIGHWAY = "highway";
  static constexpr std::string_view CONSTRUCTION = "construction";

  static constexpr std::string_view HIGHWAY_NM = "HIGHWAY_NM";
  static constexpr std::string_view ST_NAME = "ST_NAME";
  static constexpr std::string_view COND_TYPE = "COND_TYPE";

  static constexpr std::string_view ADDR_TYPE = "ADDR_TYPE";
  static constexpr std::string_view L_REFADDR = "L_REFADDR";
  const char *L_NREFADDR = "L_NREFADDR";
  const char *L_ADDRSCH = "L_ADDRSCH";
  // const char *L_ADDRFORM = "L_ADDRFORM";
  const char *R_REFADDR = "R_REFADDR";
  const char *R_NREFADDR = "R_NREFADDR";
  const char *R_ADDRSCH = "R_ADDRSCH";

  // higway classification
  const std::vector<std::string_view> DEFAULT_HWY_FUNC_TYPE = {
      "", PRIMARY, SECONDARY, SECONDARY, TERTIARY, UNCLASSIFIED, RESIDENTIAL};

  std::map<int, std::vector<std::string_view>> const HWY_FUNC_CLASS_MAP = {
      /* Applied with functional classes:
       *                         1,       2,         3,        4,        5 +
       * rural,    5 + urban */
      {0 /*"DEFAULT"*/, DEFAULT_HWY_FUNC_TYPE},
      {3 /*"GER"*/,
       {"", PRIMARY, SECONDARY, TERTIARY, TERTIARY, UNCLASSIFIED, RESIDENTIAL}},
      {8 /*"CHE"*/,
       {"", PRIMARY, SECONDARY, TERTIARY, TERTIARY, UNCLASSIFIED, RESIDENTIAL}},
      {108 /*"DEN"*/,
       {"", PRIMARY, SECONDARY, SECONDARY, TERTIARY, UNCLASSIFIED,
        RESIDENTIAL}},
      {107 /*"SWE"*/,
       {"", PRIMARY, SECONDARY, SECONDARY, TERTIARY, UNCLASSIFIED,
        RESIDENTIAL}},
      {9 /*"AUT"*/,
       {"", PRIMARY, PRIMARY, SECONDARY, TERTIARY, UNCLASSIFIED, RESIDENTIAL}}};

  static constexpr std::array<std::string_view, 9> speed_cat_metric{
      "",      ">130",  "101-130", "91-100", "71-90",
      "51-70", "31-50", "11-30",   "<11"};

  constexpr double HOUSENUMBER_CURVE_OFFSET = 0.00005;
};

#endif // STREETCONVERTER_HPP