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

#include "StreetConverter.hpp"

#include <boost/log/trivial.hpp>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/io/writer.hpp>
#include <osmium/memory/buffer.hpp>
#include <osmium/osm/location.hpp>
#include <osmium/osm/types.hpp>

#include "../../comm2osm_exceptions.hpp"
#include "../../util.hpp"

StreetConverter::StreetConverter(const boost::filesystem::path &executable_path)
    : Converter(executable_path) {}

StreetConverter::~StreetConverter() {}

void StreetConverter::convert(const std::vector<boost::filesystem::path> &dirs,
                              osmium::io::Writer &writer) {

  auto route_type_map = process_alt_steets_route_types(dirs);

  add_street_shapes(dirs, writer);
}

std::map<uint64_t, ushort> StreetConverter::process_alt_steets_route_types(
    const std::vector<boost::filesystem::path> &dirs) {

  std::map<uint64_t, ushort> route_type_map;

  const boost::filesystem::path ALT_STREETS_DBF = "AltStreets.dbf";
  for (auto dir : dirs) {
    DBFHandle alt_streets_handle = read_dbf_file(dir / ALT_STREETS_DBF);
    for (int i = 0; i < DBFGetRecordCount(alt_streets_handle); i++) {

      if (dbf_get_string_by_field(alt_streets_handle, i, ROUTE).empty())
        continue;

      osmium::unsigned_object_id_type link_id =
          dbf_get_uint_by_field(alt_streets_handle, i, LINK_ID.data());
      ushort route_type = dbf_get_uint_by_field(alt_streets_handle, i, ROUTE);

      // try to emplace <link_id, route_type> pair
      auto [insertion, inserted] = route_type_map.emplace(link_id, route_type);

      // if its already exists update routetype
      if (!inserted && insertion->second > route_type) {
        // As link id's aren't unique in AltStreets.dbf
        // just store the lowest route type
        insertion->second = route_type;
      }
    }
    DBFClose(alt_streets_handle);
  }

  return route_type_map;
}

void StreetConverter::add_street_shapes(
    const std::vector<boost::filesystem::path> &dirs,
    osmium::io::Writer &writer) {

  BOOST_LOG_TRIVIAL(info) << " processing z-levels";
  auto z_level_map = process_z_levels(dirs);

  BOOST_LOG_TRIVIAL(info) << " processing way end points";
  process_way_end_nodes(dirs, z_level_map, writer);

  BOOST_LOG_TRIVIAL(info) << " processing ways";
  process_way(dirs, z_level_map, writer);

  // create house numbers
  BOOST_LOG_TRIVIAL(info) << " processing house numbers";
  process_house_numbers(dirs, writer);
}

std::map<uint64_t, std::vector<z_lvl_index_type_t>>
StreetConverter::process_z_levels(
    const std::vector<boost::filesystem::path> &dirs) {
  std::map<uint64_t, std::vector<z_lvl_index_type_t>> z_level_map;

  for (auto &dir : dirs) {
    auto path = dir / STREETS_SHP;
    auto ds = open_shape_file(path);

    auto layer = ds->GetLayer(0);
    if (layer == nullptr)
      throw(shp_empty_error(path.string()));
    assert(layer->GetGeomType() == wkbLineString);

    init_z_level_map(dir, z_level_map);
    init_conditional_modifications(dir);
    init_country_reference(dir);
  }
  return z_level_map;
}

void StreetConverter::init_z_level_map(
    boost::filesystem::path dir,
    std::map<uint64_t, std::vector<z_lvl_index_type_t>> &z_level_map) {

  const boost::filesystem::path ZLEVELS_DBF = "Zlevels.dbf";
  // open dbf
  DBFHandle handle = read_dbf_file(dir / ZLEVELS_DBF);

  uint64_t last_link_id = 0;
  std::vector<z_lvl_index_type_t> v;

  for (int i = 0; i < DBFGetRecordCount(handle); i++) {
    uint64_t link_id = dbf_get_uint_by_field(handle, i, LINK_ID.data());
    ushort point_num = dbf_get_uint_by_field(handle, i, POINT_NUM.data()) - 1;
    short z_level = dbf_get_uint_by_field(handle, i, Z_LEVEL.data());

    if (i > 0 && last_link_id != link_id && !v.empty()) {
      z_level_map.emplace(last_link_id, v);
      v = std::vector<z_lvl_index_type_t>();
    }
    if (z_level != 0)
      v.emplace_back(point_num, z_level);
    last_link_id = link_id;
  }

  // close dbf
  DBFClose(handle);

  if (!v.empty())
    z_level_map.emplace(last_link_id, v);
}

void StreetConverter::init_conditional_modifications(
    const boost::filesystem::path &dir) {
  init_g_cnd_mod_map(dir);
  init_g_cdms_map(dir);
}

void StreetConverter::init_g_cnd_mod_map(const boost::filesystem::path &dir) {
  const boost::filesystem::path CND_MOD_DBF = "CndMod.dbf";
  DBFHandle cnd_mod_handle = read_dbf_file(dir / CND_MOD_DBF);
  for (int i = 0; i < DBFGetRecordCount(cnd_mod_handle); i++) {
    uint64_t cond_id = dbf_get_uint_by_field(cnd_mod_handle, i, COND_ID);
    std::string lang_code =
        dbf_get_string_by_field(cnd_mod_handle, i, LANG_CODE.data());
    uint64_t mod_type =
        dbf_get_uint_by_field(cnd_mod_handle, i, CM_MOD_TYPE.data());
    uint64_t mod_val =
        dbf_get_uint_by_field(cnd_mod_handle, i, CM_MOD_VAL.data());

    auto it2 = g_cnd_mod_map.find(cond_id);
    if (it2 == g_cnd_mod_map.end()) {
      std::vector<mod_group_type> new_vector;
      new_vector.push_back(mod_group_type(mod_type, mod_val, lang_code));
      g_cnd_mod_map.emplace(cond_id, new_vector);
    } else {
      //(std::vector<mod_group_type>) ()’
      auto vector = it2->second;
      g_cnd_mod_map.erase(it2);
      vector.push_back(mod_group_type(mod_type, mod_val, lang_code));
      g_cnd_mod_map.emplace(cond_id, vector);
    }
  }
  DBFClose(cnd_mod_handle);
}

void StreetConverter::init_g_cdms_map(const boost::filesystem::path &dir) {
  const boost::filesystem::path CDMS_DBF = "Cdms.dbf";
  DBFHandle cdms_handle = read_dbf_file(dir / CDMS_DBF);
  for (int i = 0; i < DBFGetRecordCount(cdms_handle); i++) {
    uint64_t link_id = dbf_get_uint_by_field(cdms_handle, i, LINK_ID.data());
    uint64_t cond_id = dbf_get_uint_by_field(cdms_handle, i, COND_ID.data());
    ushort cond_type = dbf_get_uint_by_field(cdms_handle, i, COND_TYPE);
    g_cdms_map.emplace(link_id, cond_pair_type(cond_id, cond_type));
  }
  DBFClose(cdms_handle);
}

void StreetConverter::init_country_reference(
    const boost::filesystem::path &dir) {
  if (dbf_file_exists(dir / MTD_AREA_DBF) &&
      dbf_file_exists(dir / MTD_CNTRY_REF_DBF)) {
    init_g_area_to_govt_code_map(dir);
    init_g_cntry_ref_map(dir);
  }
}

void StreetConverter::init_g_area_to_govt_code_map(
    const boost::filesystem::path &dir) {
  DBFHandle mtd_area_handle = read_dbf_file(dir / MTD_AREA_DBF);
  for (int i = 0; i < DBFGetRecordCount(mtd_area_handle); i++) {
    uint64_t area_id =
        dbf_get_uint_by_field(mtd_area_handle, i, AREA_ID.data());
    uint64_t govt_code =
        dbf_get_uint_by_field(mtd_area_handle, i, GOVT_CODE.data());
    g_area_to_govt_code_map.emplace(area_id, govt_code);
  }
  DBFClose(mtd_area_handle);
}

void StreetConverter::init_g_cntry_ref_map(const boost::filesystem::path &dir) {
  DBFHandle cntry_ref_handle = read_dbf_file(dir / MTD_CNTRY_REF_DBF);
  for (int i = 0; i < DBFGetRecordCount(cntry_ref_handle); i++) {
    uint64_t govt_code =
        dbf_get_uint_by_field(cntry_ref_handle, i, GOVT_CODE.data());
    auto unit_measure =
        dbf_get_string_by_field(cntry_ref_handle, i, UNTMEASURE.data());
    auto speed_limit_unit =
        dbf_get_string_by_field(cntry_ref_handle, i, SPEEDLIMITUNIT.data());
    auto iso_code =
        dbf_get_string_by_field(cntry_ref_handle, i, ISO_CODE.data());
    auto cntry_ref = cntry_ref_type(unit_measure, speed_limit_unit, iso_code);
    g_cntry_ref_map.emplace(govt_code, cntry_ref);
  }
  DBFClose(cntry_ref_handle);
}

void StreetConverter::process_way_end_nodes(
    const std::vector<boost::filesystem::path> &dirs,
    const std::map<uint64_t, std::vector<z_lvl_index_type_t>> &z_level_map,
    osmium::io::Writer &writer) {
  for (auto &dir : dirs) {

    // parse ramp names and refs
    init_ramp_names(dir);

    auto path = dir / STREETS_SHP;
    auto ds = open_shape_file(path);

    auto layer = ds->GetLayer(0);
    if (layer == nullptr)
      throw(shp_empty_error(path.string()));

    osmium::memory::Buffer node_buffer(BUFFER_SIZE);

    int linkIDField = layer->FindFieldIndex(LINK_ID.data(), true);

    // get all nodes which may be a routable crossing
    for (auto &feat : *layer) {
      uint64_t link_id = feat->GetFieldAsInteger(linkIDField);
      // omit way end nodes with different z-levels (they have to be handled
      // extra)
      if (z_level_map.find(link_id) == z_level_map.end())
        process_way_end_nodes(feat, node_buffer);
    }
    node_buffer.commit();
    writer(std::move(node_buffer));
    g_ramps_ref_map.clear();
  }
}

void StreetConverter::process_way_end_nodes(
    OGRFeatureUniquePtr &feat, osmium::memory::Buffer &node_buffer) {

  auto ogr_ls = static_cast<const OGRLineString *>(feat->GetGeometryRef());

  process_way_end_node(osmium::Location(ogr_ls->getX(0), ogr_ls->getY(0)),
                       node_buffer);
  process_way_end_node(
      osmium::Location(ogr_ls->getX(ogr_ls->getNumPoints() - 1),
                       ogr_ls->getY(ogr_ls->getNumPoints() - 1)),
      node_buffer);
}

void StreetConverter::process_way_end_node(
    const osmium::Location &location, osmium::memory::Buffer &node_buffer) {

  auto it = g_way_end_points_map.find(location);
  if (it != g_way_end_points_map.end())
    return;

  osmium::builder::NodeBuilder builder(node_buffer);
  osmium::unsigned_object_id_type osm_id = build_node(location, builder);
  // add ramp tags
  auto ramp = g_ramps_ref_map.find(location);
  if (ramp != g_ramps_ref_map.end()) {
    if (ramp->second.find(0) != ramp->second.end()) {
      osmium::builder::TagListBuilder tglBuilder(builder);
      tglBuilder.add_tag(HIGHWAY, "motorway_junction");
      tglBuilder.add_tag("ref", ramp->second[0]);
      tglBuilder.add_tag("name", to_camel_case_with_spaces(ramp->second[1]));
    }
  }
  g_way_end_points_map.emplace(location, osm_id);
}

void StreetConverter::init_ramp_names(const boost::filesystem::path &dir) {
  // read junction names from alt_streets
  auto junctionMap = read_junction_names(dir / ALT_STREETS_DBF);

  // create location ramps map
  parse_ramp_names(dir / STREETS_SHP, junctionMap);
}

std::map<uint64_t, std::string>
StreetConverter::read_junction_names(const boost::filesystem::path &dbf_file) {
  DBFHandle hwys_handle = read_dbf_file(dbf_file);
  std::map<uint64_t, std::string> junctionNames;
  for (int i = 0; i < DBFGetRecordCount(hwys_handle); i++) {

    uint64_t link_id = dbf_get_uint_by_field(hwys_handle, i, LINK_ID.data());
    std::string ramp_name =
        dbf_get_string_by_field(hwys_handle, i, ST_NM_BASE.data());

    if (parse_bool(dbf_get_string_by_field(hwys_handle, i, JUNCTIONNM.data())
                       .c_str())) {
      junctionNames[link_id] = ramp_name;
    }
  }
  DBFClose(hwys_handle);

  return junctionNames;
}

void StreetConverter::parse_ramp_names(
    const boost::filesystem::path &shp_file,
    const std::map<uint64_t, std::string> &junctionNames) {

  auto ds = open_shape_file(shp_file);
  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(shp_file.string()));

  int exitNameField = layer->FindFieldIndex(EXITNAME.data(), true);
  int linkIdField = layer->FindFieldIndex(LINK_ID.data(), true);
  int baseNameField = layer->FindFieldIndex(ST_NM_BASE.data(), true);
  int directionField = layer->FindFieldIndex(DIR_TRAVEL, true);
  int rampField = layer->FindFieldIndex(RAMP, true);

  for (auto &feat : *layer) {

    if (!parse_bool(feat->GetFieldAsString(rampField))) {
      continue;
    }

    auto ogr_ls = static_cast<OGRLineString *>(feat->GetGeometryRef());

    auto location = osmium::Location(ogr_ls->getX(0), ogr_ls->getY(0));
    if (!strcmp(feat->GetFieldAsString(directionField), "T"))
      location = osmium::Location(ogr_ls->getX(ogr_ls->getNumPoints() - 1),
                                  ogr_ls->getY(ogr_ls->getNumPoints() - 1));

    if (parse_bool(feat->GetFieldAsString(exitNameField))) {
      std::string exitName = feat->GetFieldAsString(baseNameField);

      // add exit name
      g_ramps_ref_map[location].emplace(0, exitName);

      // add junction name
      auto it = junctionNames.find(feat->GetFieldAsInteger(linkIdField));
      if (it != junctionNames.end()) {
        g_ramps_ref_map[location].emplace(1, it->second);
      }
    }
  }
}

void StreetConverter::process_way(
    OGRFeatureUniquePtr &feat,
    const std::map<uint64_t, std::vector<z_lvl_index_type_t>> &z_level_map,
    osmium::memory::Buffer &node_buffer, osmium::memory::Buffer &way_buffer) {

  std::map<osmium::Location, osmium::unsigned_object_id_type> node_ref_map;

  auto ogr_ls = static_cast<OGRLineString *>(feat->GetGeometryRef());

  // creates remaining nodes required for way
  middle_points_preparation(ogr_ls, node_ref_map, node_buffer);
  if (ogr_ls->getNumPoints() > 2)
    assert(node_ref_map.size() > 0);

  uint64_t link_id = get_uint_from_feature(feat, LINK_ID.data());

  auto it = z_level_map.find(link_id);
  if (it == z_level_map.end()) {
    build_way(feat, ogr_ls, node_ref_map, way_buffer, false, -5);
  } else {
    auto &index_z_lvl_vector = it->second;

    // way with different z_levels
    auto &first_point_with_different_z_lvl = index_z_lvl_vector.front();
    auto first_index = 0;
    short first_z_lvl = 0;
    if (first_point_with_different_z_lvl.index == first_index)
      first_z_lvl = first_point_with_different_z_lvl.z_level;

    process_end_point(true, first_z_lvl, ogr_ls, node_ref_map, node_buffer);

    auto &last_point_with_different_z_lvl = index_z_lvl_vector.back();
    auto last_index = ogr_ls->getNumPoints() - 1;
    short last_z_lvl = 0;
    if (last_point_with_different_z_lvl.index == last_index)
      last_z_lvl = last_point_with_different_z_lvl.z_level;

    process_end_point(false, last_z_lvl, ogr_ls, node_ref_map, node_buffer);

    way_buffer.commit();

    if (is_ferry(get_field_from_feature(feat, FERRY)))
      set_ferry_z_lvls_to_zero(feat, index_z_lvl_vector);

    split_way_by_z_level(feat, ogr_ls, index_z_lvl_vector, node_ref_map,
                         link_id, way_buffer);
  }
}

void StreetConverter::middle_points_preparation(
    OGRLineString *ogr_ls,
    std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
    osmium::memory::Buffer &node_buffer) {
  // creates remaining nodes required for way
  for (int i = 1; i < ogr_ls->getNumPoints() - 1; ++i) {
    osmium::Location location(ogr_ls->getX(i), ogr_ls->getY(i));
    node_ref_map.emplace(location, build_node(location, node_buffer));
  }
  node_buffer.commit();
}

osmium::unsigned_object_id_type StreetConverter::build_way(
    OGRFeatureUniquePtr &feat, OGRLineString *ogr_ls,
    std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
    osmium::memory::Buffer &way_buffer, bool is_sub_linestring = false,
    short z_lvl = -5) {

  if (is_sub_linestring)
    test__z_lvl_range(z_lvl);

  osmium::builder::WayBuilder builder(way_buffer);
  setObjectProperties(builder);
  osmium::Location start(ogr_ls->getX(0), ogr_ls->getY(0));
  osmium::Location end(ogr_ls->getX(ogr_ls->getNumPoints() - 1),
                       ogr_ls->getY(ogr_ls->getNumPoints() - 1));

  {
    osmium::builder::WayNodeListBuilder wnl_builder(way_buffer, &builder);

    for (int i = 0; i < ogr_ls->getNumPoints(); i++) {
      osmium::Location location(ogr_ls->getX(i), ogr_ls->getY(i));
      bool is_end_point = i == 0 || i == ogr_ls->getNumPoints() - 1;
      std::map<osmium::Location, osmium::unsigned_object_id_type>
          *map_containing_node;
      if (!is_sub_linestring) {
        if (is_end_point)
          map_containing_node = &g_way_end_points_map;
        else
          map_containing_node = &node_ref_map;
      } else {
        if (node_ref_map.find(location) != node_ref_map.end()) {
          map_containing_node = &node_ref_map;
        } else {
          // node has to be in node_ref_map or way_end_points_map
          assert(g_way_end_points_map.find(location) !=
                 g_way_end_points_map.end());
          map_containing_node = &g_way_end_points_map;
        }
      }

      wnl_builder.add_node_ref(
          osmium::NodeRef(map_containing_node->at(location), location));
    }
  }

  build_tag_list(feat, &builder, way_buffer, z_lvl);
  return builder.object().id();
}

uint64_t StreetConverter::build_tag_list(OGRFeatureUniquePtr &feat,
                                         osmium::builder::Builder *builder,
                                         osmium::memory::Buffer &buf,
                                         short z_level) {
  osmium::builder::TagListBuilder tl_builder(buf, builder);

  uint64_t link_id = parse_street_tags(
      &tl_builder, feat, &g_cdms_map, &g_cnd_mod_map, &g_area_to_govt_code_map,
      &g_cntry_ref_map, &g_mtd_area_map, &g_route_type_map, &g_hwys_ref_map,
      g_construction_set, debugMode);

  if (z_level != -5 && z_level != 0)
    tl_builder.add_tag("layer", std::to_string(z_level));
  if (link_id == 0)
    throw(format_error("layers column field '" + std::string(LINK_ID.data()) +
                       "' is missing"));
  return link_id;
}

uint64_t StreetConverter::parse_street_tags(
    osmium::builder::TagListBuilder *builder, OGRFeatureUniquePtr &f,
    cdms_map_type *cdms_map, cnd_mod_map_type *cnd_mod_map,
    area_id_govt_code_map_type *area_govt_map, cntry_ref_map_type *cntry_map,
    mtd_area_map_type *mtd_area_map, link_id_route_type_map *route_type_map,
    link_id_to_names_map *names_map,
    const std::set<link_id_type> &construction_set, bool debugMode);
{

  const char *link_id_s = get_field_from_feature(f, LINK_ID);
  link_id_type link_id = std::stoul(link_id_s);

  bool ramp = parse_bool(get_field_from_feature(f, RAMP));

  ushort route_type = 0;
  if (!((std::string)get_field_from_feature(f, ROUTE)).empty())
    route_type = get_uint_from_feature(f, ROUTE);

  auto routeTypeIter = route_type_map->find(link_id);
  if (routeTypeIter != route_type_map->end() &&
      (!route_type || routeTypeIter->second < route_type))
    route_type = routeTypeIter->second;

  // add tags for ref and int_ref to major highways
  std::string ref_name =
      add_highway_name_tags(builder, names_map, link_id, ramp);

  bool underConstruction =
      (construction_set.find(link_id) != construction_set.end());

  if (is_ferry(get_field_from_feature(f, FERRY))) {
    add_ferry_tag(builder, f);
  } else { // usual highways
    add_highway_tags(builder, f, route_type, mtd_area_map, ref_name,
                     underConstruction);
  }

  uint64_t l_area_id = get_uint_from_feature(f, L_AREA_ID);
  uint64_t r_area_id = get_uint_from_feature(f, R_AREA_ID);
  // tags which apply to highways and ferry routes
  add_additional_restrictions(builder, link_id, l_area_id, r_area_id, cdms_map,
                              cnd_mod_map, area_govt_map, cntry_map,
                              mtd_area_map);

  // tag for debug purpose
  if (debugMode) {
    builder->add_tag(LINK_ID, link_id_s);
    add_here_speed_cat_tag(builder, f);
    if (parse_bool(get_field_from_feature(f, TOLLWAY)))
      builder->add_tag("here:tollway", YES);
    if (parse_bool(get_field_from_feature(f, URBAN)))
      builder->add_tag("here:urban", YES);
    if (parse_bool(get_field_from_feature(f, CONTRACC)))
      builder->add_tag("here:controll_access", YES);
    if (route_type)
      add_uint_tag(builder, "here:route_type", route_type);

    std::string func_class = get_field_from_feature(f, FUNC_CLASS);
    if (!func_class.empty())
      builder->add_tag("here:func_class", func_class.c_str());

    add_uint_tag(builder, "here:area_code", get_area_code_l(f, mtd_area_map));
  }
  return link_id;
}

void StreetConverter::add_additional_restrictions(
    osmium::builder::TagListBuilder *builder, uint64_t link_id,
    uint64_t l_area_id, uint64_t r_area_id, cdms_map_type *cdms_map,
    cnd_mod_map_type *cnd_mod_map, area_id_govt_code_map_type *area_govt_map,
    cntry_ref_map_type *cntry_map, mtd_area_map_type *mtd_area_map) {
  if (!cdms_map || !cnd_mod_map)
    return;

  // default is metric units
  bool imperial_units = false;
  if (area_govt_map && cntry_map) {
    imperial_units =
        is_imperial(l_area_id, r_area_id, area_govt_map, cntry_map);
  }

  uint64_t max_height = 0, max_width = 0, max_length = 0, max_weight = 0,
           max_axleload = 0;

  std::vector<mod_group_type> mod_group_vector;
  auto range = cdms_map->equal_range(link_id);
  for (auto it = range.first; it != range.second; ++it) {
    cond_pair_type cond = it->second;
    if (cond.second == CT_RESTRICTED_DRIVING_MANOEUVRE ||
        cond.second == CT_TRANSPORT_RESTRICTED_DRIVING_MANOEUVRE)
      continue; // TODO RESTRICTED_DRIVING_MANOEUVRE should apply as
                // conditional turn restriction but not for current link id
    auto it2 = cnd_mod_map->find(cond.first);
    if (it2 != cnd_mod_map->end()) {
      for (auto mod_group : it2->second) {
        mod_group_vector.push_back(mod_group);
      }
    }
  }

  for (auto mod_group : mod_group_vector) {
    auto mod_type = mod_group.mod_type;
    auto mod_val = mod_group.mod_val;
    if (mod_type == MT_HEIGHT_RESTRICTION) {
      if (!max_height || mod_val < max_height)
        max_height = mod_val;
    } else if (mod_type == MT_WIDTH_RESTRICTION) {
      if (!max_width || mod_val < max_width)
        max_width = mod_val;
    } else if (mod_type == MT_LENGTH_RESTRICTION) {
      if (!max_length || mod_val < max_length)
        max_length = mod_val;
    } else if (mod_type == MT_WEIGHT_RESTRICTION) {
      if (!max_weight || mod_val < max_weight)
        max_weight = mod_val;
    } else if (mod_type == MT_WEIGHT_PER_AXLE_RESTRICTION) {
      if (!max_axleload || mod_val < max_axleload)
        max_axleload = mod_val;
    } else if (mod_type == MT_HAZARDOUS_RESTRICTION) {
      add_hazmat_tag(builder, mod_val);
    }
  }

  if (get_area_code_l(l_area_id, r_area_id, mtd_area_map) == 107) {
    /** exceptional handling for Sweden as there are BK Roads
     *
     * HERE tags these roads with the most conservative values,
     * which would make it unroutable for nearly every truck.
     * Therefore we use the highest value and add a marker for BK2 / BK3 */
    if (max_weight == 16000 && max_axleload == 10000) {
      builder->add_tag("maxweight:class", "BK2");
      max_weight = 51400;
    } else if (max_weight == 12000 && max_axleload == 8000) {
      builder->add_tag("maxweight:class", "BK3");
      max_weight = 37000;
    }
  }

  if (max_height > 0)
    builder->add_tag("maxheight", imperial_units ? inch_to_feet(max_height)
                                                 : cm_to_m(max_height));
  if (max_width > 0)
    builder->add_tag("maxwidth", imperial_units ? inch_to_feet(max_width)
                                                : cm_to_m(max_width));
  if (max_length > 0)
    builder->add_tag("maxlength", imperial_units ? inch_to_feet(max_length)
                                                 : cm_to_m(max_length));
  if (max_weight > 0)
    builder->add_tag("maxweight", imperial_units ? lbs_to_metric_ton(max_weight)
                                                 : kg_to_t(max_weight));
  if (max_axleload > 0)
    builder->add_tag("maxaxleload", imperial_units
                                        ? lbs_to_metric_ton(max_axleload)
                                        : kg_to_t(max_axleload));
}

void StreetConverter::process_end_point(
    bool first, short z_lvl, OGRLineString *ogr_ls,
    std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
    osmium::memory::Buffer &node_buffer) {
  ushort i = first ? 0 : ogr_ls->getNumPoints() - 1;
  osmium::Location location(ogr_ls->getX(i), ogr_ls->getY(i));

  if (z_lvl != 0) {
    auto node_id = std::make_pair(location, z_lvl);
    auto it = g_z_lvl_nodes_map.find(node_id);
    if (it != g_z_lvl_nodes_map.end()) {
      node_ref_map.emplace(location, it->second);
    } else {
      osmium::unsigned_object_id_type osm_id =
          build_node(location, node_buffer);
      node_ref_map.emplace(location, osm_id);
      g_z_lvl_nodes_map.emplace(node_id, osm_id);
    }
  } else {
    // adds all zero z-level end points to g_way_end_points_map
    g_way_end_points_map.emplace(location, build_node(location, node_buffer));
  }
}

void StreetConverter::split_way_by_z_level(
    OGRFeatureUniquePtr &feat, OGRLineString *ogr_ls,
    const std::vector<z_lvl_index_type_t> &node_z_level_vector,
    std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
    uint64_t link_id, osmium::memory::Buffer &way_buffer) {

  ushort first_index = 0, last_index = ogr_ls->getNumPoints() - 1;
  ushort start_index = node_z_level_vector.cbegin()->index;
  if (start_index > 0)
    start_index--;

  // first_index <= start_index < end_index <= last_index
  assert(first_index <= start_index);
  assert(start_index < last_index);

  //	if (DEBUG) print_z_level_map(link_id, true);
  if (first_index != start_index) {
    build_sub_way_by_index(feat, ogr_ls, first_index, start_index, node_ref_map,
                           way_buffer, 0);
    BOOST_LOG_TRIVIAL(debug)
        << " 1 ## " << link_id << " ## " << first_index << "/" << last_index
        << "  -  " << start_index << "/" << last_index << ": \tz_lvl=" << 0
        << std::endl;
  }

  start_index = create_continuing_sub_ways(
      feat, ogr_ls, first_index, start_index, last_index, link_id,
      node_z_level_vector, node_ref_map, way_buffer);

  if (start_index < last_index) {
    build_sub_way_by_index(feat, ogr_ls, start_index, last_index, node_ref_map,
                           way_buffer, 0);
    BOOST_LOG_TRIVIAL(debug)
        << " 4 ## " << link_id << " ## " << start_index << "/" << last_index
        << "  -  " << last_index << "/" << last_index << ": \tz_lvl=" << 0
        << std::endl;
  }
}

void StreetConverter::build_sub_way_by_index(
    OGRFeatureUniquePtr &feat, OGRLineString *ogr_ls, ushort start_index,
    ushort end_index,
    std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
    osmium::memory::Buffer &way_buffer, short z_lvl = 0) {
  assert(start_index < end_index || end_index == -1);
  assert(start_index < ogr_ls->getNumPoints());
  OGRLineString subLineString;
  subLineString.addSubLineString(ogr_ls, start_index, end_index);
  build_way(feat, &subLineString, node_ref_map, way_buffer, true, z_lvl);
}

ushort StreetConverter::create_continuing_sub_ways(
    OGRFeatureUniquePtr &feat, OGRLineString *ogr_ls, ushort first_index,
    ushort start_index, ushort last_index, uint link_id,
    const std::vector<z_lvl_index_type_t> &node_z_level_vector,
    std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
    osmium::memory::Buffer &way_buffer) {

  for (auto it = node_z_level_vector.cbegin(); it != node_z_level_vector.cend();
       ++it) {
    short z_lvl = it->z_level;
    test__z_lvl_range(z_lvl);
    bool last_element = node_z_level_vector.cend() - 1 == it;
    bool not_last_element = !last_element;
    ushort index = it->index;
    ushort next_index;
    short next_z_lvl;
    if (not_last_element) {
      auto next_it = it + 1;
      next_index = next_it->index;
      next_z_lvl = next_it->z_level;
      test__z_lvl_range(next_z_lvl);
    }
    BOOST_LOG_TRIVIAL(debug)
        << "first_index=" << first_index << "   "
        << "start_index=" << start_index << "   " << "last_index=" << last_index
        << "   " << "index=" << index << "   " << "z_lvl=" << z_lvl << "   "
        << "next_z_lvl=" << next_z_lvl << std::endl;

    if (not_last_element) {
      if (index + 2 == next_index && z_lvl == next_z_lvl)
        continue;
      bool not_second_last_element = it + 2 != node_z_level_vector.cend();
      if (not_second_last_element) {
        ushort second_next_index = (it + 2)->index;
        short second_next_z_lvl = (it + 2)->z_level;
        test__z_lvl_range(second_next_z_lvl);
        if (index + 2 == second_next_index &&
            is_superior_or_equal(second_next_z_lvl, next_z_lvl) &&
            z_lvl == second_next_z_lvl) {
          ++it;
          continue;
        }
      }
    }

    // checks for gaps within the way
    if (last_element || index + 1 < next_index || z_lvl != next_z_lvl) {
      ushort from = start_index;
      ushort to;
      if (last_element || index + 1 < next_index ||
          is_superior(z_lvl, next_z_lvl))
        to = std::min((ushort)(index + 1), last_index);
      else
        to = index;
      BOOST_LOG_TRIVIAL(debug)
          << " 2 ## " << link_id << " ## " << from << "/" << last_index
          << "  -  " << to << "/" << last_index << ": \tz_lvl=" << z_lvl
          << std::endl;
      if (from < to) {
        build_sub_way_by_index(feat, ogr_ls, from, to, node_ref_map, way_buffer,
                               z_lvl);
        start_index = to;
      }

      if (not_last_element && to < next_index - 1) {
        build_sub_way_by_index(feat, ogr_ls, to, next_index - 1, node_ref_map,
                               way_buffer);
        BOOST_LOG_TRIVIAL(debug)
            << " 3 ## " << link_id << " ## " << to << "/" << last_index
            << "  -  " << next_index - 1 << "/" << last_index
            << ": \tz_lvl=" << 0 << std::endl;
        start_index = next_index - 1;
      }
    }
  }
  return start_index;
}
