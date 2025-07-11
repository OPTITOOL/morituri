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

#include <boost/json.hpp>
#include <boost/log/trivial.hpp>
#include <fstream>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/io/writer.hpp>
#include <osmium/memory/buffer.hpp>
#include <osmium/osm/location.hpp>
#include <osmium/osm/types.hpp>

#include "../../comm2osm_exceptions.hpp"

std::map<osmium::Location, osmium::unsigned_object_id_type>
    StreetConverter::g_regionConnectingPoints;

std::mutex StreetConverter::g_regionConnectingPoints_mutex;

StreetConverter::StreetConverter(const std::filesystem::path &executable_path)
    : Converter(executable_path) {}

StreetConverter::~StreetConverter() {}

void StreetConverter::convert(const std::filesystem::path &dir,
                              osmium::io::Writer &writer) {

  BOOST_LOG_TRIVIAL(info) << " processing alt_streets route types";
  auto route_type_map = process_alt_steets_route_types(dir);

  BOOST_LOG_TRIVIAL(info) << " processing conditional modifications ";
  auto cdms_map = init_cdms_map(dir);
  init_cnd_mod(cdms_map, dir);
  init_cnd_dt_mod(cdms_map, dir);

  // patch the conditions with a additional json file
  // this is a workaround for the informations that are not fixed by HERE yet
  manipulate_cdms_map(cdms_map, dir);

  BOOST_LOG_TRIVIAL(info) << " processing country reference";
  auto cntry_ref_map = init_g_cntry_ref_map(dir);
  auto area_to_govt_code_map = init_g_area_to_govt_code_map(cntry_ref_map, dir);

  BOOST_LOG_TRIVIAL(info) << " processing ramps";
  auto ramps_ref_map = init_ramp_names(dir);

  BOOST_LOG_TRIVIAL(info) << " processing highway names";
  auto hwys_ref_map = init_highway_names(dir);

  // fill data struct for tag processing
  TagData data = {route_type_map, cdms_map,      area_to_govt_code_map,
                  cntry_ref_map,  ramps_ref_map, hwys_ref_map};

  BOOST_LOG_TRIVIAL(info) << " processing z-levels";
  auto z_level_map = init_z_level_map(dir);

  update_region_connecting_points(g_regionConnectingPoints, data, dir, writer);

  BOOST_LOG_TRIVIAL(info) << " processing way end points";
  auto way_end_points_map = process_way_end_nodes(
      dir, data, g_regionConnectingPoints, z_level_map, writer);

  BOOST_LOG_TRIVIAL(info) << " processing ways";
  process_way(dir, data, way_end_points_map, z_level_map, writer);
}

void StreetConverter::manipulate_cdms_map(
    std::multimap<uint64_t, cond_type> &cdms_map,
    const std::filesystem::path &dir) {

  std::string filePrefix = dir.parent_path().filename();

  // get the first 4 characters of the file prefix
  std::string shortPrefix = filePrefix.substr(0, 4);

  for (auto const &dir_entry :
       std::filesystem::directory_iterator{executable_path}) {
    if (dir_entry.is_directory())
      continue;

    if (dir_entry.path().extension() != ".json")
      continue;

    if (dir_entry.path().filename().string().find(shortPrefix) ==
        std::string::npos)
      continue;

    BOOST_LOG_TRIVIAL(info) << " processing json file: " << dir_entry.path();

    std::ifstream ifs(dir_entry.path());
    auto jv = boost::json::parse(ifs);

    auto jsonPatch = jv.get_object();
    if (jsonPatch["cdms"].is_null())
      return;

    // check if all Ids are in the cdms_map
    auto cdmsJson = jsonPatch["cdms"].get_object();

    auto condToDelete =
        boost::json::value_to<std::vector<uint64_t>>(cdmsJson["delete"]);

    for (auto const &condId : condToDelete) {
      auto it = std::ranges::find_if(cdms_map, [condId](auto const &entry) {
        return entry.second.cond_id_type == condId;
      });
      if (it == cdms_map.end()) {
        BOOST_LOG_TRIVIAL(error)
            << "Patch Failed: Conditional modification with id " << condId
            << " not found in database! See " << dir_entry.path();
        exit(1);
      }
      cdms_map.erase(it);
    }
  }
}

std::map<uint64_t, ushort> StreetConverter::process_alt_steets_route_types(
    const std::filesystem::path &dir) {

  std::map<uint64_t, ushort> route_type_map;

  auto ds = openDataSource(dir / ALT_STREETS_DBF);
  if (!ds)
    return route_type_map;

  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error((dir / ALT_STREETS_DBF).string()));

  for (auto &feat : *layer) {

    std::string route = get_field_from_feature(feat, ROUTE);
    if (route.empty())
      continue;

    osmium::unsigned_object_id_type link_id =
        get_uint_from_feature(feat, LINK_ID);
    ushort route_type = get_uint_from_feature(feat, ROUTE);

    // try to emplace <link_id, route_type> pair
    auto [insertion, inserted] = route_type_map.emplace(link_id, route_type);

    // if its already exists update routetype
    if (!inserted && insertion->second > route_type) {
      // As link id's aren't unique in AltStreets.dbf
      // just store the lowest route type
      insertion->second = route_type;
    }
  }

  return route_type_map;
}

std::map<uint64_t, std::vector<StreetConverter::z_lvl_index_type_t>>
StreetConverter::init_z_level_map(const std::filesystem::path &dir) {

  std::map<uint64_t, std::vector<z_lvl_index_type_t>> z_level_map;

  const std::filesystem::path ZLEVELS_DBF = "Zlevels.dbf";

  auto ds = openDataSource(dir / ZLEVELS_DBF);
  if (!ds)
    throw(shp_error(dir / ZLEVELS_DBF));

  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(dir / ZLEVELS_DBF));

  uint64_t last_link_id = 0;
  std::vector<z_lvl_index_type_t> v;

  for (auto &feat : *layer) {
    uint64_t link_id = get_uint_from_feature(feat, LINK_ID);
    ushort point_num = get_uint_from_feature(feat, POINT_NUM) - 1;
    short z_level = get_uint_from_feature(feat, Z_LEVEL);

    if (last_link_id != link_id && !v.empty()) {
      z_level_map.emplace(last_link_id, v);
      v = std::vector<z_lvl_index_type_t>();
    }
    if (z_level != 0)
      v.emplace_back(point_num, z_level);
    last_link_id = link_id;
  }

  if (!v.empty())
    z_level_map.emplace(last_link_id, v);

  return z_level_map;
}

void StreetConverter::init_cnd_mod(
    std::multimap<uint64_t, StreetConverter::cond_type> &cdms_map,
    const std::filesystem::path &dir) {
  const std::filesystem::path CND_MOD_DBF = "CndMod.dbf";

  std::map<uint64_t, cond_type *> conditions;
  for (auto &entry : cdms_map) {
    conditions.emplace(entry.second.cond_id_type, &entry.second);
  }

  auto ds = openDataSource(dir / CND_MOD_DBF);
  if (!ds)
    return;

  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(dir / CND_MOD_DBF));

  for (auto &feat : *layer) {
    uint64_t cond_id = get_uint_from_feature(feat, COND_ID);

    // only process conditional modifications for the given cond_ids
    auto it = conditions.find(cond_id);
    if (it == conditions.end())
      continue;

    std::string lang_code = get_field_from_feature(feat, LANG_CODE);
    uint64_t mod_type = get_uint_from_feature(feat, CM_MOD_TYPE);
    uint64_t mod_val = get_uint_from_feature(feat, CM_MOD_VAL);

    it->second->mod_group_map.emplace(
        mod_type, mod_group_type(mod_type, mod_val, lang_code));
  }
}

void StreetConverter::init_cnd_dt_mod(
    std::multimap<uint64_t, StreetConverter::cond_type> &cdms_map,
    const std::filesystem::path &dir) {
  const std::filesystem::path CND_DT_MOD_DBF = "CdmsDtmod.dbf";

  std::map<uint64_t, cond_type *> conditions;
  for (auto &entry : cdms_map) {
    conditions.emplace(entry.second.cond_id_type, &entry.second);
  }

  auto ds = openDataSource(dir / CND_DT_MOD_DBF);
  if (!ds)
    return;

  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(dir / CND_DT_MOD_DBF));

  for (auto &feat : *layer) {
    uint64_t cond_id = get_uint_from_feature(feat, COND_ID);

    // only process conditional modifications for the given cond_ids
    auto it = conditions.find(cond_id);
    if (it == conditions.end())
      continue;

    it->second->dt_mod.hasDateTimeMod = true;
  }
}

std::multimap<uint64_t, StreetConverter::cond_type>
StreetConverter::init_cdms_map(const std::filesystem::path &dir) {

  std::multimap<uint64_t, cond_type> cdms_map;

  const std::filesystem::path CDMS_DBF = "Cdms.dbf";
  auto ds = openDataSource(dir / CDMS_DBF);
  if (!ds)
    return cdms_map;

  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(dir / CDMS_DBF));

  for (auto &feat : *layer) {

    ushort cond_type = get_uint_from_feature(feat, COND_TYPE);

    // only store considered conditional modifications
    if (cond_type != CT_TRANSPORT_ACCESS_RESTRICTION &&
        cond_type != CT_CONSTRUCTION_STATUS_CLOSED &&
        cond_type != CT_TRANSPORT_SPECIAL_SPEED_SITUATION)
      continue;

    if (!need_to_consider(feat))
      continue;

    bool hgv_only = is_hgv_only(feat);

    uint64_t link_id = get_uint_from_feature(feat, LINK_ID);
    uint64_t cond_id = get_uint_from_feature(feat, COND_ID);
    cdms_map.emplace(link_id, StreetConverter::cond_type{
                                  cond_id, cond_type, hgv_only, {}, {}});
  }

  return cdms_map;
}

std::map<uint64_t, uint64_t> StreetConverter::init_g_area_to_govt_code_map(
    std::map<uint64_t, StreetConverter::cntry_ref_type> &cntry_ref_map,
    const std::filesystem::path &dir) {

  std::map<uint64_t, uint64_t> area_to_area_code_map;

  const std::filesystem::path MTD_AREA_DBF = "MtdArea.dbf";

  auto ds = openDataSource(dir / MTD_AREA_DBF);
  if (!ds)
    return area_to_area_code_map;

  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(dir / MTD_AREA_DBF));

  for (auto &feat : *layer) {
    std::string area_type = get_field_from_feature(feat, "AREA_TYPE");
    if (area_type != "B")
      continue;

    uint64_t area_id = get_uint_from_feature(feat, AREA_ID);
    uint64_t areacode_1 = get_uint_from_feature(feat, AREA_CODE_1);
    uint64_t govt_code = get_uint_from_feature(feat, GOVT_CODE);
    uint64_t admin_level = get_uint_from_feature(feat, ADMIN_LVL);

    // found the area id of the admin 1 area
    if (auto cntry_ref = cntry_ref_map.find(govt_code);
        admin_level == 1 && cntry_ref != cntry_ref_map.end()) {
      // store the area code of the admin 1 area in cntry_ref_map
      if (cntry_ref->second.area_code_1 == 0) {
        cntry_ref->second.area_code_1 = areacode_1;
      } else if (cntry_ref->second.area_code_1 != areacode_1) {
        BOOST_LOG_TRIVIAL(error)
            << "Area code already set for govt code: " << govt_code << " old "
            << cntry_ref->second.area_code_1 << " new " << areacode_1;
      }
    }

    area_to_area_code_map.emplace(area_id, areacode_1);
  }

  return area_to_area_code_map;
}

std::map<uint64_t, StreetConverter::cntry_ref_type>
StreetConverter::init_g_cntry_ref_map(const std::filesystem::path &dir) {

  std::map<uint64_t, cntry_ref_type> cntry_ref_map;

  static const std::filesystem::path MTD_CNTRY_REF_DBF = "MtdCntryRef.dbf";

  auto ds = openDataSource(dir / MTD_CNTRY_REF_DBF);
  if (!ds)
    return cntry_ref_map;

  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(dir / MTD_CNTRY_REF_DBF));

  for (auto &feat : *layer) {
    uint64_t govt_code = get_uint_from_feature(feat, GOVT_CODE);
    std::string unit_measure = get_field_from_feature(feat, UNTMEASURE);
    std::string speed_limit_unit = get_field_from_feature(feat, SPEEDLIMITUNIT);
    std::string iso_code = get_field_from_feature(feat, ISO_CODE);
    cntry_ref_type cntry_ref(unit_measure, speed_limit_unit, iso_code);
    cntry_ref_map.emplace(govt_code, cntry_ref);
  }

  return cntry_ref_map;
}

std::map<osmium::Location, osmium::unsigned_object_id_type>
StreetConverter::process_way_end_nodes(
    const std::filesystem::path &dir, const StreetConverter::TagData &data,
    const std::map<osmium::Location, osmium::unsigned_object_id_type>
        &regionConnectingPoints,
    std::map<uint64_t, std::vector<StreetConverter::z_lvl_index_type_t>>
        &z_level_map,
    osmium::io::Writer &writer) {

  g_regionConnectingPoints_mutex.lock();
  std::map<osmium::Location, osmium::unsigned_object_id_type>
      way_end_points_map(regionConnectingPoints);
  g_regionConnectingPoints_mutex.unlock();

  auto path = dir / STREETS_SHP;
  auto ds = openDataSource(path);
  if (!ds)
    throw(shp_error(path.string()));

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
    if (z_level_map.find(link_id) != z_level_map.end())
      continue;

    process_way_end_nodes(feat, data, way_end_points_map, node_buffer);
  }
  node_buffer.commit();
  {
    std::lock_guard<std::mutex> lock(osmiumWriterMutex);
    writer(std::move(node_buffer));
  }

  return way_end_points_map;
}

void StreetConverter::process_way_end_nodes(
    OGRFeatureUniquePtr &feat, const StreetConverter::TagData &data,
    std::map<osmium::Location, osmium::unsigned_object_id_type>
        &way_end_points_map,
    osmium::memory::Buffer &node_buffer) {

  auto ogr_ls = static_cast<const OGRLineString *>(feat->GetGeometryRef());

  process_way_end_node(osmium::Location(ogr_ls->getX(0), ogr_ls->getY(0)), data,
                       way_end_points_map, false, node_buffer);
  process_way_end_node(
      osmium::Location(ogr_ls->getX(ogr_ls->getNumPoints() - 1),
                       ogr_ls->getY(ogr_ls->getNumPoints() - 1)),
      data, way_end_points_map, false, node_buffer);
}

void StreetConverter::process_way_end_node(
    const osmium::Location &location, const StreetConverter::TagData &data,
    std::map<osmium::Location, osmium::unsigned_object_id_type>
        &way_end_points_map,
    bool gloablEndPoints, osmium::memory::Buffer &node_buffer) {

  auto it = way_end_points_map.find(location);
  if (it != way_end_points_map.end())
    return;

  osmium::builder::NodeBuilder builder(node_buffer);
  osmium::unsigned_object_id_type osm_id = build_node(location, builder);
  // add ramp tags
  auto ramp = data.ramp_names.find(location);
  if (ramp != data.ramp_names.end()) {
    auto &[exitName, junctionName] = ramp->second;
    osmium::builder::TagListBuilder tglBuilder(builder);
    tglBuilder.add_tag(HIGHWAY.data(), "motorway_junction");
    tglBuilder.add_tag("ref", exitName);
    if (!junctionName.empty())
      tglBuilder.add_tag("name", junctionName);
  }

  if (gloablEndPoints && debugMode) {
    osmium::builder::TagListBuilder tglBuilder(builder);
    tglBuilder.add_tag("region_end", "yes");
  }

  way_end_points_map.emplace(location, osm_id);
}

std::map<osmium::Location, std::tuple<std::string, std::string>>
StreetConverter::init_ramp_names(const std::filesystem::path &dir) {

  std::map<osmium::Location, std::tuple<std::string, std::string>>
      ramps_ref_map;
  // read junction names from alt_streets
  auto junctionMap = read_junction_names(dir / ALT_STREETS_DBF);

  // create location ramps map
  parse_ramp_names(dir / STREETS_SHP, ramps_ref_map, junctionMap);

  return ramps_ref_map;
}

std::map<uint64_t, std::string>
StreetConverter::read_junction_names(const std::filesystem::path &dbf_file) {

  auto ds = openDataSource(dbf_file);
  if (!ds)
    throw(shp_error(dbf_file.string()));

  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(dbf_file.string()));

  std::map<uint64_t, std::string> junctionNames;

  for (auto &feat : *layer) {
    if (get_bool_from_feature(feat, JUNCTIONNM)) {
      uint64_t link_id = get_uint_from_feature(feat, LINK_ID);
      junctionNames[link_id] = get_field_from_feature(feat, ST_NM_BASE);
    }
  }

  return junctionNames;
}

void StreetConverter::parse_ramp_names(
    const std::filesystem::path &shp_file,
    std::map<osmium::Location, std::tuple<std::string, std::string>>
        &ramps_ref_map,
    const std::map<uint64_t, std::string> &junctionNames) {

  auto ds = openDataSource(shp_file);
  if (!ds)
    return;

  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(shp_file.string()));

  int linkIdField = layer->FindFieldIndex(LINK_ID.data(), true);
  int baseNameField = layer->FindFieldIndex(ST_NM_BASE.data(), true);
  int directionField = layer->FindFieldIndex(DIR_TRAVEL.data(), true);

  for (auto &feat : *layer) {

    if (!get_bool_from_feature(feat, RAMP)) {
      continue;
    }

    auto ogr_ls = static_cast<OGRLineString *>(feat->GetGeometryRef());

    auto location = osmium::Location(ogr_ls->getX(0), ogr_ls->getY(0));
    if (!strcmp(feat->GetFieldAsString(directionField), "T"))
      location = osmium::Location(ogr_ls->getX(ogr_ls->getNumPoints() - 1),
                                  ogr_ls->getY(ogr_ls->getNumPoints() - 1));

    if (get_bool_from_feature(feat, EXITNAME)) {
      std::string exitName = feat->GetFieldAsString(baseNameField);

      // add junction name
      auto it = junctionNames.find(feat->GetFieldAsInteger(linkIdField));
      if (it != junctionNames.end()) {
        ramps_ref_map.emplace(location, std::make_tuple(exitName, it->second));
      } else {
        ramps_ref_map.emplace(location, std::make_tuple(exitName, ""));
      }
    }
  }
}

void StreetConverter::process_way(
    const std::filesystem::path &dir, const StreetConverter::TagData &data,
    std::map<osmium::Location, osmium::unsigned_object_id_type>
        &way_end_points_map,
    std::map<uint64_t, std::vector<z_lvl_index_type_t>> &z_level_map,
    osmium::io::Writer &writer) {

  std::map<std::pair<osmium::Location, short>, osmium::unsigned_object_id_type>
      z_lvl_nodes_map;

  auto path = dir / STREETS_SHP;
  auto ds = openDataSource(path);
  if (!ds)
    throw(shp_error(path.string()));

  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(path.string()));

  osmium::memory::Buffer node_buffer(BUFFER_SIZE);
  osmium::memory::Buffer way_buffer(BUFFER_SIZE);
  for (auto &feat : *layer) {
    process_way(feat, data, way_end_points_map, z_lvl_nodes_map, z_level_map,
                node_buffer, way_buffer);
  }

  node_buffer.commit();
  way_buffer.commit();
  {
    std::lock_guard<std::mutex> lock(osmiumWriterMutex);
    writer(std::move(node_buffer));
    writer(std::move(way_buffer));
  }
}

std::map<uint64_t, std::map<uint, std::string>>
StreetConverter::init_highway_names(const std::filesystem::path &dir) {

  static const std::filesystem::path MAJ_HWYS_DBF = "MajHwys.dbf";
  static const std::filesystem::path SEC_HWYS_DBF = "SecHwys.dbf";
  static const std::filesystem::path STREETS_DBF = "Streets.dbf";

  std::map<uint64_t, std::map<uint, std::string>> hwys_ref_map;
  parse_highway_names(dir / MAJ_HWYS_DBF, hwys_ref_map, false);
  parse_highway_names(dir / SEC_HWYS_DBF, hwys_ref_map, false);
  parse_highway_names(dir / ALT_STREETS_DBF, hwys_ref_map, true);
  parse_highway_names(dir / STREETS_DBF, hwys_ref_map, true);

  return hwys_ref_map;
}

void StreetConverter::parse_highway_names(
    const std::filesystem::path &dbf_file,
    std::map<uint64_t, std::map<uint, std::string>> &hwys_ref_map,
    bool isStreetLayer) {

  auto ds = openDataSource(dbf_file);
  if (!ds)
    return;

  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(dbf_file.string()));

  for (auto &feat : *layer) {
    uint64_t link_id = get_uint_from_feature(feat, LINK_ID);
    std::string hwy_name;
    if (isStreetLayer)
      hwy_name = get_field_from_feature(feat, ST_NAME);
    else
      hwy_name = get_field_from_feature(feat, HIGHWAY_NM);
    uint routeType = get_uint_from_feature(feat, ROUTE);
    hwys_ref_map[link_id].emplace(routeType, hwy_name);
  }
}

void StreetConverter::process_way(
    OGRFeatureUniquePtr &feat, const StreetConverter::TagData &data,
    std::map<osmium::Location, osmium::unsigned_object_id_type>
        &way_end_points_map,
    std::map<std::pair<osmium::Location, short>,
             osmium::unsigned_object_id_type> &z_lvl_nodes_map,
    std::map<uint64_t, std::vector<z_lvl_index_type_t>> &z_level_map,
    osmium::memory::Buffer &node_buffer, osmium::memory::Buffer &way_buffer) {

  std::map<osmium::Location, osmium::unsigned_object_id_type> node_ref_map;

  auto ogr_ls = static_cast<OGRLineString *>(feat->GetGeometryRef());

  // creates remaining nodes required for way
  middle_points_preparation(ogr_ls, node_ref_map, node_buffer);
  if (ogr_ls->getNumPoints() > 2)
    assert(node_ref_map.size() > 0);

  uint64_t link_id = get_uint_from_feature(feat, LINK_ID);

  auto it = z_level_map.find(link_id);
  if (it == z_level_map.end()) {
    build_way(feat, ogr_ls, node_ref_map, data, way_end_points_map, way_buffer,
              false, -5);
  } else {
    auto &index_z_lvl_vector = it->second;

    // way with different z_levels
    auto &first_point_with_different_z_lvl = index_z_lvl_vector.front();
    auto first_index = 0;
    short first_z_lvl = 0;
    if (first_point_with_different_z_lvl.index == first_index)
      first_z_lvl = first_point_with_different_z_lvl.z_level;

    process_end_point(true, first_z_lvl, ogr_ls, node_ref_map,
                      way_end_points_map, z_lvl_nodes_map, node_buffer);

    auto &last_point_with_different_z_lvl = index_z_lvl_vector.back();
    auto last_index = ogr_ls->getNumPoints() - 1;
    short last_z_lvl = 0;
    if (last_point_with_different_z_lvl.index == last_index)
      last_z_lvl = last_point_with_different_z_lvl.z_level;

    process_end_point(false, last_z_lvl, ogr_ls, node_ref_map,
                      way_end_points_map, z_lvl_nodes_map, node_buffer);

    way_buffer.commit();

    if (is_ferry(get_field_from_feature(feat, FERRY)))
      set_ferry_z_lvls_to_zero(feat, index_z_lvl_vector);

    split_way_by_z_level(feat, ogr_ls, index_z_lvl_vector, node_ref_map, data,
                         way_end_points_map, link_id, way_buffer);
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
    const StreetConverter::TagData &data,
    std::map<osmium::Location, osmium::unsigned_object_id_type>
        &way_end_points_map,
    osmium::memory::Buffer &way_buffer, bool is_sub_linestring = false,
    short z_lvl = -5) {

  if (is_sub_linestring)
    test__z_lvl_range(z_lvl);

  osmium::builder::WayBuilder builder(way_buffer);
  setObjectProperties(builder);
  {
    osmium::builder::WayNodeListBuilder wnl_builder(way_buffer, &builder);

    for (int i = 0; i < ogr_ls->getNumPoints(); i++) {
      osmium::Location location(ogr_ls->getX(i), ogr_ls->getY(i));
      bool is_end_point = i == 0 || i == ogr_ls->getNumPoints() - 1;
      std::map<osmium::Location, osmium::unsigned_object_id_type>
          *map_containing_node;
      if (!is_sub_linestring) {
        if (is_end_point)
          map_containing_node = &way_end_points_map;
        else
          map_containing_node = &node_ref_map;
      } else {
        if (node_ref_map.find(location) != node_ref_map.end()) {
          map_containing_node = &node_ref_map;
        } else {
          // node has to be in node_ref_map or way_end_points_map
          assert(way_end_points_map.find(location) != way_end_points_map.end());
          map_containing_node = &way_end_points_map;
        }
      }

      wnl_builder.add_node_ref(
          osmium::NodeRef(map_containing_node->at(location), location));
    }
  }

  build_tag_list(feat, data, builder, z_lvl);
  return builder.object().id();
}

void StreetConverter::process_end_point(
    bool first, short z_lvl, OGRLineString *ogr_ls,
    std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
    std::map<osmium::Location, osmium::unsigned_object_id_type>
        &way_end_points_map,
    std::map<std::pair<osmium::Location, short>,
             osmium::unsigned_object_id_type> &z_lvl_nodes_map,
    osmium::memory::Buffer &node_buffer) {
  ushort i = first ? 0 : ogr_ls->getNumPoints() - 1;
  osmium::Location location(ogr_ls->getX(i), ogr_ls->getY(i));

  if (z_lvl != 0) {
    auto node_id = std::make_pair(location, z_lvl);
    auto it = z_lvl_nodes_map.find(node_id);
    if (it != z_lvl_nodes_map.end()) {
      node_ref_map.emplace(location, it->second);
    } else {

      osmium::unsigned_object_id_type osm_id;
      auto it = way_end_points_map.find(location);
      if (it != way_end_points_map.end()) {
        osm_id = it->second;
      } else {
        osm_id = build_node(location, node_buffer);
      }

      node_ref_map.emplace(location, osm_id);
      z_lvl_nodes_map.emplace(node_id, osm_id);
    }
  } else {

    if (way_end_points_map.find(location) != way_end_points_map.end())
      return;

    // adds all zero z-level end points to g_way_end_points_map
    way_end_points_map.emplace(location, build_node(location, node_buffer));
  }
}

void StreetConverter::split_way_by_z_level(
    OGRFeatureUniquePtr &feat, OGRLineString *ogr_ls,
    const std::vector<z_lvl_index_type_t> &node_z_level_vector,
    std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
    const StreetConverter::TagData &data,
    std::map<osmium::Location, osmium::unsigned_object_id_type>
        &way_end_points_map,
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
                           data, way_end_points_map, way_buffer, 0);
    if (debugMode)
      BOOST_LOG_TRIVIAL(debug)
          << " 1 ## " << link_id << " ## " << first_index << "/" << last_index
          << "  -  " << start_index << "/" << last_index << ": \tz_lvl=" << 0
          << std::endl;
  }

  start_index = create_continuing_sub_ways(
      feat, ogr_ls, first_index, start_index, last_index, link_id,
      node_z_level_vector, node_ref_map, data, way_end_points_map, way_buffer);

  if (start_index < last_index) {
    build_sub_way_by_index(feat, ogr_ls, start_index, last_index, node_ref_map,
                           data, way_end_points_map, way_buffer, 0);
    if (debugMode)
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
    const StreetConverter::TagData &data,
    std::map<osmium::Location, osmium::unsigned_object_id_type>
        &way_end_points_map,
    osmium::memory::Buffer &way_buffer, short z_lvl = 0) {
  assert(start_index < end_index || end_index == -1);
  assert(start_index < ogr_ls->getNumPoints());
  OGRLineString subLineString;
  subLineString.addSubLineString(ogr_ls, start_index, end_index);
  build_way(feat, &subLineString, node_ref_map, data, way_end_points_map,
            way_buffer, true, z_lvl);
}

ushort StreetConverter::create_continuing_sub_ways(
    OGRFeatureUniquePtr &feat, OGRLineString *ogr_ls, ushort first_index,
    ushort start_index, ushort last_index, uint link_id,
    const std::vector<z_lvl_index_type_t> &node_z_level_vector,
    std::map<osmium::Location, osmium::unsigned_object_id_type> &node_ref_map,
    const StreetConverter::TagData &data,
    std::map<osmium::Location, osmium::unsigned_object_id_type>
        &way_end_points_map,
    osmium::memory::Buffer &way_buffer) {

  for (auto it = node_z_level_vector.cbegin(); it != node_z_level_vector.cend();
       ++it) {
    short z_lvl = it->z_level;
    test__z_lvl_range(z_lvl);
    bool last_element = node_z_level_vector.cend() - 1 == it;
    bool not_last_element = !last_element;
    ushort index = it->index;
    short next_index = -1;
    short next_z_lvl = -1;
    if (not_last_element) {
      auto next_it = it + 1;
      next_index = next_it->index;
      next_z_lvl = next_it->z_level;
      test__z_lvl_range(next_z_lvl);
    }
    if (debugMode)
      BOOST_LOG_TRIVIAL(debug)
          << "first_index=" << first_index << "   "
          << "start_index=" << start_index << "   "
          << "last_index=" << last_index << "   " << "index=" << index << "   "
          << "z_lvl=" << z_lvl << "   " << "next_z_lvl=" << next_z_lvl
          << std::endl;

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
      if (debugMode)
        BOOST_LOG_TRIVIAL(debug)
            << " 2 ## " << link_id << " ## " << from << "/" << last_index
            << "  -  " << to << "/" << last_index << ": \tz_lvl=" << z_lvl
            << std::endl;
      if (from < to) {
        build_sub_way_by_index(feat, ogr_ls, from, to, node_ref_map, data,
                               way_end_points_map, way_buffer, z_lvl);
        start_index = to;
      }

      if (not_last_element && to < next_index - 1) {
        build_sub_way_by_index(feat, ogr_ls, to, next_index - 1, node_ref_map,
                               data, way_end_points_map, way_buffer);
        if (debugMode)
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

void StreetConverter::set_ferry_z_lvls_to_zero(
    const OGRFeatureUniquePtr &feat,
    std::vector<z_lvl_index_type_t> &z_lvl_vec) {
  // erase middle z_lvls
  if (z_lvl_vec.size() > 2)
    z_lvl_vec.erase(z_lvl_vec.begin() + 1, z_lvl_vec.end() - 1);
  // erase first z_lvl if first index references first node
  if (!z_lvl_vec.empty() && z_lvl_vec.front().index != 0)
    z_lvl_vec.erase(z_lvl_vec.begin());
  // erase last z_lvl if last index references last node
  OGRLineString *ogr_ls = static_cast<OGRLineString *>(feat->GetGeometryRef());
  if (!z_lvl_vec.empty() &&
      z_lvl_vec.back().index != ogr_ls->getNumPoints() - 1)
    z_lvl_vec.erase(z_lvl_vec.end());
}

static std::set<short> z_lvl_set = {-4, -3, -2, -1, 0, 1, 2, 3, 4, 5};
void StreetConverter::test__z_lvl_range(short z_lvl) {
  if (z_lvl_set.find(z_lvl) == z_lvl_set.end())
    throw(out_of_range_exception("z_lvl " + std::to_string(z_lvl) +
                                 " is not valid"));
}

bool StreetConverter::is_ferry(const char *value) {
  if (!strcmp(value, "H"))
    return false; // H --> not a ferry
  else if (!strcmp(value, "B"))
    return true; // T --> boat ferry
  else if (!strcmp(value, "R"))
    return true; // B --> rail ferry
  throw(format_error("value '" + std::string(value) + "' for FERRY not valid"));
}

/**
 * \brief checks if first z_level is more significant than the other or equal.
 * \param superior first z_level.
 * \param than second z_level.
 * \return true if superior is greater or equal than.
 */
bool StreetConverter::is_superior_or_equal(short superior, short than) {
  if (abs(superior) >= abs(than))
    return true;
  return false;
}

/* helpers for split_way_by_z_level */
/**
 * \brief checks if first z_level is more significant than the other.
 * \param superior First z_level.
 * \param than second z_level.
 * \return true if superior is superior to than.
 */
bool StreetConverter::is_superior(short superior, short than) {
  if (abs(superior) > abs(than))
    return true;
  return false;
}

void StreetConverter::update_region_connecting_points(
    std::map<osmium::Location, osmium::unsigned_object_id_type>
        &regionConnectingPoints,
    const StreetConverter::TagData &data, const std::filesystem::path &dir,
    osmium::io::Writer &writer) {
  std::map<uint64_t, std::vector<z_lvl_index_type_t>> z_level_map;

  const std::filesystem::path ZLEVELS_SHP = "Zlevels.shp";

  auto ds = openDataSource(dir / ZLEVELS_SHP);
  if (!ds)
    throw(shp_error(dir / ZLEVELS_SHP));

  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(dir / ZLEVELS_SHP));

  osmium::memory::Buffer node_buffer(BUFFER_SIZE);

  // lock the region connecting points
  {
    std::lock_guard<std::mutex> lock(g_regionConnectingPoints_mutex);

    for (auto &feat : *layer) {

      bool aligned = get_bool_from_feature(feat, ALIGNED);
      if (!aligned)
        continue;

      auto geom = feat->GetGeometryRef();
      auto geom_type = geom->getGeometryType();

      if (geom_type != wkbPoint) {
        throw(std::runtime_error("Region connecting points with geometry=" +
                                 std::string(geom->getGeometryName()) +
                                 " is not yet supported."));
      }

      auto point = static_cast<OGRPoint *>(geom);

      // process the region connecting points
      process_way_end_node(osmium::Location(point->getX(), point->getY()), data,
                           regionConnectingPoints, true, node_buffer);
    }
  }

  node_buffer.commit();
  {
    std::lock_guard<std::mutex> lock(osmiumWriterMutex);
    writer(std::move(node_buffer));
  }
}
