
#include "Converter.hpp"

#include <boost/log/trivial.hpp>
#include <fstream>
#include <ogrsf_frmts.h>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/osm/object.hpp>
#include <osmium/osm/types.hpp>

#include "../../comm2osm_exceptions.hpp"
#include "../../util.hpp"

osmium::unsigned_object_id_type Converter::g_osm_id = 1;

/**
 * \brief Dummy attributes enable josm to read output xml files.
 *
 * \param obj OSMObject to set attributess to.
 * */
void Converter::set_dummy_osm_object_attributes(osmium::OSMObject &obj) {
  obj.set_version(VERSION.data());
  obj.set_changeset(CHANGESET.data());
  obj.set_uid(USERID.data());
  obj.set_timestamp(TIMESTAMP);
}

/**
 * \brief Adds relation members to relation.
 *
 * \param builder RelationBuilder to add members to.
 * \param ext_osm_way_ids List of external way ids.
 * \param int_osm_way_ids List of internal way ids.
 * */
void Converter::build_relation_members(
    osmium::builder::RelationBuilder &builder,
    const std::vector<osmium::unsigned_object_id_type> &ext_osm_way_ids,
    const std::vector<osmium::unsigned_object_id_type> &int_osm_way_ids) {
  osmium::builder::RelationMemberListBuilder rml_builder(builder);

  for (auto osm_id : ext_osm_way_ids)
    rml_builder.add_member(osmium::item_type::way, osm_id, "outer");

  for (auto osm_id : int_osm_way_ids)
    rml_builder.add_member(osmium::item_type::way, osm_id, "inner");
}

/**
 * \brief Creates a polygon from OGRPolygon.
 *
 * \param poly OGRPolygon to create polygon from.
 * \param ext_ring_osm_ids List of osm ids for external ring.
 * \param int_ring_osm_ids List of osm ids for internal ring.
 * \param node_buffer Buffer to store nodes.
 * \param way_buffer Buffer to store ways.
 * */
void Converter::create_multi_polygon(
    OGRMultiPolygon *mp,
    std::vector<osmium::unsigned_object_id_type> &mp_ext_ring_osm_ids,
    std::vector<osmium::unsigned_object_id_type> &mp_int_ring_osm_ids,
    std::map<osmium::Location, osmium::unsigned_object_id_type>
        &g_way_end_points_map,
    osmium::memory::Buffer &node_buffer, osmium::memory::Buffer &way_buffer) {

  for (const OGRPolygon *poly : mp) {
    create_polygon(poly, mp_ext_ring_osm_ids, mp_int_ring_osm_ids,
                   g_way_end_points_map, node_buffer, way_buffer);
  }
}

/**
 * \brief Creates a polygon from OGRPolygon.
 *
 * \param poly OGRPolygon to create polygon from.
 * \param exterior_way_ids List of osm ids for exterior ring.
 * \param interior_way_ids List of osm ids for interior ring.
 * \param node_buffer Buffer to store nodes.
 * \param way_buffer Buffer to store ways.
 * */
void Converter::create_polygon(
    const OGRPolygon *poly,
    std::vector<osmium::unsigned_object_id_type> &exterior_way_ids,
    std::vector<osmium::unsigned_object_id_type> &interior_way_ids,
    std::map<osmium::Location, osmium::unsigned_object_id_type>
        &g_way_end_points_map,
    osmium::memory::Buffer &node_buffer, osmium::memory::Buffer &way_buffer) {
  bool isExteriorRing = true; // first ring is the exterior ring
  for (const auto ring : *poly) {
    auto tmp =
        build_closed_ways(ring, g_way_end_points_map, node_buffer, way_buffer);
    if (isExteriorRing) {
      std::move(tmp.begin(), tmp.end(), std::back_inserter(exterior_way_ids));
    } else {
      std::move(tmp.begin(), tmp.end(), std::back_inserter(interior_way_ids));
    }
    isExteriorRing = false;
  }
}

/**
 * \brief Creates a closed way from OGRLineString.
 *
 * \param line OGRLineString to create way from.
 * \param node_buffer Buffer to store nodes.
 * \param way_buffer Buffer to store ways.
 * */
std::vector<std::pair<osmium::Location, osmium::unsigned_object_id_type>>
Converter::create_open_way_nodes(
    const OGRLineString *line,
    std::map<osmium::Location, osmium::unsigned_object_id_type>
        &g_way_end_points_map,
    osmium::memory::Buffer &node_buffer) {
  std::vector<std::pair<osmium::Location, osmium::unsigned_object_id_type>>
      osm_way_node_ids;

  for (auto &point : *line) {
    osmium::Location location(point.getX(), point.getY());
    auto it = g_way_end_points_map.find(location);
    if (it != g_way_end_points_map.end()) {
      osm_way_node_ids.emplace_back(location, it->second);
    } else {
      auto osm_id = build_node(location, node_buffer);
      osm_way_node_ids.emplace_back(location, osm_id);
      g_way_end_points_map.emplace(location, osm_id);
    }
  }

  return osm_way_node_ids;
}

/**
 * \brief Creates a closed way from OGRLinearRing.
 *
 * \param ring OGRLinearRing to create way from.
 * \param node_buffer Buffer to store nodes.
 * \param way_buffer Buffer to store ways.
 * */
std::vector<osmium::unsigned_object_id_type> Converter::build_closed_ways(
    const OGRLinearRing *ring,
    std::map<osmium::Location, osmium::unsigned_object_id_type>
        &g_way_end_points_map,
    osmium::memory::Buffer &node_buffer, osmium::memory::Buffer &way_buffer) {
  auto osm_way_node_ids =
      create_closed_way_nodes(ring, g_way_end_points_map, node_buffer);

  std::vector<osmium::unsigned_object_id_type> osm_way_ids;
  size_t i = 0;
  do {
    osmium::builder::WayBuilder builder(way_buffer);
    setObjectProperties(builder);

    osmium::builder::WayNodeListBuilder wnl_builder(way_buffer, &builder);
    for (size_t j = i;
         j < std::min(i + OSM_MAX_WAY_NODES, osm_way_node_ids.size()); j++) {
      const auto [location, osm_id] = osm_way_node_ids.at(j);
      wnl_builder.add_node_ref(osm_id, location);
    }
    osm_way_ids.push_back(builder.object().id());
    i += OSM_MAX_WAY_NODES - 1;
  } while (i < osm_way_node_ids.size());
  return osm_way_ids;
}

/**
 * \brief Creates a closed way from OGRLinearRing.
 *
 * \param ring OGRLinearRing to create way from.
 * \param node_buffer Buffer to store nodes.
 * */
std::vector<std::pair<osmium::Location, osmium::unsigned_object_id_type>>
Converter::create_closed_way_nodes(
    const OGRLinearRing *ring,
    std::map<osmium::Location, osmium::unsigned_object_id_type>
        &g_way_end_points_map,
    osmium::memory::Buffer &node_buffer) {
  std::vector<std::pair<osmium::Location, osmium::unsigned_object_id_type>>
      osm_way_node_ids;
  for (auto &point : *ring) {
    osmium::Location location(point.getX(), point.getY());
    auto it = g_way_end_points_map.find(location);

    if (it != g_way_end_points_map.end()) {
      osm_way_node_ids.emplace_back(location, it->second);
    } else {
      auto osm_id = build_node(location, node_buffer);
      osm_way_node_ids.emplace_back(location, osm_id);
      g_way_end_points_map.emplace(location, osm_id);
    }
  }

  // first and last node are the same in rings, hence add first node_id and
  // skip last node.
  if (!ring->get_IsClosed())
    throw format_error(
        "admin boundary ring is invalid. First and last node don't match");

  return osm_way_node_ids;
}

/**
 * \brief Creates a node from location.
 *
 * \param location Location to create node from.
 * \param node_buffer Buffer to store nodes.
 * */
osmium::unsigned_object_id_type
Converter::build_node(const osmium::Location &location,
                      osmium::memory::Buffer &node_buffer) {
  osmium::builder::NodeBuilder builder(node_buffer);
  return build_node(location, builder);
}

/**
 * \brief Creates a node from location.
 *
 * \param location Location to create node from.
 * \param builder NodeBuilder to create node from.
 * */
osmium::unsigned_object_id_type
Converter::build_node(const osmium::Location &location,
                      osmium::builder::NodeBuilder &builder) {
  setObjectProperties(builder);
  builder.object().set_location(location);
  return builder.object().id();
}

/**
 * \brief Sets object properties.
 *
 * \param builder Builder to set properties to.
 * */
template <typename T> void Converter::setObjectProperties(T &builder) {
  builder.object().set_id(g_osm_id++);
  set_dummy_osm_object_attributes(builder.object());
  builder.set_user(USER.data());
}

std::string Converter::navteq_2_osm_admin_lvl(uint navteq_admin_lvl_int) {
  if (NAVTEQ_ADMIN_LVL_MIN > navteq_admin_lvl_int ||
      navteq_admin_lvl_int > NAVTEQ_ADMIN_LVL_MAX)
    throw std::runtime_error("invalid admin level. admin level '" +
                             std::to_string(navteq_admin_lvl_int) +
                             "' is out of range.");

  return std::to_string(2 * navteq_admin_lvl_int);
}

std::string Converter::navteq_2_osm_admin_lvl(std::string navteq_admin_lvl) {
  if (string_is_not_unsigned_integer(navteq_admin_lvl))
    throw std::runtime_error("admin level contains invalid character");

  return navteq_2_osm_admin_lvl(stoi(navteq_admin_lvl));
}

// matching from http://www.loc.gov/standards/iso639-2/php/code_list.php
// http://www.loc.gov/standards/iso639-2/ISO-639-2_utf-8.txt
// ISO-639 conversion

void Converter::parse_lang_code_file() {
  const std::filesystem::path PLUGINS_NAVTEQ_ISO_639_2_UTF_8_TXT(
      "plugins/navteq/ISO-639-2_utf-8.txt");

  if (executable_path.empty())
    throw(std::runtime_error("executable_path is empty"));

  std::filesystem::path iso_file(executable_path /
                                 PLUGINS_NAVTEQ_ISO_639_2_UTF_8_TXT);
  std::ifstream file(iso_file.string());
  assert(file.is_open());
  std::string line;
  std::string delim = "|";
  if (file.is_open()) {
    while (getline(file, line, '\n')) {
      std::vector<std::string> lv;
      auto start = 0u;
      auto end = line.find(delim);
      while (end != std::string::npos) {
        lv.push_back(line.substr(start, end - start));
        start = end + delim.length();
        end = line.find(delim, start);
      }
      std::string iso_639_2 = lv.at(0);
      std::string iso_639_1 = lv.at(2);
      lang_code_map.emplace(std::make_pair(iso_639_2, iso_639_1));
    }
    file.close();
  }
}

std::string Converter::parse_lang_code(std::string lang_code) {
  std::ranges::transform(lang_code, lang_code.begin(), ::tolower);
  if (lang_code_map.empty())
    parse_lang_code_file();

  auto lc = lang_code_map.find(lang_code);
  if (lc != lang_code_map.end()) {
    if (!lc->second.empty())
      return lc->second;
    else
      return lang_code; // fallback
  }
  BOOST_LOG_TRIVIAL(error) << lang_code << " not found!\n";
  return lang_code; // fallback
}

std::map<osmium::unsigned_object_id_type, Converter::mtd_area_dataset>
Converter::process_meta_areas(std::filesystem::path dir) {
  const std::filesystem::path MTD_AREA_DBF = "MtdArea.dbf";

  std::map<osmium::unsigned_object_id_type, mtd_area_dataset> mtd_area_map;

  auto layer = openDataSource(dir / MTD_AREA_DBF).or_else([]() -> OGRLayer * {
    throw(std::runtime_error("could not open MtdArea.dbf"));
  });

  for (auto &feat : *layer) {
    osmium::unsigned_object_id_type area_id =
        get_uint_from_feature(feat, AREA_ID);

    // find or create a new area data set
    mtd_area_dataset &data = mtd_area_map[area_id];

    data.area_id = area_id;

    std::string admin_lvl =
        std::to_string(get_uint_from_feature(feat, ADMIN_LVL));
    if (data.admin_lvl.empty()) {
      data.admin_lvl = admin_lvl;
    } else if (data.admin_lvl != admin_lvl) {
      BOOST_LOG_TRIVIAL(error)
          << "entry with area_id=" << area_id
          << " has multiple admin_lvls:" << data.admin_lvl << ", " << admin_lvl;
    }

    std::string lang_code = get_field_from_feature(feat, LANG_CODE);
    std::string area_name = get_field_from_feature(feat, AREA_NAME);

    std::string area_type = get_field_from_feature(feat, "AREA_TYPE");
    if (area_type == "B") {
      data.name = to_camel_case_with_spaces(area_name);
      data.lang_code_2_area_name.emplace_back(
          lang_code, to_camel_case_with_spaces(area_name));
      data.area_code_1 = get_uint_from_feature(feat, AREA_CODE_1);
    } else if (area_type == "A") {
      data.short_name = to_camel_case_with_spaces(area_name);
    } else {
      data.lang_code_2_area_name.emplace_back(
          lang_code, to_camel_case_with_spaces(area_name));
      data.area_code_1 = get_uint_from_feature(feat, AREA_CODE_1);
    }
  }
  return mtd_area_map;
}

uint Converter::get_area_code_l(
    uint64_t l_area_id, uint64_t r_area_id,
    const std::map<osmium::unsigned_object_id_type, mtd_area_dataset>
        &mtd_area_map) {

  auto l_area = mtd_area_map.find(l_area_id);
  if (l_area != mtd_area_map.end())
    return l_area->second.area_code_1;

  auto r_area = mtd_area_map.find(r_area_id);
  if (r_area != mtd_area_map.end())
    return r_area->second.area_code_1;

  throw(out_of_range_exception("could not find area_id " +
                               std::to_string(mtd_area_map.size())));
}

uint Converter::get_area_code_l(
    const OGRFeatureUniquePtr &f,
    const std::map<osmium::unsigned_object_id_type, mtd_area_dataset>
        &mtd_area_map) {
  uint64_t l_area_id = get_uint_from_feature(f, L_AREA_ID);
  uint64_t r_area_id = get_uint_from_feature(f, R_AREA_ID);

  return get_area_code_l(l_area_id, r_area_id, mtd_area_map);
}

std::optional<OGRLayer *>
openDataSource(const std::filesystem::path &shape_file) {
  auto ds = GDALDatasetUniquePtr(GDALDataset::Open(shape_file.c_str()));
  if (!ds) {
    BOOST_LOG_TRIVIAL(debug) << "No shp found in " << shape_file;
    return std::nullopt;
  }
  auto layer = ds->GetLayer(0);
  if (!layer)
    throw(shp_empty_error(shape_file.string()));

  return std::optional(layer);
}
