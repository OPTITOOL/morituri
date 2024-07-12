
#include "Converter.hpp"

#include <ogrsf_frmts.h>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/osm/object.hpp>
#include <osmium/osm/types.hpp>

#include "../../comm2osm_exceptions.hpp"

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
