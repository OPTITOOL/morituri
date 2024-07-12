/* \include
 * navteq.hpp
 *
 *  Created on: 12.06.2015
 *      Author: philip
 *
 * Convert Shapefiles into OSM files.
 */

#ifndef NAVTEQ_HPP_
#define NAVTEQ_HPP_

#include <iostream>

#include <ogrsf_frmts.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/timer/progress_display.hpp>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/index/map/sparse_file_array.hpp>
#include <osmium/osm/item_type.hpp>
#include <osmium/osm/object.hpp>
#include <osmium/osm/types.hpp>

#include "../comm2osm_exceptions.hpp"
#include "../readers.hpp"
#include "navteq2osm_tag_parser.hpp"
#include "navteq_mappings.hpp"
#include "navteq_types.hpp"
#include "navteq_util.hpp"

#define DEBUG false

static constexpr int buffer_size = 10 * 1000 * 1000;

// maps location of way end nodes to node ids
node_map_type g_way_end_points_map;

z_lvl_nodes_map_type g_z_lvl_nodes_map;

// id counter for object creation
osmium::unsigned_object_id_type g_osm_id = 1;

// g_route_type_map maps navteq link_ids to the lowest occurring route_type
// value
link_id_route_type_map g_route_type_map;

// g_hwys_ref_map maps navteq link_ids to a vector of highway names
link_id_to_names_map g_hwys_ref_map;

// g_hwys_ref_map maps navteq link_ids to a vector of highway names
std::set<link_id_type> g_construction_set;

// g_ramps_ref_map maps navteq link_ids to a vector of ramp names
std::map<osmium::Location, std::map<uint, std::string>> g_ramps_ref_map;

// auxiliary map which maps datasets with tags for administrative boundaries
mtd_area_map_type g_mtd_area_map;

// map for conditional modifications
cnd_mod_map_type g_cnd_mod_map;

// map for conditional driving manoeuvres
cdms_map_type g_cdms_map;
std::map<area_id_type, govt_code_type> g_area_to_govt_code_map;
cntry_ref_map_type g_cntry_ref_map;

bool debugMode = false;

/**
 * \brief Dummy attributes enable josm to read output xml files.
 *
 * \param obj OSMObject to set attributess to.
 * */
void set_dummy_osm_object_attributes(osmium::OSMObject &obj) {
  obj.set_version(VERSION);
  obj.set_changeset(CHANGESET);
  obj.set_uid(USERID);
  obj.set_timestamp(TIMESTAMP);
}

/**
 * \brief parses and writes tags from features with builder
 * \param builder builder to add tags to.
 * \param z_level Sets z_level if valid (-5 is invalid default)
 * \return link id of parsed feature.
 */

link_id_type build_tag_list(OGRFeatureUniquePtr &feat,
                            osmium::builder::Builder *builder,
                            osmium::memory::Buffer &buf, short z_level) {
  osmium::builder::TagListBuilder tl_builder(buf, builder);

  link_id_type link_id = parse_street_tags(
      &tl_builder, feat, &g_cdms_map, &g_cnd_mod_map, &g_area_to_govt_code_map,
      &g_cntry_ref_map, &g_mtd_area_map, &g_route_type_map, &g_hwys_ref_map,
      g_construction_set, debugMode);

  if (z_level != -5 && z_level != 0)
    tl_builder.add_tag("layer", std::to_string(z_level));
  if (link_id == 0)
    throw(format_error("layers column field '" + std::string(LINK_ID) +
                       "' is missing"));
  return link_id;
}

/**
 * \brief creates Node with given builder.
 * \param location Location of Node being created.
 * \param builder NodeBuilder to create Node.
 * \return id of created Node.
 */
osmium::unsigned_object_id_type
build_node(const osmium::Location &location,
           osmium::builder::NodeBuilder *builder) {
  builder->object().set_id(g_osm_id++);
  set_dummy_osm_object_attributes(builder->object());
  builder->set_user(USER);
  builder->object().set_location(location);
  return builder->object().id();
}

/**
 * \brief creates Node and writes it to m_buffer.
 * \param location Location of Node being created.
 * \return id of created Node.
 */
osmium::unsigned_object_id_type
build_node(const osmium::Location &location,
           osmium::memory::Buffer &node_buffer) {
  osmium::builder::NodeBuilder builder(node_buffer);
  return build_node(location, &builder);
}

/**
 * \brief adds WayNode to Way.
 * \param location Location of WayNode
 * \param wnl_builder Builder to create WayNode
 * \param node_ref_map holds Node ids suitable to given Location.
 */
void add_way_node(const osmium::Location &location,
                  osmium::builder::WayNodeListBuilder &wnl_builder,
                  node_map_type *node_ref_map) {
  assert(node_ref_map);
  wnl_builder.add_node_ref(
      osmium::NodeRef(node_ref_map->at(location), location));
}

static std::set<short> z_lvl_set = {-4, -3, -2, -1, 0, 1, 2, 3, 4, 5};
void test__z_lvl_range(short z_lvl) {
  if (z_lvl_set.find(z_lvl) == z_lvl_set.end())
    throw(out_of_range_exception("z_lvl " + std::to_string(z_lvl) +
                                 " is not valid"));
}

/**
 * \brief creates way with tags in m_buffer.
 * \param ogr_ls provides geometry (linestring) for the way.
 * \param node_ref_map provides osm_ids of Nodes to a given location.
 * \param is_sub_linestring true if given linestring is a sublinestring.
 * \param z_lvl z-level of way. initially invalid (-5).
 * \return id of created Way.
 */
osmium::unsigned_object_id_type
build_way(OGRFeatureUniquePtr &feat, OGRLineString *ogr_ls,
          node_map_type *node_ref_map, osmium::memory::Buffer &way_buffer,
          bool is_sub_linestring = false, short z_lvl = -5) {

  if (is_sub_linestring)
    test__z_lvl_range(z_lvl);

  osmium::builder::WayBuilder builder(way_buffer);
  builder.object().set_id(g_osm_id++);
  set_dummy_osm_object_attributes(builder.object());

  osmium::Location start(ogr_ls->getX(0), ogr_ls->getY(0));
  osmium::Location end(ogr_ls->getX(ogr_ls->getNumPoints() - 1),
                       ogr_ls->getY(ogr_ls->getNumPoints() - 1));

  builder.set_user(USER);
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
          map_containing_node = node_ref_map;
      } else {
        if (node_ref_map->find(location) != node_ref_map->end()) {
          map_containing_node = node_ref_map;
        } else {
          // node has to be in node_ref_map or way_end_points_map
          assert(g_way_end_points_map.find(location) !=
                 g_way_end_points_map.end());
          map_containing_node = &g_way_end_points_map;
        }
      }
      add_way_node(location, wnl_builder, map_containing_node);
    }
  }

  build_tag_list(feat, &builder, way_buffer, z_lvl);
  return builder.object().id();
}

/**
 * \brief  creates a sublinestring from ogr_ls from range [start_index,
 * end_index] inclusive \param ogr_ls Linestring from which the sublinestring
 * is taken from. \param start_index Node index in ogr_ls, where sublinestring
 * will begin. \param end_index Node index in ogr_ls, where sublinestring will
 * end. \return sublinestring from ogr_ls [start_index, end_index] inclusive
 */
OGRLineString create_sublinestring_geometry(OGRLineString *ogr_ls,
                                            int start_index,
                                            int end_index = -1) {
  assert(start_index < end_index || end_index == -1);
  assert(start_index < ogr_ls->getNumPoints());
  OGRLineString ogr_sub_ls;
  ogr_sub_ls.addSubLineString(ogr_ls, start_index, end_index);
  return ogr_sub_ls;
}

/* helpers for split_way_by_z_level */
/**
 * \brief checks if first z_level is more significant than the other.
 * \param superior First z_level.
 * \param than second z_level.
 * \return true if superior is superior to than.
 */
bool is_superior(short superior, short than) {
  if (abs(superior) > abs(than))
    return true;
  return false;
}

/**
 * \brief checks if first z_level is more significant than the other or equal.
 * \param superior first z_level.
 * \param than second z_level.
 * \return true if superior is greater or equal than.
 */
bool is_superior_or_equal(short superior, short than) {
  if (abs(superior) >= abs(than))
    return true;
  return false;
}

/**
 * \brief splits a OGRLinestring by index.
 * \param start_index index where sub_way begins.
 * \param end_index index where sub_way ends.
 * \param ogr_ls geometry of index.
 * \param node_ref_map provides osm_ids of Nodes to a given location.
 * \param z_lvl
 */
void build_sub_way_by_index(OGRFeatureUniquePtr &feat, OGRLineString *ogr_ls,
                            ushort start_index, ushort end_index,
                            node_map_type *node_ref_map,
                            osmium::memory::Buffer &way_buffer,
                            short z_lvl = 0) {
  OGRLineString subLineString =
      create_sublinestring_geometry(ogr_ls, start_index, end_index);
  build_way(feat, &subLineString, node_ref_map, way_buffer, true, z_lvl);
}

/**
 * \brief creates a way for each z_level change
 * \param start_index starting index of next way creation
 * \param end_index index where the last z_level is not 0 (not default)
 * \param last_index index of the last node in given way
 * \param link_id for debug only
 * \param node_z_level_vector holds [index, z_level] pairs to process
 * \param ogr_ls given way which has to be splitted
 * \param node_ref_map location to osm_id mapping (to preserve uniqueness of
 * node locations) \return start_index
 */
ushort create_continuing_sub_ways(
    OGRFeatureUniquePtr &feat, OGRLineString *ogr_ls, ushort first_index,
    ushort start_index, ushort last_index, uint link_id,
    const index_z_lvl_vector_type &node_z_level_vector,
    node_map_type *node_ref_map, osmium::memory::Buffer &way_buffer) {

  for (auto it = node_z_level_vector.cbegin(); it != node_z_level_vector.cend();
       ++it) {
    short z_lvl = it->second;
    test__z_lvl_range(z_lvl);
    bool last_element = node_z_level_vector.cend() - 1 == it;
    bool not_last_element = !last_element;
    ushort index = it->first;
    ushort next_index;
    short next_z_lvl;
    if (not_last_element) {
      auto next_it = it + 1;
      next_index = next_it->first;
      next_z_lvl = next_it->second;
      test__z_lvl_range(next_z_lvl);
    }
    if (DEBUG)
      std::cout << "first_index=" << first_index << "   "
                << "start_index=" << start_index << "   "
                << "last_index=" << last_index << "   " << "index=" << index
                << "   " << "z_lvl=" << z_lvl << "   "
                << "next_z_lvl=" << next_z_lvl << std::endl;

    if (not_last_element) {
      if (index + 2 == next_index && z_lvl == next_z_lvl)
        continue;
      bool not_second_last_element = it + 2 != node_z_level_vector.cend();
      if (not_second_last_element) {
        ushort second_next_index = (it + 2)->first;
        short second_next_z_lvl = (it + 2)->second;
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
      if (DEBUG)
        std::cout << " 2 ## " << link_id << " ## " << from << "/" << last_index
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
        if (DEBUG)
          std::cout << " 3 ## " << link_id << " ## " << to << "/" << last_index
                    << "  -  " << next_index - 1 << "/" << last_index
                    << ": \tz_lvl=" << 0 << std::endl;
        start_index = next_index - 1;
      }
    }
  }
  return start_index;
}

/**
 * \brief splits a given linestring into sub_ways to be able to apply
 * z_levels. \param ogr_ls Linestring to be splitted. \param
 * node_z_level_vector holds pairs of Node indices in linestring and their
 * z_level. ommited indices imply default value of 0. \param node_ref_map
 * provides osm_ids of Nodes to a given location. \param link_id link_id of
 * processed feature - for debug only.
 */
void split_way_by_z_level(OGRFeatureUniquePtr &feat, OGRLineString *ogr_ls,
                          const index_z_lvl_vector_type &node_z_level_vector,
                          node_map_type *node_ref_map, uint link_id,
                          osmium::memory::Buffer &way_buffer) {

  ushort first_index = 0, last_index = ogr_ls->getNumPoints() - 1;
  ushort start_index = node_z_level_vector.cbegin()->first;
  if (start_index > 0)
    start_index--;

  // first_index <= start_index < end_index <= last_index
  assert(first_index <= start_index);
  assert(start_index < last_index);

  //	if (DEBUG) print_z_level_map(link_id, true);
  if (first_index != start_index) {
    build_sub_way_by_index(feat, ogr_ls, first_index, start_index, node_ref_map,
                           way_buffer);
    if (DEBUG)
      std::cout << " 1 ## " << link_id << " ## " << first_index << "/"
                << last_index << "  -  " << start_index << "/" << last_index
                << ": \tz_lvl=" << 0 << std::endl;
  }

  start_index = create_continuing_sub_ways(
      feat, ogr_ls, first_index, start_index, last_index, link_id,
      node_z_level_vector, node_ref_map, way_buffer);

  if (start_index < last_index) {
    build_sub_way_by_index(feat, ogr_ls, start_index, last_index, node_ref_map,
                           way_buffer);
    if (DEBUG)
      std::cout << " 4 ## " << link_id << " ## " << start_index << "/"
                << last_index << "  -  " << last_index << "/" << last_index
                << ": \tz_lvl=" << 0 << std::endl;
  }
}

/****************************************************
 * processing input: converting from navteq to osm
 * 					 and writing it to osmium
 ****************************************************/

/**
 * \brief determines osm_id for end_point. If it doesn't exist it will be
 * created.
 */

void process_end_point(bool first, z_lvl_type z_lvl, OGRLineString *ogr_ls,
                       node_map_type &node_ref_map,
                       osmium::memory::Buffer &node_buffer) {
  ushort i = first ? 0 : ogr_ls->getNumPoints() - 1;
  osmium::Location location(ogr_ls->getX(i), ogr_ls->getY(i));

  if (z_lvl != 0) {
    node_id_type node_id = std::make_pair(location, z_lvl);
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

void process_first_end_point(z_lvl_type z_lvl, OGRLineString *ogr_ls,
                             node_map_type &node_ref_map,
                             osmium::memory::Buffer &node_buffer) {
  process_end_point(true, z_lvl, ogr_ls, node_ref_map, node_buffer);
}

void process_last_end_point(z_lvl_type z_lvl, OGRLineString *ogr_ls,
                            node_map_type &node_ref_map,
                            osmium::memory::Buffer &node_buffer) {
  process_end_point(false, z_lvl, ogr_ls, node_ref_map, node_buffer);
}

void middle_points_preparation(OGRLineString *ogr_ls,
                               node_map_type &node_ref_map,
                               osmium::memory::Buffer &node_buffer) {
  // creates remaining nodes required for way
  for (int i = 1; i < ogr_ls->getNumPoints() - 1; ++i) {
    osmium::Location location(ogr_ls->getX(i), ogr_ls->getY(i));
    node_ref_map.emplace(location, build_node(location, node_buffer));
  }
  node_buffer.commit();
}

/**
 * \brief replaces all z-levels by zero, which are not an endpoint
 * \param z_lvl_vec vector containing pairs of [z_lvl_index, z_lvl]
 */
void set_ferry_z_lvls_to_zero(OGRFeatureUniquePtr &feat,
                              index_z_lvl_vector_type &z_lvl_vec) {
  // erase middle z_lvls
  if (z_lvl_vec.size() > 2)
    z_lvl_vec.erase(z_lvl_vec.begin() + 1, z_lvl_vec.end() - 1);
  // erase first z_lvl if first index references first node
  if (!z_lvl_vec.empty() && z_lvl_vec.begin()->first != 0)
    z_lvl_vec.erase(z_lvl_vec.begin());
  // erase last z_lvl if last index references last node
  OGRLineString *ogr_ls = static_cast<OGRLineString *>(feat->GetGeometryRef());
  if (!z_lvl_vec.empty() &&
      (z_lvl_vec.end() - 1)->first != ogr_ls->getNumPoints() - 1)
    z_lvl_vec.erase(z_lvl_vec.end());
}

/**
 * \brief creates interpolated house numbers alongside of a given linestring
 * if feature holds suitable tags \param feat feature which holds the tags
 * \param ogr_ls linestring which receives the interpolated house numbers
 * \param left specifies on which side of the linestring the house numbers
 * will be applied
 */
void create_house_numbers(const OGRFeatureUniquePtr &feat,
                          const OGRLineString *ogr_ls, bool left,
                          osmium::memory::Buffer &node_buffer,
                          osmium::memory::Buffer &way_buffer) {
  const char *ref_addr = left ? L_REFADDR : R_REFADDR;
  const char *nref_addr = left ? L_NREFADDR : R_NREFADDR;
  const char *addr_schema = left ? L_ADDRSCH : R_ADDRSCH;

  if (!strcmp(get_field_from_feature(feat, ref_addr), ""))
    return;
  if (!strcmp(get_field_from_feature(feat, nref_addr), ""))
    return;
  if (!strcmp(get_field_from_feature(feat, addr_schema), ""))
    return;
  if (!strcmp(get_field_from_feature(feat, addr_schema), "M"))
    return;

  std::string startNumber =
      get_field_from_feature(feat, left ? ref_addr : nref_addr);

  std::string endNumber =
      get_field_from_feature(feat, left ? nref_addr : ref_addr);

  std::unique_ptr<OGRLineString> offset_ogr_ls(
      create_offset_curve(ogr_ls, HOUSENUMBER_CURVE_OFFSET, left));
  if (startNumber == endNumber) {
    // no interpolation for signel addresses
    OGRPoint midPoint;
    offset_ogr_ls->Centroid(&midPoint);
    {
      osmium::Location location(midPoint.getX(), midPoint.getY());
      // scope node_builder
      osmium::builder::NodeBuilder node_builder(node_buffer);
      build_node(location, &node_builder);
      {
        // scope tl_builder
        osmium::builder::TagListBuilder tl_builder(node_builder);
        tl_builder.add_tag("addr:housenumber", startNumber);
        tl_builder.add_tag(
            "addr:street",
            to_camel_case_with_spaces(get_field_from_feature(feat, ST_NAME)));
      }
    }
  } else {
    // osm address interpolation
    osmium::builder::WayBuilder way_builder(way_buffer);
    way_builder.object().set_id(g_osm_id++);
    set_dummy_osm_object_attributes(way_builder.object());
    way_builder.set_user(USER);
    {
      // scope wnl_builder
      osmium::builder::WayNodeListBuilder wnl_builder(way_buffer, &way_builder);

      for (int i = 0; i < offset_ogr_ls->getNumPoints(); i++) {
        osmium::Location location(offset_ogr_ls->getX(i),
                                  offset_ogr_ls->getY(i));
        {
          // scope node_builder
          osmium::builder::NodeBuilder node_builder(node_buffer);
          auto node_id = build_node(location, &node_builder);
          {
            // scope tl_builder
            osmium::builder::TagListBuilder tl_builder(node_buffer,
                                                       &node_builder);
            if (i == 0 || i == offset_ogr_ls->getNumPoints() - 1) {
              if (i == 0) {
                tl_builder.add_tag("addr:housenumber", startNumber);
              } else if (i == offset_ogr_ls->getNumPoints() - 1) {
                tl_builder.add_tag("addr:housenumber", endNumber);
              }
              tl_builder.add_tag("addr:street",
                                 to_camel_case_with_spaces(
                                     get_field_from_feature(feat, ST_NAME)));
            }
          }

          wnl_builder.add_node_ref(osmium::NodeRef(node_id, location));
        }
      }
    }
    {
      // scope tl_builder
      osmium::builder::TagListBuilder tl_builder(way_buffer, &way_builder);
      const char *schema =
          parse_house_number_schema(get_field_from_feature(feat, addr_schema));
      tl_builder.add_tag("addr:interpolation", schema);
    }
  }
  node_buffer.commit();
  way_buffer.commit();
}

void create_house_numbers(const OGRFeatureUniquePtr &feat,
                          const OGRLineString *ogr_ls,
                          osmium::memory::Buffer &node_buffer,
                          osmium::memory::Buffer &way_buffer) {
  create_house_numbers(feat, ogr_ls, true, node_buffer, way_buffer);
  create_house_numbers(feat, ogr_ls, false, node_buffer, way_buffer);
}

void create_premium_house_numbers(
    const OGRFeatureUniquePtr &feat,
    const std::vector<std::pair<osmium::Location, std::string>> &addressList,
    int linkId, osmium::memory::Buffer &node_buffer) {

  for (auto &address : addressList) {

    // scope node_builder
    osmium::builder::NodeBuilder node_builder(node_buffer);
    build_node(address.first, &node_builder);
    {
      // scope tl_builder
      osmium::builder::TagListBuilder tl_builder(node_buffer, &node_builder);
      tl_builder.add_tag(LINK_ID, std::to_string(linkId));
      tl_builder.add_tag("addr:housenumber", address.second);
      tl_builder.add_tag(
          "addr:street",
          to_camel_case_with_spaces(get_field_from_feature(feat, ST_NAME)));
    }
  }
}

/**
 * \brief creates Way from linestring.
 * 		  creates missing Nodes needed for Way and Way itself.
 * \param ogr_ls linestring which provides the geometry.
 * \param z_level_map holds z_levels to Nodes of Ways.
 */
void process_way(OGRFeatureUniquePtr &feat, z_lvl_map *z_level_map,
                 osmium::memory::Buffer &node_buffer,
                 osmium::memory::Buffer &way_buffer) {

  node_map_type node_ref_map;

  auto ogr_ls = static_cast<OGRLineString *>(feat->GetGeometryRef());

  // creates remaining nodes required for way
  middle_points_preparation(ogr_ls, node_ref_map, node_buffer);
  if (ogr_ls->getNumPoints() > 2)
    assert(node_ref_map.size() > 0);

  link_id_type link_id = get_uint_from_feature(feat, LINK_ID);

  auto it = z_level_map->find(link_id);
  if (it == z_level_map->end()) {
    build_way(feat, ogr_ls, &node_ref_map, way_buffer);
  } else {
    auto &index_z_lvl_vector = it->second;

    // way with different z_levels
    auto &first_point_with_different_z_lvl = index_z_lvl_vector.front();
    auto first_index = 0;
    z_lvl_type first_z_lvl = 0;
    if (first_point_with_different_z_lvl.first == first_index)
      first_z_lvl = first_point_with_different_z_lvl.second;

    process_first_end_point(first_z_lvl, ogr_ls, node_ref_map, node_buffer);

    auto &last_point_with_different_z_lvl = index_z_lvl_vector.back();
    auto last_index = ogr_ls->getNumPoints() - 1;
    z_lvl_type last_z_lvl = 0;
    if (last_point_with_different_z_lvl.first == last_index)
      last_z_lvl = last_point_with_different_z_lvl.second;

    process_last_end_point(last_z_lvl, ogr_ls, node_ref_map, node_buffer);

    way_buffer.commit();

    if (is_ferry(get_field_from_feature(feat, FERRY)))
      set_ferry_z_lvls_to_zero(feat, index_z_lvl_vector);

    split_way_by_z_level(feat, ogr_ls, index_z_lvl_vector, &node_ref_map,
                         link_id, way_buffer);
  }
}

/**
 * \brief creates Way from linestring.
 * 		  creates missing Nodes needed for Way and Way itself.
 * \param ogr_ls linestring which provides the geometry.
 * \param z_level_map holds z_levels to Nodes of Ways.
 */
void process_house_numbers(
    const OGRFeatureUniquePtr &feat,
    std::map<uint64_t, std::vector<std::pair<osmium::Location, std::string>>>
        *pointAddresses,
    int linkId, osmium::memory::Buffer &node_buffer,
    osmium::memory::Buffer &way_buffer) {

  auto ogr_ls = static_cast<const OGRLineString *>(feat->GetGeometryRef());

  auto it = pointAddresses->find(linkId);
  if (it != pointAddresses->end()) {
    create_premium_house_numbers(feat, it->second, linkId, node_buffer);
  } else {
    if (!strcmp(get_field_from_feature(feat, ADDR_TYPE), "B")) {
      create_house_numbers(feat, ogr_ls, node_buffer, way_buffer);
    }
  }
}

// \brief writes way end node to way_end_points_map.
void process_way_end_node(const osmium::Location &location,
                          osmium::memory::Buffer &node_buffer) {

  auto it = g_way_end_points_map.find(location);
  if (it != g_way_end_points_map.end())
    return;

  osmium::builder::NodeBuilder builder(node_buffer);
  osmium::unsigned_object_id_type osm_id = build_node(location, &builder);
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

// \brief gets end nodes of linestring and processes them.
void process_way_end_nodes(OGRFeatureUniquePtr &feat,
                           osmium::memory::Buffer &node_buffer) {

  auto ogr_ls = static_cast<const OGRLineString *>(feat->GetGeometryRef());

  process_way_end_node(osmium::Location(ogr_ls->getX(0), ogr_ls->getY(0)),
                       node_buffer);
  process_way_end_node(
      osmium::Location(ogr_ls->getX(ogr_ls->getNumPoints() - 1),
                       ogr_ls->getY(ogr_ls->getNumPoints() - 1)),
      node_buffer);
}

/**
 * \brief creates nodes for closed ways (i.e. for administrative boundary)
 * \return osm_ids of created nodes
 */
node_vector_type create_closed_way_nodes(const OGRLinearRing *ring,
                                         osmium::memory::Buffer &node_buffer) {
  node_vector_type osm_way_node_ids;
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
 * \brief creates nodes for open ways
 * \return osm_ids of created nodes
 */
node_vector_type create_open_way_nodes(const OGRLineString *line,
                                       osmium::memory::Buffer &node_buffer) {
  node_vector_type osm_way_node_ids;

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
 * \brief creates closed ways (i.e. for administrative boundary)
 * \return osm_ids of created ways
 */
osm_id_vector_type build_closed_ways(const OGRLinearRing *ring,
                                     osmium::memory::Buffer &node_buffer,
                                     osmium::memory::Buffer &way_buffer) {
  node_vector_type osm_way_node_ids =
      create_closed_way_nodes(ring, node_buffer);

  osm_id_vector_type osm_way_ids;
  size_t i = 0;
  do {
    osmium::builder::WayBuilder builder(way_buffer);
    builder.object().set_id(g_osm_id++);
    set_dummy_osm_object_attributes(builder.object());
    builder.set_user(USER);
    osmium::builder::WayNodeListBuilder wnl_builder(way_buffer, &builder);
    for (size_t j = i;
         j < std::min(i + OSM_MAX_WAY_NODES, osm_way_node_ids.size()); j++)
      wnl_builder.add_node_ref(osm_way_node_ids.at(j).second,
                               osm_way_node_ids.at(j).first);
    osm_way_ids.push_back(builder.object().id());
    i += OSM_MAX_WAY_NODES - 1;
  } while (i < osm_way_node_ids.size());
  return osm_way_ids;
}

/**
 * \brief adds navteq administrative boundary tags to Relation
 */
void build_admin_boundary_taglist(osmium::builder::Builder &builder,
                                  osmium::unsigned_object_id_type area_id,
                                  uint level) {
  osmium::builder::TagListBuilder tl_builder(builder);
  // Mind tl_builder scope in calling method!
  if (level == 5) {
    // only landuse residential
    tl_builder.add_tag("type", "multipolygon");
    tl_builder.add_tag("landuse", "residential");
  } else {
    tl_builder.add_tag("type", "boundary");
    tl_builder.add_tag("boundary", "administrative");
  }

  auto it = g_mtd_area_map.find(area_id);
  if (it != g_mtd_area_map.end()) {
    auto d = it->second;
    if (!d.admin_lvl.empty())
      tl_builder.add_tag("navteq_admin_level", d.admin_lvl);

    if (!d.admin_lvl.empty())
      tl_builder.add_tag("admin_level", navteq_2_osm_admin_lvl(d.admin_lvl));
    if (!d.name.empty())
      tl_builder.add_tag("name", d.name);
    if (!d.short_name.empty())
      tl_builder.add_tag("short_name", d.short_name);
    if (level != 5) {
      for (auto it : d.lang_code_2_area_name)
        tl_builder.add_tag(std::string("name:" + parse_lang_code(it.first)),
                           it.second);
    }
  } else {
    BOOST_LOG_TRIVIAL(error) << "Skipping unknown navteq_admin_level";
  }
}

/**
 * \brief adds navteq administrative boundary tags to Relation
 */
void build_admin_boundary_taglist(osmium::builder::Builder &builder,
                                  const OGRFeatureUniquePtr &feat) {
  osmium::builder::TagListBuilder tl_builder(builder);

  int level = 0;

  std::string featureCode = feat->GetFieldAsString(FEAT_COD);
  if (featureCode == "907196") {
    level = 1;
  } else if (featureCode == "909996") {
    level = 2;
  } else if (featureCode == "900170") {
    level = 3;
  } else if (featureCode == "900101") {
    level = 4;
  } else if (featureCode == "900156") {
    level = 5;
  } else {
    BOOST_LOG_TRIVIAL(error) << "Unknown admin level " << featureCode;
    return;
  }

  // Mind tl_builder scope in calling method!
  if (level == 5) {
    // only landuse residential
    tl_builder.add_tag("type", "multipolygon");
    tl_builder.add_tag("landuse", "residential");
  } else {
    tl_builder.add_tag("type", "boundary");
    tl_builder.add_tag("boundary", "administrative");
  }

  std::string polygonName = feat->GetFieldAsString(POLYGON_NM);
  if (!polygonName.empty()) {
    std::string waters_name = to_camel_case_with_spaces(polygonName);
    if (!waters_name.empty())
      tl_builder.add_tag("name", waters_name);
  }

  tl_builder.add_tag("admin_level", navteq_2_osm_admin_lvl(level));
}

/**
 * \brief adds navteq landuse tags to Relation
 */
void build_landuse_taglist(osmium::builder::RelationBuilder &builder,
                           const OGRFeatureUniquePtr &feat) {
  // Mind tl_builder scope!
  osmium::builder::TagListBuilder tl_builder(builder);
  tl_builder.add_tag("type", "multipolygon");

  std::string polygonName = feat->GetFieldAsString(POLYGON_NM);
  if (!polygonName.empty()) {
    std::string poly_name = to_camel_case_with_spaces(polygonName);
    if (!poly_name.empty())
      tl_builder.add_tag("name", poly_name);
  }

  std::string featureCode = feat->GetFieldAsString(FEAT_COD);

  // Land Use A types
  if (featureCode == "509998") {
    // FEAT_TYPE 'BEACH'
    tl_builder.add_tag("natural", "beach");
  } else if (featureCode == "900103") {
    // FEAT_TYPE 'PARK/MONUMENT (NATIONAL)'
    tl_builder.add_tag("boundary", "national_park");
  } else if (featureCode == "900130") {
    // FEAT_TYPE 'PARK (STATE)'
    // In many cases this is meant to be a national park or
    // protected area but this is not guaranteed
    tl_builder.add_tag("leisure", "park");
  } else if (featureCode == "900140") {
    // FEAT_TYPE 'PARK IN WATER'
    tl_builder.add_tag("boundary", "national_park");
  } else if (featureCode == "900150") {
    // FEAT_TYPE 'PARK (CITY/COUNTY)'
    tl_builder.add_tag("leisure", "park");
  } else if (featureCode == "900159") {
    // FEAT_TYPE 'UNDEFINED TRAFFIC AREA'
    // Possible handling: area=yes, highway=pedestrian
  } else if (featureCode == "900202") {
    // FEAT_TYPE 'WOODLAND'
    tl_builder.add_tag("landuse", "forest");
  } else if (featureCode == "1700215") {
    // FEAT_TYPE 'PARKING LOT'
    tl_builder.add_tag("amenity", "parking");
  } else if (featureCode == "1900403") {
    // FEAT_TYPE 'AIRPORT'
    tl_builder.add_tag("aeroway", "aerodrome");
  } else if (featureCode == "2000124") {
    // FEAT_TYPE 'SHOPPING CENTRE'
    tl_builder.add_tag("shop", "mall");
    tl_builder.add_tag("building", "retail");
  } else if (featureCode == "2000200") {
    // FEAT_TYPE 'INDUSTRIAL COMPLEX'
    tl_builder.add_tag("landuse", "commercial");
  } else if (featureCode == "2000403") {
    // FEAT_TYPE 'UNIVERSITY/COLLEGE'
    tl_builder.add_tag("amenity", "university");
  } else if (featureCode == "2000408") {
    // FEAT_TYPE 'HOSPITAL'
    tl_builder.add_tag("amenity", "hospital");
  } else if (featureCode == "2000420") {
    // FEAT_TYPE 'CEMETERY'
    tl_builder.add_tag("landuse", "cemetery");
  } else if (featureCode == "2000457") {
    // FEAT_TYPE 'SPORTS COMPLEX'
    tl_builder.add_tag("leisure", "stadium");
    // tl_builder.add_tag("building", "yes");
  } else if (featureCode == "2000460") {
    // FEAT_TYPE 'AMUSEMENT PARK'
    tl_builder.add_tag("leisure", "park");
    tl_builder.add_tag("tourism", "theme_park");
  } else if (featureCode == "908002") {
    // FEAT_TYPE 'Neighbourhood'
    tl_builder.add_tag("place", "suburb");
  } else if (featureCode == "2000461") {
    // FEAT_TYPE 'ANIMAL PARK'
    tl_builder.add_tag("tourism", "zoo");
  }

  // Not implemented so far due to missing sample in data:
  // MILITARY BASE (900108), NATIVE AMERICAN
  // RESERVATION (900107), RAILYARD (9997007)
  // Land Use B types
  else if (featureCode == "900158") {
    // FEAT_TYPE 'PEDESTRIAN ZONE'
    tl_builder.add_tag("highway", "pedestrian");
  } else if (featureCode == "1907403") {
    // FEAT_TYPE 'AIRCRAFT ROADS'
    tl_builder.add_tag("aeroway", "runway");
  } else if (featureCode == "2000123") {
    // FEAT_TYPE 'GOLF COURSE'
    tl_builder.add_tag("leisure", "golf_course");
    tl_builder.add_tag("sport", "golf");
  } else if (featureCode == "9997004") {
    // FEAT_TYPE 'CONGESTION ZONE'
    // skipping due to no osm equivalent
  } else if (featureCode == "9997010") {
    // FEAT_TYPE 'ENVIRONMENTAL ZONE'
    tl_builder.add_tag("boundary", "low_emission_zone");
    tl_builder.add_tag("type", "boundary");
  }
  // Unknown if Land Use A or B types, but seems to appear somewhere
  else if (featureCode == "509997") {
    // FEAT_TYPE 'GLACIER'
    tl_builder.add_tag("natural", "glacier");
  } else {
    BOOST_LOG_TRIVIAL(error) << "Skipping unknown landuse type " << featureCode;
  }
}

void build_relation_members(osmium::builder::RelationBuilder &builder,
                            const osm_id_vector_type &ext_osm_way_ids,
                            const osm_id_vector_type &int_osm_way_ids) {
  osmium::builder::RelationMemberListBuilder rml_builder(builder);

  for (osmium::unsigned_object_id_type osm_id : ext_osm_way_ids)
    rml_builder.add_member(osmium::item_type::way, osm_id, "outer");

  for (osmium::unsigned_object_id_type osm_id : int_osm_way_ids)
    rml_builder.add_member(osmium::item_type::way, osm_id, "inner");
}

osmium::unsigned_object_id_type build_admin_boundary_relation_with_tags(
    osmium::unsigned_object_id_type area_id,
    const osm_id_vector_type &ext_osm_way_ids,
    const osm_id_vector_type &int_osm_way_ids,
    osmium::memory::Buffer &rel_buffer, uint level) {
  osmium::builder::RelationBuilder builder(rel_buffer);
  builder.object().set_id(g_osm_id++);
  set_dummy_osm_object_attributes(builder.object());
  builder.set_user(USER);
  build_admin_boundary_taglist(builder, area_id, level);
  build_relation_members(builder, ext_osm_way_ids, int_osm_way_ids);
  return builder.object().id();
}

osmium::unsigned_object_id_type build_admin_boundary_relation_with_tags(
    const OGRFeatureUniquePtr &feat, const osm_id_vector_type &ext_osm_way_ids,
    const osm_id_vector_type &int_osm_way_ids,
    osmium::memory::Buffer &rel_buffer) {
  osmium::builder::RelationBuilder builder(rel_buffer);
  builder.object().set_id(g_osm_id++);
  set_dummy_osm_object_attributes(builder.object());
  builder.set_user(USER);
  build_admin_boundary_taglist(builder, feat);
  build_relation_members(builder, ext_osm_way_ids, int_osm_way_ids);
  return builder.object().id();
}

osmium::unsigned_object_id_type build_landuse_relation_with_tags(
    const OGRFeatureUniquePtr &feat, osm_id_vector_type ext_osm_way_ids,
    osm_id_vector_type int_osm_way_ids, osmium::memory::Buffer &rel_buffer) {
  osmium::builder::RelationBuilder builder(rel_buffer);
  builder.object().set_id(g_osm_id++);
  set_dummy_osm_object_attributes(builder.object());
  builder.set_user(USER);
  build_landuse_taglist(builder, feat);
  build_relation_members(builder, ext_osm_way_ids, int_osm_way_ids);
  return builder.object().id();
}

void create_polygon(const OGRPolygon *poly,
                    osm_id_vector_type &exterior_way_ids,
                    osm_id_vector_type &interior_way_ids,
                    osmium::memory::Buffer &node_buffer,
                    osmium::memory::Buffer &way_buffer) {
  bool isExteriorRing = true; // first ring is the exterior ring
  for (const OGRLinearRing *ring : *poly) {
    auto tmp = build_closed_ways(ring, node_buffer, way_buffer);
    if (isExteriorRing) {
      std::move(tmp.begin(), tmp.end(), std::back_inserter(exterior_way_ids));
    } else {
      std::move(tmp.begin(), tmp.end(), std::back_inserter(interior_way_ids));
    }
    isExteriorRing = false;
  }
}

/**
 * \brief handles administrative boundary multipolygons
 */
void create_multi_polygon(OGRMultiPolygon *mp,
                          osm_id_vector_type &mp_ext_ring_osm_ids,
                          osm_id_vector_type &mp_int_ring_osm_ids,
                          osmium::memory::Buffer &node_buffer,
                          osmium::memory::Buffer &way_buffer) {

  for (const OGRPolygon *poly : mp) {
    create_polygon(poly, mp_ext_ring_osm_ids, mp_int_ring_osm_ids, node_buffer,
                   way_buffer);
  }
}

/**
 * \brief adds administrative boundaries as Relations to m_buffer
 */
void process_admin_boundary(
    const OGRFeatureUniquePtr &feat, osmium::memory::Buffer &node_buffer,
    osmium::memory::Buffer &way_buffer, osmium::memory::Buffer &rel_buffer,
    std::map<int, std::pair<osm_id_vector_type, osm_id_vector_type>>
        &adminLineMap) {
  auto geom = feat->GetGeometryRef();

  osm_id_vector_type exterior_way_ids, interior_way_ids;
  if (geom->getGeometryType() == wkbMultiPolygon) {
    create_multi_polygon(static_cast<OGRMultiPolygon *>(geom), exterior_way_ids,
                         interior_way_ids, node_buffer, way_buffer);
  } else if (geom->getGeometryType() == wkbPolygon) {
    create_polygon(static_cast<OGRPolygon *>(geom), exterior_way_ids,
                   interior_way_ids, node_buffer, way_buffer);
  } else {
    throw(std::runtime_error("Adminboundaries with geometry=" +
                             std::string(geom->getGeometryName()) +
                             " are not yet supported."));
  }
  osmium::unsigned_object_id_type area_id = feat->GetFieldAsInteger(AREA_ID);
  if (area_id != 0) {
    boost::copy(exterior_way_ids,
                std::back_inserter(adminLineMap[area_id].first));
    boost::copy(interior_way_ids,
                std::back_inserter(adminLineMap[area_id].second));
  } else {
    // need to tag here because there is no unique Area_ID
    build_admin_boundary_relation_with_tags(feat, exterior_way_ids,
                                            interior_way_ids, rel_buffer);
    rel_buffer.commit();
  }

  node_buffer.commit();
  way_buffer.commit();
}

/**
 * \brief adds landuse polygons as Relations to m_buffer
 */
void process_landuse(const OGRFeatureUniquePtr &feat,
                     osmium::memory::Buffer &node_buffer,
                     osmium::memory::Buffer &way_buffer,
                     osmium::memory::Buffer &rel_buffer) {
  auto geom = feat->GetGeometryRef();
  auto geom_type = geom->getGeometryType();

  osm_id_vector_type exterior_way_ids, interior_way_ids;
  if (geom_type == wkbMultiPolygon) {
    create_multi_polygon(static_cast<OGRMultiPolygon *>(geom), exterior_way_ids,
                         interior_way_ids, node_buffer, way_buffer);
  } else if (geom_type == wkbPolygon) {
    create_polygon(static_cast<OGRPolygon *>(geom), exterior_way_ids,
                   interior_way_ids, node_buffer, way_buffer);
  } else {
    throw(std::runtime_error(
        "Landuse item with geometry=" + std::string(geom->getGeometryName()) +
        " is not yet supported."));
  }

  build_landuse_relation_with_tags(feat, exterior_way_ids, interior_way_ids,
                                   rel_buffer);

  node_buffer.commit();
  way_buffer.commit();
  rel_buffer.commit();
}

/**
 * \brief adds cities to the node buffer
 */
void process_city(const OGRFeatureUniquePtr &feat, uint fac_type,
                  osmium::memory::Buffer &node_buffer,
                  const std::map<std::string, std::string> &translations) {

  auto geom = feat->GetGeometryRef();
  auto geom_type = geom->getGeometryType();

  osm_id_vector_type exterior_way_ids, interior_way_ids;
  if (geom_type != wkbPoint) {
    throw(std::runtime_error(
        "City item with geometry=" + std::string(geom->getGeometryName()) +
        " is not yet supported."));
  }

  auto point = static_cast<OGRPoint *>(geom);
  osmium::Location location(point->getX(), point->getY());
  {
    // scope node_builder
    // Add new node
    osmium::builder::NodeBuilder node_builder(node_buffer);
    build_node(location, &node_builder);
    osmium::builder::TagListBuilder tl_builder(node_builder);

    std::string name = feat->GetFieldAsString(POI_NAME);
    tl_builder.add_tag("name", to_camel_case_with_spaces(name));
    if (fac_type == 9709) {
      // 9709 means 'neighbourhood'
      tl_builder.add_tag("place", "suburb");
    } else { //=> fac_type = 4444 means 'named place' which is the city centre
      int population = feat->GetFieldAsInteger(POPULATION);
      if (population > 0)
        tl_builder.add_tag("population", std::to_string(population));
      uint capital = feat->GetFieldAsInteger(CAPITAL);
      tl_builder.add_tag("place", get_place_value(population, capital));
      if (capital == 1) {
        // for capitals of countries use 'capital' = 'yes'
        tl_builder.add_tag("capital", "yes");
      } else if (capital > 1) {
        // for subdivisions of countries use 'capital' = admin_lvl
        tl_builder.add_tag("capital", navteq_2_osm_admin_lvl(capital));
      }
    }

    // add translation tags
    for (auto loc : translations) {
      tl_builder.add_tag("name:" + loc.first, loc.second);
    }
  }
  node_buffer.commit();
}

/**
 * \brief adds hamlets to the node_buffer
 */
void process_hamlets(const OGRFeatureUniquePtr &feat,
                     osmium::memory::Buffer &node_buffer) {

  auto geom = feat->GetGeometryRef();
  auto geom_type = geom->getGeometryType();

  if (geom_type != wkbPoint) {
    throw(std::runtime_error(
        "Hamlet item with geometry=" + std::string(geom->getGeometryName()) +
        " is not yet supported."));
  }

  auto point = static_cast<OGRPoint *>(geom);
  osmium::Location location(point->getX(), point->getY());
  {
    // scope node_builder
    // Add new node
    osmium::builder::NodeBuilder node_builder(node_buffer);
    build_node(location, &node_builder);
    osmium::builder::TagListBuilder tl_builder(node_builder);

    std::string name = feat->GetFieldAsString(POI_NAME);
    tl_builder.add_tag("name", to_camel_case_with_spaces(name));
    tl_builder.add_tag("place", "hamlet");
  }
  node_buffer.commit();
}

/**
 * \brief adds tags from administrative boundaries to mtd_area_map.
 * 		  adds tags from administrative boundaries to mtd_area_map
 * 		  to be accesible when creating the Relations of
 * 		  administrative boundaries.
 * \param handle file handle to navteq administrative meta data.
 */
void process_meta_areas(boost::filesystem::path dir) {
  DBFHandle handle = read_dbf_file(dir / MTD_AREA_DBF);

  for (int i = 0; i < DBFGetRecordCount(handle); i++) {

    osmium::unsigned_object_id_type area_id =
        dbf_get_uint_by_field(handle, i, AREA_ID);

    // find or create a new area data set
    mtd_area_dataset &data = g_mtd_area_map[area_id];

    data.area_id = area_id;

    std::string admin_lvl =
        std::to_string(dbf_get_uint_by_field(handle, i, ADMIN_LVL));
    if (data.admin_lvl.empty()) {
      data.admin_lvl = admin_lvl;
    } else if (data.admin_lvl != admin_lvl) {
      BOOST_LOG_TRIVIAL(error)
          << "entry with area_id=" << area_id
          << " has multiple admin_lvls:" << data.admin_lvl << ", " << admin_lvl;
    }

    std::string lang_code = dbf_get_string_by_field(handle, i, LANG_CODE);
    std::string area_name = dbf_get_string_by_field(handle, i, AREA_NAME);

    std::string area_type = dbf_get_string_by_field(handle, i, "AREA_TYPE");
    if (area_type == "B") {
      data.name = to_camel_case_with_spaces(area_name);
      data.lang_code_2_area_name.emplace_back(
          lang_code, to_camel_case_with_spaces(area_name));
      data.area_code_1 = dbf_get_uint_by_field(handle, i, AREA_CODE_1);
    } else if (area_type == "A") {
      data.short_name = to_camel_case_with_spaces(area_name);
    } else {
      data.lang_code_2_area_name.emplace_back(
          lang_code, to_camel_case_with_spaces(area_name));
      data.area_code_1 = dbf_get_uint_by_field(handle, i, AREA_CODE_1);
    }
  }
  DBFClose(handle);
}

void preprocess_meta_areas(const std::vector<boost::filesystem::path> &dirs) {
  for (auto dir : dirs) {
    process_meta_areas(dir);
  }
}

void init_cdms_map(
    DBFHandle cdms_handle,
    std::map<osmium::unsigned_object_id_type, ushort> &cdms_map) {
  for (int i = 0; i < DBFGetRecordCount(cdms_handle); i++) {
    osmium::unsigned_object_id_type cond_id =
        dbf_get_uint_by_field(cdms_handle, i, COND_ID);
    ushort cond_type = dbf_get_uint_by_field(cdms_handle, i, COND_TYPE);
    cdms_map.insert(std::make_pair(cond_id, cond_type));
  }
}

void add_city_nodes(const std::vector<boost::filesystem::path> &dirs,
                    osmium::io::Writer &writer) {

  for (auto dir : dirs) {
    auto ds = open_shape_file(dir / NAMED_PLC_SHP);
    auto layer = ds->GetLayer(0);
    if (layer == nullptr)
      throw(shp_empty_error(dir.string()));

    osmium::memory::Buffer node_buffer(buffer_size);

    int facTypeField = layer->FindFieldIndex(FAC_TYPE, true);
    int poiNmTypeField = layer->FindFieldIndex(POI_NMTYPE, true);
    int poiLangCodeField = layer->FindFieldIndex(POI_LANGCD, true);
    int poiIdField = layer->FindFieldIndex(POI_ID, true);
    int poiNameField = layer->FindFieldIndex(POI_NAME, true);

    std::map<uint64_t, std::map<std::string, std::string>> translations;
    // read translations
    for (auto &feat : *layer) {
      uint fac_type = feat->GetFieldAsInteger(facTypeField);
      if (fac_type != 4444 && fac_type != 9709) {
        BOOST_LOG_TRIVIAL(error)
            << "Skipping city node because of wrong POI type";
        continue;
      }
      std::string name_type = feat->GetFieldAsString(poiNmTypeField);
      if (name_type == "B") {
        // Skip this entry as it's just a translated namePlc of former one
        continue;
      }
      int poiId = feat->GetFieldAsInteger(poiIdField);
      std::string langCode = feat->GetFieldAsString(poiLangCodeField);
      std::string locName = feat->GetFieldAsString(poiNameField);

      translations[poiId].emplace(parse_lang_code(langCode),
                                  to_camel_case_with_spaces(locName));
    }

    layer->ResetReading();

    for (auto &feat : *layer) {
      uint fac_type = feat->GetFieldAsInteger(facTypeField);
      if (fac_type != 4444 && fac_type != 9709) {
        BOOST_LOG_TRIVIAL(error)
            << "Skipping city node because of wrong POI type";
        continue;
      }

      std::string name_type = feat->GetFieldAsString(poiNmTypeField);
      if (name_type != "B") {
        // Skip this entry as it's just a translated namePlc of former one
        continue;
      }
      int poiId = feat->GetFieldAsInteger(poiIdField);

      process_city(feat, fac_type, node_buffer, translations[poiId]);
    }
    node_buffer.commit();
    writer(std::move(node_buffer));
  }
}

void add_hamlet_nodes(const std::vector<boost::filesystem::path> &dirs,
                      osmium::io::Writer &writer) {

  for (auto dir : dirs) {

    // hamlets are optional
    if (!shp_file_exists(dir / HAMLET_SHP))
      continue;

    auto ds = open_shape_file(dir / HAMLET_SHP);
    auto layer = ds->GetLayer(0);
    if (layer == nullptr)
      throw(shp_empty_error(dir.string()));

    osmium::memory::Buffer node_buffer(buffer_size);

    int facTypeField = layer->FindFieldIndex(FAC_TYPE, true);
    int poiNmTypeField = layer->FindFieldIndex(POI_NMTYPE, true);

    for (auto &feat : *layer) {
      uint fac_type = feat->GetFieldAsInteger(facTypeField);
      if (fac_type != 9998) {
        BOOST_LOG_TRIVIAL(error)
            << "Skipping hamlet node because of wrong POI type";
        continue;
      }

      std::string name_type = feat->GetFieldAsString(poiNmTypeField);
      if (name_type != "B") {
        // Skip this entry as it's just a translated namePlc of former one
        continue;
      }
      process_hamlets(feat, node_buffer);
    }
    writer(std::move(node_buffer));
  }
}

void init_g_cnd_mod_map(const boost::filesystem::path &dir) {
  DBFHandle cnd_mod_handle = read_dbf_file(dir / CND_MOD_DBF);
  for (int i = 0; i < DBFGetRecordCount(cnd_mod_handle); i++) {
    cond_id_type cond_id = dbf_get_uint_by_field(cnd_mod_handle, i, COND_ID);
    std::string lang_code =
        dbf_get_string_by_field(cnd_mod_handle, i, LANG_CODE);
    mod_typ_type mod_type =
        dbf_get_uint_by_field(cnd_mod_handle, i, CM_MOD_TYPE);
    mod_val_type mod_val = dbf_get_uint_by_field(cnd_mod_handle, i, CM_MOD_VAL);

    auto it2 = g_cnd_mod_map.find(cond_id);
    if (it2 == g_cnd_mod_map.end()) {
      mod_group_vector_type new_vector;
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

void init_g_cdms_map(const boost::filesystem::path &dir) {
  DBFHandle cdms_handle = read_dbf_file(dir / CDMS_DBF);
  for (int i = 0; i < DBFGetRecordCount(cdms_handle); i++) {
    link_id_type link_id = dbf_get_uint_by_field(cdms_handle, i, LINK_ID);
    cond_id_type cond_id = dbf_get_uint_by_field(cdms_handle, i, COND_ID);
    ushort cond_type = dbf_get_uint_by_field(cdms_handle, i, COND_TYPE);
    g_cdms_map.emplace(link_id, cond_pair_type(cond_id, cond_type));
  }
  DBFClose(cdms_handle);
}

void init_g_area_to_govt_code_map(const boost::filesystem::path &dir) {
  DBFHandle mtd_area_handle = read_dbf_file(dir / MTD_AREA_DBF);
  for (int i = 0; i < DBFGetRecordCount(mtd_area_handle); i++) {
    area_id_type area_id = dbf_get_uint_by_field(mtd_area_handle, i, AREA_ID);
    govt_code_type govt_code =
        dbf_get_uint_by_field(mtd_area_handle, i, GOVT_CODE);
    g_area_to_govt_code_map.emplace(area_id, govt_code);
  }
  DBFClose(mtd_area_handle);
}

void init_g_cntry_ref_map(const boost::filesystem::path &dir) {
  DBFHandle cntry_ref_handle = read_dbf_file(dir / MTD_CNTRY_REF_DBF);
  for (int i = 0; i < DBFGetRecordCount(cntry_ref_handle); i++) {
    govt_code_type govt_code =
        dbf_get_uint_by_field(cntry_ref_handle, i, GOVT_CODE);
    auto unit_measure =
        dbf_get_string_by_field(cntry_ref_handle, i, UNTMEASURE);
    auto speed_limit_unit =
        dbf_get_string_by_field(cntry_ref_handle, i, SPEEDLIMITUNIT);
    auto iso_code = dbf_get_string_by_field(cntry_ref_handle, i, ISO_CODE);
    auto cntry_ref = cntry_ref_type(unit_measure, speed_limit_unit, iso_code);
    g_cntry_ref_map.emplace(govt_code, cntry_ref);
  }
  DBFClose(cntry_ref_handle);
}

// \brief stores z_levels in z_level_map for later use. Maps link_ids to pairs
// of indices and z-levels of waypoints with z-levels not equal 0.
void init_z_level_map(boost::filesystem::path dir, z_lvl_map &z_level_map) {

  // open dbf
  DBFHandle handle = read_dbf_file(dir / ZLEVELS_DBF);

  link_id_type last_link_id = 0;
  index_z_lvl_vector_type v;

  for (int i = 0; i < DBFGetRecordCount(handle); i++) {
    link_id_type link_id = dbf_get_uint_by_field(handle, i, LINK_ID);
    ushort point_num = dbf_get_uint_by_field(handle, i, POINT_NUM) - 1;
    short z_level = dbf_get_uint_by_field(handle, i, Z_LEVEL);

    if (i > 0 && last_link_id != link_id && !v.empty()) {
      z_level_map.emplace(last_link_id, v);
      v = index_z_lvl_vector_type();
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

void init_conditional_driving_manoeuvres(const boost::filesystem::path &dir) {
  if (dbf_file_exists(dir / CND_MOD_DBF) && dbf_file_exists(dir / CDMS_DBF)) {
    init_g_cnd_mod_map(dir);
    init_g_cdms_map(dir);
  }
}

void init_country_reference(const boost::filesystem::path &dir) {
  if (dbf_file_exists(dir / MTD_AREA_DBF) &&
      dbf_file_exists(dir / MTD_CNTRY_REF_DBF)) {
    init_g_area_to_govt_code_map(dir);
    init_g_cntry_ref_map(dir);
  }
}

void init_under_construction(const boost::filesystem::path &dir) {
  if (!dbf_file_exists(dir / CDMS_DBF))
    return;

  DBFHandle cond = read_dbf_file(dir / CDMS_DBF);
  for (int i = 0; i < DBFGetRecordCount(cond); i++) {
    link_id_type link_id = dbf_get_uint_by_field(cond, i, LINK_ID);
    uint condType = dbf_get_uint_by_field(cond, i, COND_TYPE);

    if (condType == 3)
      g_construction_set.emplace(link_id);
  }
  DBFClose(cond);
}

void parse_highway_names(const boost::filesystem::path &dbf_file,
                         bool isStreetLayer) {
  DBFHandle hwys_handle = read_dbf_file(dbf_file);
  for (int i = 0; i < DBFGetRecordCount(hwys_handle); i++) {

    link_id_type link_id = dbf_get_uint_by_field(hwys_handle, i, LINK_ID);
    std::string hwy_name;
    if (isStreetLayer)
      hwy_name = dbf_get_string_by_field(hwys_handle, i, ST_NAME);
    else
      hwy_name = dbf_get_string_by_field(hwys_handle, i, HIGHWAY_NM);

    uint routeType = dbf_get_uint_by_field(hwys_handle, i, ROUTE);

    g_hwys_ref_map[link_id].emplace(routeType, hwy_name);
  }
  DBFClose(hwys_handle);
}

void init_highway_names(const boost::filesystem::path &dir) {
  if (dbf_file_exists(dir / MAJ_HWYS_DBF))
    parse_highway_names(dir / MAJ_HWYS_DBF, false);
  if (dbf_file_exists(dir / SEC_HWYS_DBF))
    parse_highway_names(dir / SEC_HWYS_DBF, false);
  if (dbf_file_exists(dir / ALT_STREETS_DBF))
    parse_highway_names(dir / ALT_STREETS_DBF, true);
  if (dbf_file_exists(dir / STREETS_DBF))
    parse_highway_names(dir / STREETS_DBF, true);
}

void parse_ramp_names(
    const boost::filesystem::path &shp_file,
    const std::map<link_id_type, std::string> &junctionNames) {

  auto ds = open_shape_file(shp_file);
  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(shp_file.string()));

  int exitNameField = layer->FindFieldIndex(EXITNAME, true);
  int linkIdField = layer->FindFieldIndex(LINK_ID, true);
  int baseNameField = layer->FindFieldIndex(ST_NM_BASE, true);
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

std::map<link_id_type, std::string>
read_junction_names(const boost::filesystem::path &dbf_file) {
  DBFHandle hwys_handle = read_dbf_file(dbf_file);
  std::map<link_id_type, std::string> junctionNames;
  for (int i = 0; i < DBFGetRecordCount(hwys_handle); i++) {

    link_id_type link_id = dbf_get_uint_by_field(hwys_handle, i, LINK_ID);
    std::string ramp_name = dbf_get_string_by_field(hwys_handle, i, ST_NM_BASE);

    if (parse_bool(
            dbf_get_string_by_field(hwys_handle, i, JUNCTIONNM).c_str())) {
      junctionNames[link_id] = ramp_name;
    }
  }
  DBFClose(hwys_handle);

  return junctionNames;
}

void init_ramp_names(const boost::filesystem::path &dir) {
  // read junction names from alt_streets
  auto junctionMap = read_junction_names(dir / ALT_STREETS_DBF);

  // create location ramps map
  parse_ramp_names(dir / STREETS_SHP, junctionMap);
}

z_lvl_map process_z_levels(const std::vector<boost::filesystem::path> &dirs) {

  z_lvl_map z_level_map;

  for (auto &dir : dirs) {
    auto path = dir / STREETS_SHP;
    auto ds = open_shape_file(path);

    auto layer = ds->GetLayer(0);
    if (layer == nullptr)
      throw(shp_empty_error(path.string()));
    assert(layer->GetGeomType() == wkbLineString);

    init_z_level_map(dir, z_level_map);
    init_conditional_driving_manoeuvres(dir);
    init_country_reference(dir);
  }
  return z_level_map;
}

void process_way_end_nodes(const std::vector<boost::filesystem::path> &dirs,
                           z_lvl_map &z_level_map, osmium::io::Writer &writer) {
  for (auto &dir : dirs) {

    // parse ramp names and refs
    init_ramp_names(dir);

    auto path = dir / STREETS_SHP;
    auto ds = open_shape_file(path);

    auto layer = ds->GetLayer(0);
    if (layer == nullptr)
      throw(shp_empty_error(path.string()));

    osmium::memory::Buffer node_buffer(buffer_size);

    int linkIDField = layer->FindFieldIndex(LINK_ID, true);

    // get all nodes which may be a routable crossing
    for (auto &feat : *layer) {
      link_id_type link_id = feat->GetFieldAsInteger(linkIDField);
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

void process_way(const std::vector<boost::filesystem::path> &dirs,
                 z_lvl_map &z_level_map, osmium::io::Writer &writer) {
  for (auto &dir : dirs) {
    // parse highway names and refs
    init_highway_names(dir);

    // parse conditionals
    init_under_construction(dir);

    auto path = dir / STREETS_SHP;
    auto ds = open_shape_file(path);

    auto layer = ds->GetLayer(0);
    if (layer == nullptr)
      throw(shp_empty_error(path.string()));
    boost::timer::progress_display progress(layer->GetFeatureCount());

    osmium::memory::Buffer node_buffer(buffer_size);
    osmium::memory::Buffer way_buffer(buffer_size);
    for (auto &feat : *layer) {
      process_way(feat, &z_level_map, node_buffer, way_buffer);
      ++progress;
    }

    node_buffer.commit();
    way_buffer.commit();
    writer(std::move(node_buffer));
    writer(std::move(way_buffer));

    g_hwys_ref_map.clear();
  }
}

auto createPointAddressMapList(const boost::filesystem::path dir) {

  auto pointAddressMap =
      new std::map<uint64_t,
                   std::vector<std::pair<osmium::Location, std::string>>>();
  if (shp_file_exists(dir / POINT_ADDRESS_SHP)) {
    auto ds = open_shape_file(dir / POINT_ADDRESS_SHP);

    auto layer = ds->GetLayer(0);
    if (layer == nullptr)
      throw(shp_empty_error((dir / POINT_ADDRESS_SHP).string()));

    int linkIdField = layer->FindFieldIndex(LINK_ID, true);
    int latField = layer->FindFieldIndex("DISP_LAT", true);
    int lonField = layer->FindFieldIndex("DISP_LON", true);
    int addressField = layer->FindFieldIndex("ADDRESS", true);

    for (auto &feat : *layer) {
      int linkId = feat->GetFieldAsInteger(linkIdField);
      auto houseNumber = std::string(feat->GetFieldAsString(addressField));

      double lat = 0.0;
      double lon = 0.0;

      if (feat->IsFieldNull(lonField) && feat->IsFieldNull(latField)) {
        auto point = static_cast<const OGRPoint *>(feat->GetGeometryRef());
        lat = point->getY();
        lon = point->getX();
      } else {
        lon = feat->GetFieldAsDouble(lonField);
        lat = feat->GetFieldAsDouble(latField);
      }

      (*pointAddressMap)[linkId].emplace_back(osmium::Location(lon, lat),
                                              houseNumber);
    }
  }
  return pointAddressMap;
}

void process_house_numbers(const std::vector<boost::filesystem::path> &dirs,
                           osmium::io::Writer &writer) {
  for (auto &dir : dirs) {
    // create point addresses from PointAddress.dbf
    auto pointMap = createPointAddressMapList(dir);

    auto path = dir / STREETS_SHP;
    auto ds = open_shape_file(path);

    auto layer = ds->GetLayer(0);
    if (layer == nullptr)
      throw(shp_empty_error(path.string()));
    boost::timer::progress_display progress(layer->GetFeatureCount());

    osmium::memory::Buffer node_buffer(buffer_size);
    osmium::memory::Buffer way_buffer(buffer_size);

    int linkIdField = layer->FindFieldIndex(LINK_ID, true);

    for (auto &feat : *layer) {
      int linkId = feat->GetFieldAsInteger(linkIdField);
      process_house_numbers(feat, pointMap, linkId, node_buffer, way_buffer);
      ++progress;
    }

    node_buffer.commit();
    way_buffer.commit();
    writer(std::move(node_buffer));
    writer(std::move(way_buffer));
    delete pointMap;
  }
}

/**
 * \brief Parses AltStreets.dbf for route type values.
 */
void process_alt_steets_route_types(
    const std::vector<boost::filesystem::path> &dirs) {
  for (auto dir : dirs) {
    DBFHandle alt_streets_handle = read_dbf_file(dir / ALT_STREETS_DBF);
    for (int i = 0; i < DBFGetRecordCount(alt_streets_handle); i++) {

      if (dbf_get_string_by_field(alt_streets_handle, i, ROUTE).empty())
        continue;

      osmium::unsigned_object_id_type link_id =
          dbf_get_uint_by_field(alt_streets_handle, i, LINK_ID);
      ushort route_type = dbf_get_uint_by_field(alt_streets_handle, i, ROUTE);

      // try to emplace <link_id, route_type> pair
      auto [insertion, inserted] =
          g_route_type_map.emplace(link_id, route_type);

      // if its already exists update routetype
      if (!inserted && insertion->second > route_type) {
        // As link id's aren't unique in AltStreets.dbf
        // just store the lowest route type
        insertion->second = route_type;
      }
    }
    DBFClose(alt_streets_handle);
  }
}

/****************************************************
 * adds layers to osmium:
 *      cur_layer and cur_feature have to be set
 ****************************************************/

/**
 * \brief adds streets to m_buffer / osmium.
 * \param layer Pointer to administrative layer.
 */

void add_street_shapes(const std::vector<boost::filesystem::path> &dirs,
                       osmium::io::Writer &writer) {

  BOOST_LOG_TRIVIAL(info) << " processing z-levels";
  z_lvl_map z_level_map = process_z_levels(dirs);

  BOOST_LOG_TRIVIAL(info) << " processing way end points";
  process_way_end_nodes(dirs, z_level_map, writer);

  BOOST_LOG_TRIVIAL(info) << " processing ways";
  process_way(dirs, z_level_map, writer);

  // create house numbers
  BOOST_LOG_TRIVIAL(info) << " processing house numbers";
  process_house_numbers(dirs, writer);

  BOOST_LOG_TRIVIAL(info) << " clean";
  for (auto elem : z_level_map)
    elem.second.clear();
  z_level_map.clear();

  // now we can clear some maps:
  g_hwys_ref_map.clear();
  g_cdms_map.clear();
  g_cnd_mod_map.clear();
  g_area_to_govt_code_map.clear();
  g_cntry_ref_map.clear();
  g_z_lvl_nodes_map.clear();
  g_route_type_map.clear();
}

void add_street_shapes(const boost::filesystem::path &dir,
                       osmium::io::Writer &writer) {
  std::vector<boost::filesystem::path> dir_vector;
  dir_vector.push_back(dir);
  add_street_shapes(dir_vector, writer);
}

/**
 * \brief adds administrative boundaries to m_buffer / osmium.
 * \param layer pointer to administrative layer.
 */

void add_admin_shape(
    boost::filesystem::path admin_shape_file, osmium::io::Writer &writer,
    std::map<int, std::pair<osm_id_vector_type, osm_id_vector_type>>
        &adminLineMap) {

  auto ds = open_shape_file(admin_shape_file);
  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(admin_shape_file.string()));
  assert(layer->GetGeomType() == wkbPolygon);
  osmium::memory::Buffer node_buffer(buffer_size);
  osmium::memory::Buffer way_buffer(buffer_size);
  osmium::memory::Buffer rel_buffer(buffer_size);
  for (auto &feat : *layer) {
    process_admin_boundary(feat, node_buffer, way_buffer, rel_buffer,
                           adminLineMap);
  }
  writer(std::move(node_buffer));
  writer(std::move(way_buffer));
  writer(std::move(rel_buffer));
}

void add_landuse_shape(boost::filesystem::path landuse_shape_file,
                       osmium::io::Writer &writer) {
  g_way_end_points_map.clear();
  auto ds = open_shape_file(landuse_shape_file);
  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(landuse_shape_file.string()));
  assert(layer->GetGeomType() == wkbPolygon);
  osmium::memory::Buffer node_buffer(buffer_size);
  osmium::memory::Buffer way_buffer(buffer_size);
  osmium::memory::Buffer rel_buffer(buffer_size);
  for (auto &feat : *layer) {
    process_landuse(feat, node_buffer, way_buffer, rel_buffer);
  }

  writer(std::move(node_buffer));
  writer(std::move(way_buffer));
  writer(std::move(rel_buffer));
  g_way_end_points_map.clear();
}

/****************************************************
 * cleanup and assertions
 ****************************************************/

/**
 * \brief clears all global variables.
 */

void clear_all() {
  g_osm_id = 1;
  g_hwys_ref_map.clear();

  g_mtd_area_map.clear();
}

osm_id_vector_type build_admin_line(OGRFeatureUniquePtr &feat,
                                    osmium::memory::Buffer &node_buffer,
                                    osmium::memory::Buffer &way_buffer) {

  auto line = static_cast<OGRLineString *>(feat->GetGeometryRef());

  node_vector_type osm_way_node_ids = create_open_way_nodes(line, node_buffer);

  osm_id_vector_type osm_way_ids;
  size_t i = 0;
  do {
    osmium::builder::WayBuilder builder(way_buffer);
    builder.object().set_id(g_osm_id++);
    set_dummy_osm_object_attributes(builder.object());
    builder.set_user(USER);
    osmium::builder::WayNodeListBuilder wnl_builder(builder);
    for (size_t j = i;
         j < std::min(i + OSM_MAX_WAY_NODES, osm_way_node_ids.size()); j++)
      wnl_builder.add_node_ref(osm_way_node_ids.at(j).second,
                               osm_way_node_ids.at(j).first);
    osm_way_ids.push_back(builder.object().id());
    i += OSM_MAX_WAY_NODES - 1;
  } while (i < osm_way_node_ids.size());
  return osm_way_ids;
}

/**
 * \brief adds administrative lines to m_buffer / osmium.
 * \param layer pointer to administrative layer.
 */
std::map<int, osm_id_vector_type>
add_admin_lines(boost::filesystem::path admin_line_shape_file,
                osmium::io::Writer &writer) {
  std::map<int, osm_id_vector_type> result;

  auto ds = open_shape_file(admin_line_shape_file);
  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(admin_line_shape_file.string()));
  assert(layer->GetGeomType() == wkbLineString);
  osmium::memory::Buffer node_buffer(buffer_size);
  osmium::memory::Buffer way_buffer(buffer_size);

  std::set<std::pair<link_id_type, area_id_type>> convertedLinkIds;

  for (auto &feat : *layer) {
    auto areaId = feat->GetFieldAsInteger(AREA_ID);
    auto linkId = feat->GetFieldAsInteger(LINK_ID);

    if (convertedLinkIds.find(std::make_pair(linkId, areaId)) ==
        convertedLinkIds.end()) {
      auto osmIdVector = build_admin_line(feat, node_buffer, way_buffer);
      boost::copy(osmIdVector, std::back_inserter(result[areaId]));
      convertedLinkIds.insert(std::make_pair(linkId, areaId));
    }
  }
  node_buffer.commit();
  way_buffer.commit();

  writer(std::move(node_buffer));
  writer(std::move(way_buffer));

  return result;
}

void addLevel1Boundaries(const std::vector<boost::filesystem::path> &dirs,
                         osmium::io::Writer &writer) {
  std::map<int, std::pair<osm_id_vector_type, osm_id_vector_type>> map;

  for (auto dir : dirs) {
    // for some countries the Adminbndy1.shp doesn't contain the whole country
    // border therefore we additionally add the links from AdminLine1.shp
    if (shp_file_exists(dir / ADMINLINE_1_SHP)) {
      auto adminLine = add_admin_lines(dir / ADMINLINE_1_SHP, writer);
      // merge maps
      for (auto &mapEntry : adminLine) {
        boost::copy(mapEntry.second,
                    std::back_inserter(map[mapEntry.first].first));
      }
    } else if (shp_file_exists(dir / ADMINBNDY_1_SHP)) {
      add_admin_shape(dir / ADMINBNDY_1_SHP, writer, map);
    }
  }

  // create relations for admin boundary 1
  osmium::memory::Buffer rel_buffer(buffer_size);
  for (auto &adminBoundary : map) {

    build_admin_boundary_relation_with_tags(
        adminBoundary.first, adminBoundary.second.first,
        adminBoundary.second.second, rel_buffer, 1);
  }
  rel_buffer.commit();
  writer(std::move(rel_buffer));
}

void addLevelNBoundaries(boost::filesystem::path dir,
                         osmium::io::Writer &writer, uint level) {
  std::map<int, std::pair<osm_id_vector_type, osm_id_vector_type>> map;
  add_admin_shape(dir, writer, map);

  // bild boundary relation
  osmium::memory::Buffer rel_buffer(buffer_size);
  for (auto &adminBoundary : map) {

    build_admin_boundary_relation_with_tags(
        adminBoundary.first, adminBoundary.second.first,
        adminBoundary.second.second, rel_buffer, level);
  }
  rel_buffer.commit();
  writer(std::move(rel_buffer));
}

#endif /* NAVTEQ_HPP_ */
