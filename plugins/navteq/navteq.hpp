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

// map for conditional modifications
cnd_mod_map_type g_cnd_mod_map;

// map for conditional driving manoeuvres
cdms_map_type g_cdms_map;
std::map<area_id_type, govt_code_type> g_area_to_govt_code_map;
cntry_ref_map_type g_cntry_ref_map;

bool debugMode = false;

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

#endif /* NAVTEQ_HPP_ */
