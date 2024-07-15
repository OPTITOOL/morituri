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
