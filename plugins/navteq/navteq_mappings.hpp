/*
 * navteq_mappings.hpp
 *
 *  Created on: 22.06.2015
 *      Author: philip
 */

#ifndef PLUGINS_NAVTEQ_NAME_MAPPING_HPP_
#define PLUGINS_NAVTEQ_NAME_MAPPING_HPP_

#include <osmium/osm.hpp>
#include <osmium/osm/object.hpp>

namespace {

static const std::filesystem::path STREETS_SHP = "Streets.shp";
static const std::filesystem::path ADMINLINE_1_SHP = "AdminLine1.shp";
static const std::filesystem::path ADMINBNDY_1_SHP = "Adminbndy1.shp";
static const std::filesystem::path ADMINBNDY_2_SHP = "Adminbndy2.shp";
static const std::filesystem::path ADMINBNDY_3_SHP = "Adminbndy3.shp";
static const std::filesystem::path ADMINBNDY_4_SHP = "Adminbndy4.shp";
static const std::filesystem::path ADMINBNDY_5_SHP = "Adminbndy5.shp";

static const std::filesystem::path RDMS_DBF = "Rdms.dbf";

} // namespace

#endif /* PLUGINS_NAVTEQ_MAPPINGS_HPP_ */
