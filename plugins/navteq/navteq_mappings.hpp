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

static const boost::filesystem::path STREETS_SHP = "Streets.shp";
static const boost::filesystem::path ADMINLINE_1_SHP = "AdminLine1.shp";
static const boost::filesystem::path ADMINBNDY_1_SHP = "Adminbndy1.shp";
static const boost::filesystem::path ADMINBNDY_2_SHP = "Adminbndy2.shp";
static const boost::filesystem::path ADMINBNDY_3_SHP = "Adminbndy3.shp";
static const boost::filesystem::path ADMINBNDY_4_SHP = "Adminbndy4.shp";
static const boost::filesystem::path ADMINBNDY_5_SHP = "Adminbndy5.shp";

static const boost::filesystem::path POINT_ADDRESS_SHP = "PointAddress.shp";

static const boost::filesystem::path MTD_CNTRY_REF_DBF = "MtdCntryRef.dbf";

static const boost::filesystem::path RDMS_DBF = "Rdms.dbf";

static const boost::filesystem::path MAJ_HWYS_DBF = "MajHwys.dbf";
static const boost::filesystem::path SEC_HWYS_DBF = "SecHwys.dbf";

static const boost::filesystem::path STREETS_DBF = "Streets.dbf";

// condition types (CT)
#define CT_RESTRICTED_DRIVING_MANOEUVRE 7
#define CT_TRANSPORT_ACCESS_RESTRICTION 23
#define CT_TRANSPORT_RESTRICTED_DRIVING_MANOEUVRE 26

// modifier types (MT)
#define MT_HAZARDOUS_RESTRICTION 39
#define MT_HEIGHT_RESTRICTION 41
#define MT_WEIGHT_RESTRICTION 42
#define MT_WEIGHT_PER_AXLE_RESTRICTION 43
#define MT_LENGTH_RESTRICTION 44
#define MT_WIDTH_RESTRICTION 45

#define RESTRICTED_DRIVING_MANOEUVRE 7

} // namespace

#endif /* PLUGINS_NAVTEQ_MAPPINGS_HPP_ */
