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

// STREETS columns
const char *LINK_ID = "LINK_ID";

const char *ADDR_TYPE = "ADDR_TYPE";
const char *L_REFADDR = "L_REFADDR";
const char *L_NREFADDR = "L_NREFADDR";
const char *L_ADDRSCH = "L_ADDRSCH";
// const char *L_ADDRFORM = "L_ADDRFORM";
const char *R_REFADDR = "R_REFADDR";
const char *R_NREFADDR = "R_NREFADDR";
const char *R_ADDRSCH = "R_ADDRSCH";
// const char *R_ADDRFORM = "R_ADDRFORM";

// const char *AREA_NAME_LANG_CODE = "NM_LANGCD";

// RDMS_DBF columns
// const char* LINK_ID = "LINK_ID";
const char *COND_ID = "COND_ID";

// CDMS_DBF columns

// const char *COND_VAL1 = "COND_VAL1";
// const char *COND_VAL2 = "COND_VAL2";
// const char *COND_VAL3 = "COND_VAL3";
// const char *COND_VAL4 = "COND_VAL4";

// MAJ_HWYS columns
// const char* LINK_ID = "LINK_ID";

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

#define YES "yes"
#define NO "no"

#define NAVTEQ_ADMIN_LVL_MIN 1
#define NAVTEQ_ADMIN_LVL_MAX 7

#define OSM_MAX_WAY_NODES 1000

// default tags for osm nodes ways and relations
#define USER "import"
#define VERSION "1"
#define CHANGESET "1"
#define USERID "1"
#define TIMESTAMP 1
} // namespace

#endif /* PLUGINS_NAVTEQ_MAPPINGS_HPP_ */
