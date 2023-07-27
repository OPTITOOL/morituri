/*
 * util.hpp
 *
 *  Created on: 17.06.2015
 *      Author: philip
 */

#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <boost/filesystem/path.hpp>
#include <map>
#include <memory>
#include <ogr_api.h>
#include <ogrsf_frmts.h>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/osm/types.hpp>
#include <shapefil.h>
#include <sstream>
#include <string>
#include <unicode/unistr.h>

#include "../plugins/comm2osm_exceptions.hpp"
#include "ogr_util.hpp"
#include "readers.hpp"

const int INCH_BASE = 12;
const int POUND_BASE = 2000;
// short ton in metric tons (source:
// http://wiki.openstreetmap.org/wiki/Key:maxweight)
const double SHORT_TON = 0.90718474;

/**

 * helpers to check for file existance and validity
 * */

/**
 * \brief Checks shapefile existance and validity
 * \param shp_file path of SHP file
 * \return Returns true if existing and valid
 * */

bool shp_file_exists(const std::string &shp_file) {
  auto input_data_source =
      GDALDatasetUniquePtr(GDALDataset::Open(shp_file.c_str()));
  return input_data_source != nullptr;
}

bool shp_file_exists(boost::filesystem::path shp_file) {
  return shp_file_exists(shp_file.string());
}

/**
 * \brief Checks DBF file existance and validity
 * \param dbf_file path of DBF file
 * \return Returns true if existing and valid
 * */
bool dbf_file_exists(const std::string &dbf_file) {
  DBFHandle handle = DBFOpen(dbf_file.c_str(), "rb");
  if (handle == nullptr) {
    return false;
  }
  DBFClose(handle);
  return true;
}

bool dbf_file_exists(const boost::filesystem::path &dbf_file) {
  return dbf_file_exists(dbf_file.string());
}

/**
 * \brief returns index of a given field in a DBFHandle
 * \param handle DBFHandle
 * \param field_name field of DBFhandle
 * \return index of given field
 */

int dbf_get_field_index(DBFHandle handle, const char *field_name) {
  assert(handle);
  assert(field_name);
  // if (row >= DBFGetRecordCount(handle)) throw(std::runtime_error("row=" +
  // std::to_string(row) + " is out of bounds."));
  int index = DBFGetFieldIndex(handle, field_name);
  if (index == -1)
    throw(std::runtime_error("DBFfile doesnt contain " +
                             std::string(field_name)));
  return index;
}

/**
 * \brief get field of a given DBFHandle as string
 */
std::string dbf_get_string_by_field(DBFHandle handle, int row,
                                    const char *field_name) {
  return DBFReadStringAttribute(handle, row,
                                dbf_get_field_index(handle, field_name));
}

/**
 * \brief get field of a given DBFHandle as uint64_t
 */
uint64_t dbf_get_uint_by_field(DBFHandle handle, int row,
                               const char *field_name) {
  return DBFReadIntegerAttribute(handle, row,
                                 dbf_get_field_index(handle, field_name));
}

/* getting fields from OGRFeatures -- begin */

/**
 * \brief returns field from OGRFeature
 *        aborts if feature is nullpointer or field key is invalid
 * \param feat feature from which field is read
 * \param field field name as key
 * \return const char* of field value
 */
const char *get_field_from_feature(const OGRFeature *feat, const char *field) {
  int field_index = feat->GetFieldIndex(field);
  if (field_index == -1)
    std::cerr << field << std::endl;
  return feat->GetFieldAsString(field_index);
}

/**
 * \brief returns field from OGRFeature
 *        throws exception if field_value is not
 * \param feat feature from which field is read
 * \param field field name as key
 * \return field value as uint
 */
uint64_t get_uint_from_feature(const OGRFeature *feat, const char *field) {
  const char *value = get_field_from_feature(feat, field);
  assert(value);
  try {
    return std::stoul(value);
  } catch (const std::invalid_argument &) {
    throw format_error("Could not parse field='" + std::string(field) +
                       "' with value='" + std::string(value) + "'");
  }
}

/* getting fields from OGRFeatures -- end */

template <class Type> bool is_in_range(Type test, Type from, Type to) {
  if (test < from || test > to)
    return false;
  return true;
}

bool string_is_unsigned_integer(const std::string &s) {
  if (s.empty())
    return false;
  for (auto i : s)
    if (!isdigit(i))
      return false;
  return true;
}

bool string_is_not_unsigned_integer(const std::string &s) {
  return !string_is_unsigned_integer(s);
}

/**
 * \brief converts kilogram to tons
 */
template <class T> std::string kg_to_t(T kilo) {
  return std::to_string(kilo / 1000.0f);
}
/**
 * \brief converts centimeter to meters
 */
template <class T> std::string cm_to_m(T meter) {
  return std::to_string(meter / 100.0f);
}
/**
 * \brief converts inches to feet
 */
std::string inch_to_feet(unsigned int inches) {
  return std::to_string((unsigned int)floor(inches / INCH_BASE)) + "'" +
         std::to_string(inches % INCH_BASE) + "\"";
}

/**
 * \brief converts pounds to metric tons
 */
std::string lbs_to_metric_ton(double lbs) {
  double short_ton = lbs / (double)POUND_BASE;
  double metric_ton = short_ton * SHORT_TON;
  std::stringstream stream;
  stream << metric_ton;
  return stream.str();
}

/**
 * \brief duplicate const char* value to change
 */
std::string to_camel_case_with_spaces(const char *camel) {

  std::string titleString;
  icu::UnicodeString ustr(camel);
  ustr.toTitle(nullptr);
  ustr.toUTF8String(titleString);

  return titleString;
}

/**
 * \brief apply camel case with spaces to string
 */
std::string to_camel_case_with_spaces(const std::string &camel) {
  return to_camel_case_with_spaces(camel.c_str());
}

void add_uint_tag(osmium::builder::TagListBuilder *tl_builder,
                  const char *tag_key, uint uint_tag_val) {
  std::string val_s = std::to_string(uint_tag_val);
  if (tag_key) {
    tl_builder->add_tag(tag_key, val_s);
  }
}

bool checkInBoundingBox(const OGREnvelope &boundingBox,
                        const boost::filesystem::path &shp_file) {

  if (!boundingBox.IsInit())
    return true;

  auto ds = open_shape_file(shp_file);
  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(shp_file.string()));

  OGREnvelope layerEnvelop;

  if (layer->GetExtent(&layerEnvelop) != OGRERR_NONE) {
    return false;
  }

  bool result = false;
  if (boundingBox.Intersects(layerEnvelop)) {
    result = true;
  }

  return result;
}

#endif /* UTIL_HPP_ */
