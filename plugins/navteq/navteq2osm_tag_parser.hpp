#ifndef NAVTEQ2OSMTAGPARSE_HPP_
#define NAVTEQ2OSMTAGPARSE_HPP_

#include <fstream>
#include <iostream>

int ctr = 0;

bool fits_street_ref(const std::string &st_name) {
  if (st_name.empty())
    return false;
  if (st_name.size() > 6)
    return false;

  bool number_started = false;
  for (auto it = st_name.begin(); it != st_name.end(); ++it) {
    if (std::isdigit(*it)) {
      number_started = true;
    } else if (number_started) {
      return false;
    }
  }

  return number_started;
}

/**
 * \brief adds maxspeed tag
 */

/**
 * \brief adds here:speed_cat tag
 */
void add_here_speed_cat_tag(osmium::builder::TagListBuilder *builder,
                            OGRFeatureUniquePtr &f) {
  auto speed_cat = get_uint_from_feature(f, SPEED_CAT);
  if (0 < speed_cat &&
      speed_cat < (sizeof(speed_cat_metric) / sizeof(const char *)))
    builder->add_tag("here:speed_cat", speed_cat_metric[speed_cat]);
  else
    throw format_error("SPEED_CAT=" + std::to_string(speed_cat) +
                       " is not valid.");
}

/**
 * \brief adds maxheight, maxwidth, maxlength, maxweight and maxaxleload tags.
 */

void add_ferry_tag(osmium::builder::TagListBuilder *builder,
                   OGRFeatureUniquePtr &f) {
  const char *ferry = get_field_from_feature(f, FERRY);
  builder->add_tag("route", "ferry");
  if (!strcmp(ferry, "B")) {
    if (only_pedestrians(f)) {
      builder->add_tag("foot", YES);
    } else {
      builder->add_tag(
          "foot",
          parse_bool(get_field_from_feature(f, AR_PEDESTRIANS)) ? YES : NO);
      builder->add_tag("motorcar",
                       parse_bool(get_field_from_feature(f, AR_AUTO)) ? YES
                                                                      : NO);
    }

  } else if (!strcmp(ferry, "R")) {
    builder->add_tag("railway", "ferry");
  } else
    throw(format_error("value '" + std::string(ferry) + "' for " +
                       std::string(FERRY) + " not valid"));
}

std::string navteq_2_osm_admin_lvl(uint navteq_admin_lvl_int) {
  if (!is_in_range(navteq_admin_lvl_int, (uint)NAVTEQ_ADMIN_LVL_MIN,
                   (uint)NAVTEQ_ADMIN_LVL_MAX))
    throw std::runtime_error("invalid admin level. admin level '" +
                             std::to_string(navteq_admin_lvl_int) +
                             "' is out of range.");

  return std::to_string(2 * navteq_admin_lvl_int);
}

std::string navteq_2_osm_admin_lvl(std::string navteq_admin_lvl) {
  if (string_is_not_unsigned_integer(navteq_admin_lvl))
    throw std::runtime_error("admin level contains invalid character");

  return navteq_2_osm_admin_lvl(stoi(navteq_admin_lvl));
}

const char *parse_house_number_schema(const char *schema) {
  if (!strcmp(schema, "E"))
    return "even";
  if (!strcmp(schema, "O"))
    return "odd";
  std::cerr << "schema = " << schema << " unsupported" << std::endl;
  return "";
  throw std::runtime_error("scheme " + std::string(schema) +
                           " is currently not supported");
}

#endif /* NAVTEQ2OSMTAGPARSE_HPP_ */
