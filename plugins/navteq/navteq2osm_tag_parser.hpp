#ifndef NAVTEQ2OSMTAGPARSE_HPP_
#define NAVTEQ2OSMTAGPARSE_HPP_

#include <fstream>
#include <iostream>

#include "navteq_mappings.hpp"
#include "navteq_types.hpp"
#include "navteq_util.hpp"

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

uint get_number_after(const std::string &str, const char *start_str) {
  if (!str.starts_with(start_str))
    return 0; /* doesn't start with start_str */

  /* Get number string after start_str until first non-digit appears */
  std::string end_str = str.substr(strlen(start_str));
  std::string number_str;
  for (auto it = end_str.begin(); it != end_str.end(); ++it) {
    if (!std::isdigit(*it)) {
      /* break because B107a should return 107*/
      break;
    }
    number_str += *it;
  }

  try {
    return std::stoul(number_str);
  } catch (const std::invalid_argument &) {
    return 0;
  }
}

bool is_motorized_allowed(OGRFeatureUniquePtr &f) {
  if (parse_bool(get_field_from_feature(f, AR_AUTO)))
    return true;
  if (parse_bool(get_field_from_feature(f, AR_BUS)))
    return true;
  if (parse_bool(get_field_from_feature(f, AR_TAXIS)))
    return true;
  if (parse_bool(get_field_from_feature(f, AR_TRUCKS)))
    return true;
  if (parse_bool(get_field_from_feature(f, AR_DELIV)))
    return true;
  if (parse_bool(get_field_from_feature(f, AR_EMERVEH)))
    return true;
  if (parse_bool(get_field_from_feature(f, AR_MOTORCYCLES)))
    return true;

  return false;
}

std::vector<std::string>
get_hwy_vector(const std::map<int, std::vector<std::string>> &HWY_TYPE_MAP,
               uint area_code_1) {
  auto it = HWY_TYPE_MAP.find(area_code_1);
  if (it != HWY_TYPE_MAP.end()) {
    return it->second;
  } else {
    return DEFAULT_HWY_FUNC_TYPE;
  }
}

std::string get_hwy_value(ushort route_type, ushort func_class,
                          uint area_code_1, const std::string &ref_name,
                          bool urban) {
  /* some exceptional cases for better route type parsing */
  if (area_code_1 == 2 && route_type == 4) { /*"FRA"*/
    /* Too many different highways have route type 4
     * so we also take functional class into consideration */
    if (func_class == 2)
      return PRIMARY;
    else if (func_class == 3)
      return SECONDARY;
    else if (func_class > 3)
      return TERTIARY;
  } else if (area_code_1 == 5 && route_type == 3) { /*"BEL"*/
    /* N# and N## is like PRIMARY
     * N### SECONDARY
     * N#### TERTIARY */
    uint hwy_num = get_number_after(ref_name, "N");
    if (hwy_num > 999)
      return TERTIARY;
    if (hwy_num > 99)
      return SECONDARY;
  } else if (area_code_1 == 9) { /*"AUT"*/
    if (route_type == 4) {
      return PRIMARY; // bundesstrasse
    } else if (route_type == 5) {
      return SECONDARY; // landesstrasse
    }
  } else if (area_code_1 == 23 && route_type == 2) { /*"IRL"*/
    /* N## is TRUNK if ## smaller 50 otherwise PRIMARY */
    uint hwy_num = get_number_after(ref_name, "N");
    if (hwy_num > 0 && hwy_num < 50)
      return TRUNK;
  } else if (area_code_1 == 109 ||
             area_code_1 == 110 /*UK - Wales*/ /*UK - England*/
             || area_code_1 == 112 ||
             area_code_1 == 22) { /*UK - Scotland*/ /*UK - Northern Ireland*/
    /* Differ between white and green shield A-Roads */
    if (route_type == 2) {
      if (func_class == 2 || func_class == 1)
        return TRUNK;
      else
        return PRIMARY;
    } else if (route_type == 3 && func_class == 4) {
      return SECONDARY;
    }
  } else if (area_code_1 == 107) { // SWE
    if (func_class == 1 && route_type == 1)
      return TRUNK; // func_class 1 und not controlled
  }

  uint apply_func_class = func_class;
  if (apply_func_class > 4 && urban) {
    apply_func_class++;
  } else if (func_class == 2 && (route_type == 3 || route_type == 2)) {
    return PRIMARY; // primary
  }

  /* default case */
  return get_hwy_vector(HWY_FUNC_CLASS_MAP, area_code_1).at(apply_func_class);
}

void add_highway_tag(osmium::builder::TagListBuilder *builder,
                     OGRFeatureUniquePtr &f, ushort route_type,
                     ushort func_class, mtd_area_map_type *mtd_area_map,
                     const std::string &ref_name, bool underConstruction) {

  bool paved = parse_bool(get_field_from_feature(f, PAVED));
  bool motorized_allowed = is_motorized_allowed(f);

  std::string highwayTagName = HIGHWAY;

  if (underConstruction) {
    builder->add_tag(HIGHWAY, CONSTRUCTION);
    highwayTagName = CONSTRUCTION;
  }

  if (!paved) {
    if (!motorized_allowed) {
      // unpaved + non-motorized => path
      builder->add_tag(highwayTagName, PATH);
    } else {
      // unpaved + motorized allowed => track
      builder->add_tag(highwayTagName, TRACK);
    }
  } else {
    if (!motorized_allowed) {
      // paved + non-motorized => footway
      // it seems imposref_nameible to distinguish footways from cycle ways or
      // pedestrian zones
      builder->add_tag(highwayTagName, FOOTWAY);
    } else {
      // paved + motorized allowed
      bool controlled_access = parse_bool(get_field_from_feature(f, CONTRACC));
      bool urban = parse_bool(get_field_from_feature(f, URBAN));
      bool ramp = parse_bool(get_field_from_feature(f, RAMP));
      uint area_code_1 = get_area_code_l(f, mtd_area_map);

      if (controlled_access) {
        // controlled_access => motorway
        if (ramp)
          builder->add_tag(highwayTagName, MOTORWAY_LINK);
        else
          builder->add_tag(highwayTagName, MOTORWAY);
      } else if (func_class || route_type) {
        std::string hwy_value =
            get_hwy_value(route_type, func_class, area_code_1, ref_name, urban);
        if (!hwy_value.empty()) {
          builder->add_tag(highwayTagName, hwy_value);
        } else {
          std::cerr << "ignoring highway_level'" << std::to_string(route_type)
                    << "' for " << area_code_1 << std::endl;
        }
      } else {
        std::cerr << " highway misses route_type and func_class! " << std::endl;
      }
    }
  }
}

const char *parse_one_way_tag(const char *value) {
  if (!strcmp(value, "F")) // F --> FROM reference node
    return YES;
  else if (!strcmp(value, "T")) // T --> TO reference node
    return "-1";                // todo reverse way instead using "-1"
  else if (!strcmp(value, "B")) // B --> BOTH ways are allowed
    return nullptr;
  throw(
      format_error("value '" + std::string(value) + "' for oneway not valid"));
}

void add_one_way_tag(osmium::builder::TagListBuilder *builder,
                     const char *value) {
  const char *one_way = "oneway";
  const char *parsed_value = parse_one_way_tag(value);
  if (parsed_value)
    builder->add_tag(one_way, parsed_value);
}

void add_access_tags(osmium::builder::TagListBuilder *builder,
                     OGRFeatureUniquePtr &f) {
  bool automobile_allowed = parse_bool(get_field_from_feature(f, AR_AUTO));
  if (!automobile_allowed)
    builder->add_tag("motorcar", NO);
  if (!parse_bool(get_field_from_feature(f, AR_BUS)))
    builder->add_tag("bus", NO);
  if (!parse_bool(get_field_from_feature(f, AR_TAXIS)))
    builder->add_tag("taxi", NO);
  //    if (! parse_bool(get_field_from_feature(f, AR_CARPOOL)))
  //    builder->add_tag("hov",  NO);
  if (!parse_bool(get_field_from_feature(f, AR_PEDESTRIANS)))
    builder->add_tag("foot", NO);
  if (!parse_bool(get_field_from_feature(f, AR_TRUCKS))) {
    // truck access handling:
    if (!parse_bool(get_field_from_feature(f, AR_DELIV)))
      builder->add_tag("hgv",
                       NO); // no truck + no delivery => hgv not allowed at all
    else if (!automobile_allowed)
      builder->add_tag("access",
                       "delivery"); // no automobile + no truck but delivery
                                    // => general access is 'delivery'
    else if (automobile_allowed)
      builder->add_tag("hgv", "delivery"); // automobile generally allowed =>
                                           // only truck is 'delivery'
  }
  if (!parse_bool(get_field_from_feature(f, AR_EMERVEH)))
    builder->add_tag("emergency", NO);
  if (!parse_bool(get_field_from_feature(f, AR_MOTORCYCLES)))
    builder->add_tag("motorcycle", NO);
  if (!parse_bool(get_field_from_feature(f, PUB_ACCESS)) ||
      parse_bool(get_field_from_feature(f, PRIVATE))) {
    builder->add_tag("access", "private");
  } else if (!parse_bool(get_field_from_feature(f, AR_THROUGH_TRAFFIC))) {
    builder->add_tag("access", "destination");
  }
}

/**
 * \brief adds maxspeed tag
 */
void add_maxspeed_tags(osmium::builder::TagListBuilder *builder,
                       OGRFeatureUniquePtr &f) {
  char *from_speed_limit_s = strdup(get_field_from_feature(f, FR_SPEED_LIMIT));
  char *to_speed_limit_s = strdup(get_field_from_feature(f, TO_SPEED_LIMIT));

  uint from_speed_limit = get_uint_from_feature(f, FR_SPEED_LIMIT);
  uint to_speed_limit = get_uint_from_feature(f, TO_SPEED_LIMIT);

  if (from_speed_limit >= 1000 || to_speed_limit >= 1000)
    throw(format_error("from_speed_limit='" + std::string(from_speed_limit_s) +
                       "' or to_speed_limit='" + std::string(to_speed_limit_s) +
                       "' is not valid (>= 1000)"));

  // 998 is a ramp without speed limit information
  if (from_speed_limit == 998 || to_speed_limit == 998)
    return;

  // 999 means no speed limit at all
  const char *from = from_speed_limit == 999 ? "none" : from_speed_limit_s;
  const char *to = to_speed_limit == 999 ? "none" : to_speed_limit_s;

  if (from_speed_limit != 0 && to_speed_limit != 0) {
    if (from_speed_limit != to_speed_limit) {
      builder->add_tag("maxspeed:forward", from);
      builder->add_tag("maxspeed:backward", to);
    } else {
      builder->add_tag("maxspeed", from);
    }
  } else if (from_speed_limit != 0 && to_speed_limit == 0) {
    builder->add_tag("maxspeed", from);
  } else if (from_speed_limit == 0 && to_speed_limit != 0) {
    builder->add_tag("maxspeed", to);
  }

  free(from_speed_limit_s);
  free(to_speed_limit_s);
}

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
 * \brief check if unit is imperial. area_id(Streets.dbf) ->
 * govt_id(MtdArea.dbf) -> unit_measure(MtdCntryRef.dbf) \param area_id
 * area_id \param area_govt_map maps area_ids to govt_codes \param cntry_map
 * maps govt_codes to cntry_ref_types \return returns false if any of the
 * areas contain metric units or if its unclear
 */
bool is_imperial(area_id_type area_id,
                 area_id_govt_code_map_type *area_govt_map,
                 cntry_ref_map_type *cntry_map) {
  if (area_govt_map->find(area_id) != area_govt_map->end()) {
    if (cntry_map->find(area_govt_map->at(area_id)) != cntry_map->end()) {
      auto unit_measure =
          cntry_map->at(area_govt_map->at(area_id)).unit_measure;
      if (unit_measure == "E") {
        return true;
      } else if (unit_measure != "M") {
        format_error("unit_measure in navteq data is invalid: '" +
                     unit_measure + "'");
      }
    }
  }
  return false;
}

/**
 * \brief check if unit is imperial. area_id(Streets.dbf) ->
 * govt_id(MtdArea.dbf) -> unit_measure(MtdCntryRef.dbf) \param l_area_id
 * area_id on the left side of the link \param r_area_id area_id on the right
 * side of the link \param area_govt_map maps area_ids to govt_codes \param
 * cntry_map maps govt_codes to cntry_ref_types \return returns false if any
 * of the areas contain metric units or if its unclear
 */
bool is_imperial(area_id_type l_area_id, area_id_type r_area_id,
                 area_id_govt_code_map_type *area_govt_map,
                 cntry_ref_map_type *cntry_map) {
  if (is_imperial(l_area_id, area_govt_map, cntry_map))
    return true;
  if (is_imperial(r_area_id, area_govt_map, cntry_map))
    return true;
  return false;
}

void add_hazmat_tag(osmium::builder::TagListBuilder *builder,
                    mod_val_type mod_val) {
  if (mod_val == 20) { // || mod_val == 21
    builder->add_tag("hazmat", "no");
  } else if (mod_val == 22) {
    builder->add_tag("hazmat:water", "no");
  } else if (mod_val == 24) {
    builder->add_tag("hazmat:B", "no");
  } else if (mod_val == 28) {
    builder->add_tag("hazmat:C", "no");
  } else if (mod_val == 32) {
    builder->add_tag("hazmat:D", "no");
  } else if (mod_val == 34) {
    builder->add_tag("hazmat:E", "no");
  } else if (mod_val == 23) {
    /* 23 = Explosive and Flammable */
  } else {
    /**
     * Do nothing for the residual values,
     * which do not occur and/or have no proper OSM tagging equivalent
     * 1 = Explosives
     * 2 = Gas
     * 3 = Flammable
     * 4 = Flammable solid/Combustible
     * 5 = Organic
     * 6 = Poison
     * 7 = Radioactive
     * 8 = Corrosive
     * 9 = Other
     */
    std::cerr << "Hazardous material value " << mod_val
              << " hasn't been parsed!" << std::endl;
  }
}

/**
 * \brief adds maxheight, maxwidth, maxlength, maxweight and maxaxleload tags.
 */

bool only_pedestrians(OGRFeatureUniquePtr &f) {
  if (strcmp(get_field_from_feature(f, AR_PEDESTRIANS), "Y"))
    return false;
  if (!strcmp(get_field_from_feature(f, AR_AUTO), "Y"))
    return false;
  if (!strcmp(get_field_from_feature(f, AR_BUS), "Y"))
    return false;
  //    if (! strcmp(get_field_from_feature(f, AR_CARPOOL),"Y")) return false;
  if (!strcmp(get_field_from_feature(f, AR_EMERVEH), "Y"))
    return false;
  if (!strcmp(get_field_from_feature(f, AR_MOTORCYCLES), "Y"))
    return false;
  if (!strcmp(get_field_from_feature(f, AR_TAXIS), "Y"))
    return false;
  if (!strcmp(get_field_from_feature(f, AR_THROUGH_TRAFFIC), "Y"))
    return false;
  return true;
}

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

void add_lanes_tag(osmium::builder::TagListBuilder *builder,
                   OGRFeatureUniquePtr &f) {
  const char *number_of_physical_lanes = get_field_from_feature(f, PHYS_LANES);
  if (strcmp(number_of_physical_lanes, "0"))
    builder->add_tag("lanes", number_of_physical_lanes);
}

void add_postcode_tag(osmium::builder::TagListBuilder *builder,
                      OGRFeatureUniquePtr &f) {
  std::string l_postcode = get_field_from_feature(f, L_POSTCODE);
  std::string r_postcode = get_field_from_feature(f, R_POSTCODE);

  if (l_postcode.empty() && r_postcode.empty())
    return;

  std::string postcode;
  if (l_postcode == r_postcode)
    postcode = l_postcode;
  else
    postcode = l_postcode + ";" + r_postcode;

  builder->add_tag("addr:postcode", postcode);
}

std::string add_highway_name_tags(osmium::builder::TagListBuilder *builder,
                                  link_id_to_names_map *names_map,
                                  link_id_type link_id, bool ramp) {
  std::string ref_tag;

  auto it = names_map->find(link_id);
  if (it != names_map->end()) {
    auto &highway_names_vector = it->second;
    std::string street_name;
    std::string int_ref_tag;
    std::string nat_ref_tag;

    for (auto highwayName : highway_names_vector) {
      if (highwayName.first == 0) {
        street_name = highwayName.second;
      } else if (highwayName.first == 1) {
        int_ref_tag = highwayName.second;
      } else if (highwayName.first > 1) {
        nat_ref_tag = highwayName.second;
      }
    }

    if (!nat_ref_tag.empty())
      ref_tag = nat_ref_tag;
    else
      ref_tag = int_ref_tag;

    if (!street_name.empty() && !ramp)
      builder->add_tag("name", to_camel_case_with_spaces(street_name));
    if (!ref_tag.empty()) // national ref (Autobahn)
      builder->add_tag("ref", ref_tag);
    if (!int_ref_tag.empty()) // international ref (European street)
      builder->add_tag("int_ref", int_ref_tag);
    if (!nat_ref_tag.empty()) // national ref (European street)
      builder->add_tag("nat_ref", nat_ref_tag);
  }

  return ref_tag;
}

void add_highway_tags(osmium::builder::TagListBuilder *builder,
                      OGRFeatureUniquePtr &f, ushort route_type,
                      mtd_area_map_type *mtd_area_map,
                      const std::string &ref_name, bool underConstruction) {

  ushort func_class = 0;
  std::string func_class_s = get_field_from_feature(f, FUNC_CLASS);
  if (!func_class_s.empty())
    func_class = get_uint_from_feature(f, FUNC_CLASS);

  add_highway_tag(builder, f, route_type, func_class, mtd_area_map, ref_name,
                  underConstruction);

  add_one_way_tag(builder, get_field_from_feature(f, DIR_TRAVEL));
  add_access_tags(builder, f);
  add_maxspeed_tags(builder, f);
  add_lanes_tag(builder, f);
  add_postcode_tag(builder, f);

  if (parse_bool(get_field_from_feature(f, PAVED)))
    builder->add_tag("surface", "paved");
  if (parse_bool(get_field_from_feature(f, BRIDGE)))
    builder->add_tag("bridge", YES);
  if (parse_bool(get_field_from_feature(f, TUNNEL)))
    builder->add_tag("tunnel", YES);
  if (parse_bool(get_field_from_feature(f, TOLLWAY)))
    builder->add_tag("toll", YES);
  if (parse_bool(get_field_from_feature(f, ROUNDABOUT)))
    builder->add_tag("junction", "roundabout");
  if (parse_bool(get_field_from_feature(f, FOURWHLDR)))
    builder->add_tag("4wd_only", YES);
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
