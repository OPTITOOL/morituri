#ifndef NAVTEQ2OSMTAGPARSE_HPP_
#define NAVTEQ2OSMTAGPARSE_HPP_

#include <fstream>
#include <iostream>

#include "navteq_mappings.hpp"
#include "navteq_types.hpp"
#include "navteq_util.hpp"

boost::filesystem::path g_executable_path;
const boost::filesystem::path
    PLUGINS_NAVTEQ_ISO_639_2_UTF_8_TXT("plugins/navteq/ISO-639-2_utf-8.txt");
int ctr = 0;

// helper
bool parse_bool(const char *value) {
  if (!strcmp(value, "Y"))
    return true;
  return false;
}

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

bool begins_with(const std::string &str, const char *start_str) {
  return str.rfind(start_str) == 0;
}

uint get_number_after(const std::string &str, const char *start_str) {
  if (!begins_with(str, start_str))
    return 0; /* doesn't start with start_str */

  /* Get number string after start_str until first non-digit appears */
  std::string end_str = str.substr(strlen(start_str));
  std::string number_str;
  for (auto it = end_str.begin(); it != end_str.end(); ++it) {
    if (std::isdigit(*it)) {
      number_str += *it;
    } else {
      /* break because B107a should return 107*/
      break;
    }
  }

  try {
    return std::stoul(number_str);
  } catch (const std::invalid_argument &) {
    return 0;
  }
}

bool is_motorized_allowed(OGRFeature *f) {
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

uint get_area_code_l(area_id_type l_area_id, area_id_type r_area_id,
                     mtd_area_map_type *mtd_area_map) {

  auto l_area = mtd_area_map->find(l_area_id);
  if (l_area != mtd_area_map->end())
    return l_area->second.area_code_1;

  auto r_area = mtd_area_map->find(r_area_id);
  if (r_area != mtd_area_map->end())
    return r_area->second.area_code_1;

  throw(out_of_range_exception("could not find area_id " +
                               std::to_string(++ctr) + ", " +
                               std::to_string(mtd_area_map->size())));
}

uint get_area_code_l(OGRFeature *f, mtd_area_map_type *mtd_area_map) {
  area_id_type l_area_id = get_uint_from_feature(f, L_AREA_ID);
  area_id_type r_area_id = get_uint_from_feature(f, R_AREA_ID);

  return get_area_code_l(l_area_id, r_area_id, mtd_area_map);
}

std::vector<std::string>
get_hwy_vector(const std::map<int, std::vector<std::string>> &HWY_TYPE_MAP,
               uint area_code_1) {
  auto it = HWY_TYPE_MAP.find(area_code_1);
  if (it != HWY_TYPE_MAP.end()) {
    return it->second;
  } else {
    //		std::cerr << "could not find area_id " << area_code_1 << " use
    // default" << std::endl;
    return DEFAULT_HWY_ROUTE_TYPE;
  }
}

std::string get_hwy_value(ushort route_type, ushort func_class,
                          uint area_code_1, const std::string &ref_name,
                          bool urban, bool ramp) {
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
      // Type 4 streets named B### are primary otherwise secondary
      if (!begins_with(ref_name, "B"))
        return SECONDARY;
    } else if (route_type == 5) {
      // Type 5 streets named L### are secondary otherwise tertiary
      if (begins_with(ref_name, "L"))
        return SECONDARY;
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
    }
  }

  uint apply_func_class = func_class;
  if (apply_func_class > 4 && urban) {
    apply_func_class++;
  } else if (func_class == 2 && route_type == 3) {
    apply_func_class = 1; // primary
  }

  /* default case */
  return get_hwy_vector(HWY_FUNC_CLASS_MAP, area_code_1).at(apply_func_class);
}

void add_highway_tag(osmium::builder::TagListBuilder *builder, OGRFeature *f,
                     ushort route_type, ushort func_class,
                     mtd_area_map_type *mtd_area_map,
                     const std::string &ref_name) {

  bool paved = parse_bool(get_field_from_feature(f, PAVED));
  bool motorized_allowed = is_motorized_allowed(f);

  if (!paved) {
    if (!motorized_allowed) {
      // unpaved + non-motorized => path
      builder->add_tag(HIGHWAY, PATH);
    } else {
      // unpaved + motorized allowed => track
      builder->add_tag(HIGHWAY, TRACK);
    }
  } else {
    if (!motorized_allowed) {
      // paved + non-motorized => footway
      // it seems imposref_nameible to distinguish footways from cycle ways or
      // pedestrian zones
      builder->add_tag(HIGHWAY, FOOTWAY);
    } else {
      // paved + motorized allowed
      bool controlled_access = parse_bool(get_field_from_feature(f, CONTRACC));
      bool urban = parse_bool(get_field_from_feature(f, URBAN));
      bool ramp = parse_bool(get_field_from_feature(f, RAMP));
      uint area_code_1 = get_area_code_l(f, mtd_area_map);
      if (controlled_access) {
        // controlled_access => motorway
        if (ramp)
          builder->add_tag(HIGHWAY, MOTORWAY_LINK);
        else
          builder->add_tag(HIGHWAY, MOTORWAY);
      } else if (func_class || route_type) {
        std::string hwy_value = get_hwy_value(
            route_type, func_class, area_code_1, ref_name, urban, ramp);
        if (!hwy_value.empty()) {
          builder->add_tag(HIGHWAY, hwy_value);
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

void add_access_tags(osmium::builder::TagListBuilder *builder, OGRFeature *f) {
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
                       OGRFeature *f) {
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
                            OGRFeature *f) {
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
void add_additional_restrictions(
    osmium::builder::TagListBuilder *builder, link_id_type link_id,
    area_id_type l_area_id, area_id_type r_area_id, cdms_map_type *cdms_map,
    cnd_mod_map_type *cnd_mod_map, area_id_govt_code_map_type *area_govt_map,
    cntry_ref_map_type *cntry_map, mtd_area_map_type *mtd_area_map) {
  if (!cdms_map || !cnd_mod_map)
    return;

  // default is metric units
  bool imperial_units = false;
  if (area_govt_map && cntry_map) {
    imperial_units =
        is_imperial(l_area_id, r_area_id, area_govt_map, cntry_map);
  }

  uint64_t max_height = 0, max_width = 0, max_length = 0, max_weight = 0,
           max_axleload = 0;

  std::vector<mod_group_type> mod_group_vector;
  auto range = cdms_map->equal_range(link_id);
  for (auto it = range.first; it != range.second; ++it) {
    cond_pair_type cond = it->second;
    if (cond.second == CT_RESTRICTED_DRIVING_MANOEUVRE ||
        cond.second == CT_TRANSPORT_RESTRICTED_DRIVING_MANOEUVRE)
      continue; // TODO RESTRICTED_DRIVING_MANOEUVRE should apply as
                // conditional turn restriction but not for current link id
    auto it2 = cnd_mod_map->find(cond.first);
    if (it2 != cnd_mod_map->end()) {
      for (auto mod_group : it2->second) {
        mod_group_vector.push_back(mod_group);
      }
    }
  }

  for (auto mod_group : mod_group_vector) {
    auto mod_type = mod_group.mod_type;
    auto mod_val = mod_group.mod_val;
    if (mod_type == MT_HEIGHT_RESTRICTION) {
      if (!max_height || mod_val < max_height)
        max_height = mod_val;
    } else if (mod_type == MT_WIDTH_RESTRICTION) {
      if (!max_width || mod_val < max_width)
        max_width = mod_val;
    } else if (mod_type == MT_LENGTH_RESTRICTION) {
      if (!max_length || mod_val < max_length)
        max_length = mod_val;
    } else if (mod_type == MT_WEIGHT_RESTRICTION) {
      if (!max_weight || mod_val < max_weight)
        max_weight = mod_val;
    } else if (mod_type == MT_WEIGHT_PER_AXLE_RESTRICTION) {
      if (!max_axleload || mod_val < max_axleload)
        max_axleload = mod_val;
    } else if (mod_type == MT_HAZARDOUS_RESTRICTION) {
      add_hazmat_tag(builder, mod_val);
    }
  }

  if (get_area_code_l(l_area_id, r_area_id, mtd_area_map) == 107) {
    /** exceptional handling for Sweden as there are BK Roads
     *
     * HERE tags these roads with the most conservative values,
     * which would make it unroutable for nearly every truck.
     * Therefore we use the highest value and add a marker for BK2 / BK3 */
    if (max_weight == 16000 && max_axleload == 10000) {
      builder->add_tag("maxweight:class", "BK2");
      max_weight = 51400;
    } else if (max_weight == 12000 && max_axleload == 8000) {
      builder->add_tag("maxweight:class", "BK3");
      max_weight = 37000;
    }
  }

  if (max_height > 0)
    builder->add_tag("maxheight", imperial_units ? inch_to_feet(max_height)
                                                 : cm_to_m(max_height));
  if (max_width > 0)
    builder->add_tag("maxwidth", imperial_units ? inch_to_feet(max_width)
                                                : cm_to_m(max_width));
  if (max_length > 0)
    builder->add_tag("maxlength", imperial_units ? inch_to_feet(max_length)
                                                 : cm_to_m(max_length));
  if (max_weight > 0)
    builder->add_tag("maxweight", imperial_units ? lbs_to_metric_ton(max_weight)
                                                 : kg_to_t(max_weight));
  if (max_axleload > 0)
    builder->add_tag("maxaxleload", imperial_units
                                        ? lbs_to_metric_ton(max_axleload)
                                        : kg_to_t(max_axleload));
}

bool is_ferry(const char *value) {
  if (!strcmp(value, "H"))
    return false; // H --> not a ferry
  else if (!strcmp(value, "B"))
    return true; // T --> boat ferry
  else if (!strcmp(value, "R"))
    return true; // B --> rail ferry
  throw(format_error("value '" + std::string(value) + "' for " +
                     std::string(FERRY) + " not valid"));
}

bool only_pedestrians(OGRFeature *f) {
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

void add_ferry_tag(osmium::builder::TagListBuilder *builder, OGRFeature *f) {
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

void add_lanes_tag(osmium::builder::TagListBuilder *builder, OGRFeature *f) {
  const char *number_of_physical_lanes = get_field_from_feature(f, PHYS_LANES);
  if (strcmp(number_of_physical_lanes, "0"))
    builder->add_tag("lanes", number_of_physical_lanes);
}

void add_postcode_tag(osmium::builder::TagListBuilder *builder, OGRFeature *f) {
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

    for (auto highwayName : highway_names_vector) {
      if (highwayName.first == 0) {
        street_name = highwayName.second;
      } else if (highwayName.first == 1) {
        int_ref_tag = highwayName.second;
      } else {
        ref_tag = highwayName.second;
      }
    }

    if (!street_name.empty() && !ramp)
      builder->add_tag("name", to_camel_case_with_spaces(street_name));
    if (!ref_tag.empty()) // national ref (Autobahn)
      builder->add_tag("ref", ref_tag);
    if (!int_ref_tag.empty()) // international ref (European street)
      builder->add_tag("int_ref", int_ref_tag);
  }

  return ref_tag;
}

void add_highway_tags(osmium::builder::TagListBuilder *builder, OGRFeature *f,
                      ushort route_type, mtd_area_map_type *mtd_area_map,
                      const std::string &ref_name) {

  ushort func_class = 0;
  std::string func_class_s = get_field_from_feature(f, FUNC_CLASS);
  if (!func_class_s.empty())
    func_class = get_uint_from_feature(f, FUNC_CLASS);

  add_highway_tag(builder, f, route_type, func_class, mtd_area_map, ref_name);

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

/**
 * \brief maps navteq tags for access, tunnel, bridge, etc. to osm tags
 * \return link id of processed feature.
 */
link_id_type parse_street_tags(osmium::builder::TagListBuilder *builder,
                               OGRFeature *f, cdms_map_type *cdms_map,
                               cnd_mod_map_type *cnd_mod_map,
                               area_id_govt_code_map_type *area_govt_map,
                               cntry_ref_map_type *cntry_map,
                               mtd_area_map_type *mtd_area_map,
                               link_id_route_type_map *route_type_map,
                               link_id_to_names_map *names_map) {

  const char *link_id_s = get_field_from_feature(f, LINK_ID);
  link_id_type link_id = std::stoul(link_id_s);
  builder->add_tag(LINK_ID, link_id_s); // tag for debug purpose

  bool ramp = parse_bool(get_field_from_feature(f, RAMP));

  ushort route_type = 0;
  if (!((std::string)get_field_from_feature(f, ROUTE)).empty())
    route_type = get_uint_from_feature(f, ROUTE);

  auto routeTypeIter = route_type_map->find(link_id);
  if (routeTypeIter != route_type_map->end() &&
      (!route_type || routeTypeIter->second < route_type))
    route_type = routeTypeIter->second;

  // add tags for ref and int_ref to major highways
  std::string ref_name =
      add_highway_name_tags(builder, names_map, link_id, ramp);

  if (is_ferry(get_field_from_feature(f, FERRY))) {
    add_ferry_tag(builder, f);
  } else { // usual highways
    add_highway_tags(builder, f, route_type, mtd_area_map, ref_name);
  }

  area_id_type l_area_id = get_uint_from_feature(f, L_AREA_ID);
  area_id_type r_area_id = get_uint_from_feature(f, R_AREA_ID);
  // tags which apply to highways and ferry routes
  add_additional_restrictions(builder, link_id, l_area_id, r_area_id, cdms_map,
                              cnd_mod_map, area_govt_map, cntry_map,
                              mtd_area_map);
  add_here_speed_cat_tag(builder, f);
  if (parse_bool(get_field_from_feature(f, TOLLWAY)))
    builder->add_tag("here:tollway", YES);
  if (parse_bool(get_field_from_feature(f, URBAN)))
    builder->add_tag("here:urban", YES);
  if (parse_bool(get_field_from_feature(f, CONTRACC)))
    builder->add_tag("here:controll_access", YES);
  if (route_type)
    add_uint_tag(builder, "here:route_type", route_type);

  std::string func_class = get_field_from_feature(f, FUNC_CLASS);
  if (!func_class.empty())
    builder->add_tag("here:func_class", func_class.c_str());

  add_uint_tag(builder, "here:area_code", get_area_code_l(f, mtd_area_map));

  return link_id;
}

const char *get_place_value(uint population, uint capital) {
  if (capital == 1 || capital == 2 || population > 100000)
    return "city";

  if (capital == 3 || capital == 4 || population > 10000)
    return "town";

  if (capital == 5 || population > 100)
    return "village";

  if (population > 0)
    return "hamlet";

  // population = 0 is more often a village than a hamlet
  return "village";
}

// matching from http://www.loc.gov/standards/iso639-2/php/code_list.php
// http://www.loc.gov/standards/iso639-2/ISO-639-2_utf-8.txt
// ISO-639 conversion
std::map<std::string, std::string> g_lang_code_map;
void parse_lang_code_file() {
  if (g_executable_path.empty())
    throw(std::runtime_error("executable_path is empty"));

  boost::filesystem::path iso_file(g_executable_path /
                                   PLUGINS_NAVTEQ_ISO_639_2_UTF_8_TXT);
  std::ifstream file(iso_file.string());
  assert(file.is_open());
  std::string line;
  std::string delim = "|";
  if (file.is_open()) {
    while (getline(file, line, '\n')) {
      std::vector<std::string> lv;
      auto start = 0u;
      auto end = line.find(delim);
      while (end != std::string::npos) {
        lv.push_back(line.substr(start, end - start));
        start = end + delim.length();
        end = line.find(delim, start);
      }
      std::string iso_639_2 = lv.at(0);
      std::string iso_639_1 = lv.at(2);
      g_lang_code_map.insert(std::make_pair(iso_639_2, iso_639_1));
    }
    file.close();
  }
}

std::string parse_lang_code(std::string lang_code) {
  std::transform(lang_code.begin(), lang_code.end(), lang_code.begin(),
                 ::tolower);
  if (g_lang_code_map.empty())
    parse_lang_code_file();

  auto lc = g_lang_code_map.find(lang_code);
  if (lc != g_lang_code_map.end()) {
    if (!lc->second.empty())
      return lc->second;
    else
      return lang_code; // fallback
  }
  std::cerr << lang_code << " not found!" << std::endl;
  throw std::runtime_error("Language code '" + lang_code + "' not found");
}

std::string navteq_2_osm_admin_lvl(uint navteq_admin_lvl_int) {
  if (!is_in_range(navteq_admin_lvl_int, (uint)NAVTEQ_ADMIN_LVL_MIN,
                   (uint)NAVTEQ_ADMIN_LVL_MAX))
    throw std::runtime_error("invalid admin level. admin level '" +
                             std::to_string(navteq_admin_lvl_int) +
                             "' is out of range.");

  return std::to_string(2 * navteq_admin_lvl_int).c_str();
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
