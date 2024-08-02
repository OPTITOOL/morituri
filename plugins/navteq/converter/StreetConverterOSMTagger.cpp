/*
 * This file is part of the Morituri project.
 * Morituri is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Morituri is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Morituri.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "StreetConverter.hpp"

#include <boost/log/trivial.hpp>
#include <format>
#include <ranges>

// modifier types (MT)
const ushort MT_TA_DIRECTION_CLOSURE = 38;
const ushort MT_TA_HAZARDOUS_RESTRICTION = 39;
const ushort MT_TA_HEIGHT_RESTRICTION = 41;
const ushort MT_TA_WEIGHT_RESTRICTION = 42;
const ushort MT_TA_WEIGHT_PER_AXLE_RESTRICTION = 43;
const ushort MT_TA_LENGTH_RESTRICTION = 44;
const ushort MT_TA_WIDTH_RESTRICTION = 45;

const ushort MT_SSS_SPEED_LIMIT = 48;
const ushort MT_SSS_SPEED_SITUATION_TYPE = 59;
const ushort MT_SSS_DIRECTION_CLOSURE = 60;
const ushort MT_SSS_WEIGHT_DEPENDENT = 61;
const ushort MT_SSS_WEATHER_TYPE = 62;
const ushort MT_SSS_SPEED_LIMIT_TYPE = 83;
const ushort MT_SSS_HAZARDOUS_DEPENDENT = 39;
const ushort MT_SSS_TRAILER_DEPENDENT = 46;

const int INCH_BASE = 12;
const int POUND_BASE = 2000;
// short ton in metric tons (source:
// http://wiki.openstreetmap.org/wiki/Key:maxweight)
const double SHORT_TON = 0.90718474;

static constexpr std::string_view MOTORWAY = "motorway";
static constexpr std::string_view MOTORWAY_LINK = "motorway_link";
static constexpr std::string_view TRUNK = "trunk";
static constexpr std::string_view PRIMARY = "primary";
static constexpr std::string_view PRIMARY_LINK = "primary_link";
static constexpr std::string_view SECONDARY = "secondary";
static constexpr std::string_view SECONDARY_LINK = "secondary_link";
static constexpr std::string_view TERTIARY = "tertiary";
static constexpr std::string_view UNCLASSIFIED = "unclassified";
static constexpr std::string_view RESIDENTIAL = "residential";
static constexpr std::string_view TRACK = "track";
static constexpr std::string_view PATH = "path";
static constexpr std::string_view FOOTWAY = "footway";

static constexpr std::string_view CONSTRUCTION = "construction";

// higway classification
const std::vector<std::string_view> DEFAULT_HWY_FUNC_TYPE = {
    "", PRIMARY, SECONDARY, SECONDARY, TERTIARY, UNCLASSIFIED, RESIDENTIAL};

std::map<int, std::vector<std::string_view>> const HWY_FUNC_CLASS_MAP = {
    /* Applied with functional classes:
     *                         1,       2,         3,        4,        5 +
     * rural,    5 + urban */
    {0 /*"DEFAULT"*/, DEFAULT_HWY_FUNC_TYPE},
    {3 /*"GER"*/,
     {"", PRIMARY, SECONDARY, TERTIARY, TERTIARY, UNCLASSIFIED, RESIDENTIAL}},
    {8 /*"CHE"*/,
     {"", PRIMARY, SECONDARY, TERTIARY, TERTIARY, UNCLASSIFIED, RESIDENTIAL}},
    {108 /*"DEN"*/,
     {"", PRIMARY, SECONDARY, SECONDARY, TERTIARY, UNCLASSIFIED, RESIDENTIAL}},
    {107 /*"SWE"*/,
     {"", PRIMARY, SECONDARY, SECONDARY, TERTIARY, UNCLASSIFIED, RESIDENTIAL}},
    {9 /*"AUT"*/,
     {"", PRIMARY, PRIMARY, SECONDARY, TERTIARY, UNCLASSIFIED, RESIDENTIAL}}};

static constexpr std::array<std::string_view, 9> speed_cat_metric{
    "", ">130", "101-130", "91-100", "71-90", "51-70", "31-50", "11-30", "<11"};

uint64_t StreetConverter::build_tag_list(OGRFeatureUniquePtr &feat,
                                         const TagData &data,
                                         osmium::builder::Builder &builder,
                                         short z_level) {
  osmium::builder::TagListBuilder tl_builder(builder);

  uint64_t link_id = parse_street_tags(tl_builder, feat, data.cdms_map,
                                       data.area_govt_map, data.cntry_map,
                                       data.route_type_map, data.highway_names);

  if (z_level != -5 && z_level != 0)
    tl_builder.add_tag("layer", std::to_string(z_level));
  if (link_id == 0)
    throw(format_error("layers column field '" + std::string(LINK_ID.data()) +
                       "' is missing"));
  return link_id;
}

uint64_t StreetConverter::parse_street_tags(
    osmium::builder::TagListBuilder &builder, OGRFeatureUniquePtr &f,
    const std::multimap<uint64_t, cond_type> &cdms_map,
    const std::map<uint64_t, uint64_t> &area_to_areacode1_map,
    const std::map<uint64_t, cntry_ref_type> &cntry_map,
    const std::map<uint64_t, ushort> &route_type_map,
    const std::map<uint64_t, std::map<uint, std::string>> &names_map) {

  cntry_ref_type cntry_ref = get_cntry_ref(f, area_to_areacode1_map, cntry_map);

  const char *link_id_s = get_field_from_feature(f, LINK_ID);
  uint64_t link_id = std::stoul(link_id_s);

  bool ramp = get_bool_from_feature(f, RAMP);

  ushort route_type = 0;
  if (!(std::string(get_field_from_feature(f, ROUTE)).empty()))
    route_type = get_uint_from_feature(f, ROUTE);

  auto routeTypeIter = route_type_map.find(link_id);
  if (routeTypeIter != route_type_map.end() &&
      (!route_type || routeTypeIter->second < route_type))
    route_type = routeTypeIter->second;

  if (is_ferry(get_field_from_feature(f, FERRY))) {
    add_ferry_tag(builder, f);
  } else { // usual highways
    // add tags for ref and int_ref to major highways
    std::string ref_name =
        add_highway_name_tags(builder, names_map, link_id, ramp);

    // get under construction condition
    auto itPair = cdms_map.equal_range(link_id);
    bool underConstruction = false;
    for (auto i = itPair.first; i != itPair.second; ++i) {
      if (i->second.cond_type_type == CT_CONSTRUCTION_STATUS_CLOSED) {
        underConstruction = true;
        break;
      }
    }

    add_highway_tags(builder, f, route_type, cntry_ref, ref_name,
                     underConstruction);
  }

  // tags which apply to highways and ferry routes
  add_additional_restrictions(builder, link_id, cdms_map, cntry_ref);

  // tag for debug purpose
  if (debugMode) {
    builder.add_tag(LINK_ID.data(), link_id_s);
    add_here_speed_cat_tag(builder, f);
    if (get_bool_from_feature(f, TOLLWAY))
      builder.add_tag("here:tollway", YES.data());
    if (get_bool_from_feature(f, URBAN))
      builder.add_tag("here:urban", YES.data());
    if (get_bool_from_feature(f, CONTRACC))
      builder.add_tag("here:controll_access", YES.data());
    if (route_type)
      add_uint_tag(builder, "here:route_type", route_type);

    std::string func_class = get_field_from_feature(f, FUNC_CLASS);
    if (!func_class.empty())
      builder.add_tag("here:func_class", func_class.c_str());

    add_uint_tag(builder, "here:area_code", cntry_ref.area_code_1);
  }
  return link_id;
}

/**
 * \brief converts pounds to metric tons
 */
std::string StreetConverter::lbs_to_metric_ton(double lbs) {
  double short_ton = lbs / (double)POUND_BASE;
  double metric_ton = short_ton * SHORT_TON;
  std::stringstream stream;
  stream << metric_ton;
  return stream.str();
}

bool StreetConverter::only_pedestrians(const OGRFeatureUniquePtr &f) {
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

void StreetConverter::add_lanes_tag(osmium::builder::TagListBuilder &builder,
                                    const OGRFeatureUniquePtr &f) {
  const char *number_of_physical_lanes = get_field_from_feature(f, PHYS_LANES);
  if (strcmp(number_of_physical_lanes, "0"))
    builder.add_tag("lanes", number_of_physical_lanes);
}

void StreetConverter::add_postcode_tag(osmium::builder::TagListBuilder &builder,
                                       const OGRFeatureUniquePtr &f) {
  std::string l_postcode = get_field_from_feature(f, L_POSTCODE);
  std::string r_postcode = get_field_from_feature(f, R_POSTCODE);

  if (l_postcode.empty() && r_postcode.empty())
    return;

  std::string postcode;
  if (l_postcode == r_postcode)
    postcode = l_postcode;
  else
    postcode = l_postcode + ";" + r_postcode;

  builder.add_tag("addr:postcode", postcode);
}

void StreetConverter::add_maxspeed_tags(
    osmium::builder::TagListBuilder &builder, const OGRFeatureUniquePtr &f,
    const cntry_ref_type &cntry_ref) {

  uint from_speed_limit = get_uint_from_feature(f, FR_SPEED_LIMIT);
  uint to_speed_limit = get_uint_from_feature(f, TO_SPEED_LIMIT);

  if (from_speed_limit >= 1000 || to_speed_limit >= 1000)
    throw(format_error(std::format(
        "from_speed_limit='{}' or to_speed_limit='{}' is not valid (>= 1000)",
        from_speed_limit, to_speed_limit)));

  // 998 is a ramp without speed limit information
  if (from_speed_limit == 998 || to_speed_limit == 998)
    return;

  bool mph = cntry_ref.speed_limit_unit == "MPH";

  // 999 means no speed limit at all
  std::string from = from_speed_limit == 999 ? "none"
                     : mph ? std::format("{} mph", from_speed_limit)
                           : std::to_string(from_speed_limit);
  std::string to = to_speed_limit == 999 ? "none"
                   : mph                 ? std::format("{} mph", to_speed_limit)
                                         : std::to_string(to_speed_limit);

  if (from_speed_limit != 0 && to_speed_limit != 0) {
    if (from_speed_limit != to_speed_limit) {
      builder.add_tag("maxspeed:forward", from);
      builder.add_tag("maxspeed:backward", to);
    } else {
      builder.add_tag("maxspeed", from);
    }
  } else if (from_speed_limit != 0 && to_speed_limit == 0) {
    builder.add_tag("maxspeed", from);
  } else if (from_speed_limit == 0 && to_speed_limit != 0) {
    builder.add_tag("maxspeed", to);
  }
}

void StreetConverter::add_ferry_tag(osmium::builder::TagListBuilder &builder,
                                    const OGRFeatureUniquePtr &f) {
  const char *ferry = get_field_from_feature(f, FERRY);
  builder.add_tag("route", "ferry");
  if (!strcmp(ferry, "B")) {
    if (only_pedestrians(f)) {
      builder.add_tag("foot", YES.data());
    } else {
      builder.add_tag("foot", get_bool_from_feature(f, AR_PEDESTRIANS)
                                  ? YES.data()
                                  : NO.data());
      builder.add_tag("motorcar", get_bool_from_feature(f, AR_AUTO)
                                      ? YES.data()
                                      : NO.data());
    }

  } else if (!strcmp(ferry, "R")) {
    builder.add_tag("railway", "ferry");
  } else
    throw(
        format_error(std::format("value '{}' for {} not valid", ferry, FERRY)));
}

void StreetConverter::add_highway_tags(osmium::builder::TagListBuilder &builder,
                                       const OGRFeatureUniquePtr &f,
                                       ushort route_type,
                                       const cntry_ref_type &cntry_ref,
                                       const std::string &ref_name,
                                       bool underConstruction) {

  ushort func_class = 0;
  std::string func_class_s = get_field_from_feature(f, FUNC_CLASS);
  if (!func_class_s.empty())
    func_class = get_uint_from_feature(f, FUNC_CLASS);

  add_highway_tag(builder, f, route_type, func_class, cntry_ref, ref_name,
                  underConstruction);

  add_one_way_tag(builder, get_field_from_feature(f, DIR_TRAVEL));
  add_access_tags(builder, f);
  add_maxspeed_tags(builder, f, cntry_ref);
  add_lanes_tag(builder, f);
  add_postcode_tag(builder, f);

  if (get_bool_from_feature(f, PAVED))
    builder.add_tag("surface", "paved");
  if (get_bool_from_feature(f, BRIDGE))
    builder.add_tag("bridge", YES.data());
  if (get_bool_from_feature(f, TUNNEL))
    builder.add_tag("tunnel", YES.data());
  if (get_bool_from_feature(f, TOLLWAY))
    builder.add_tag("toll", YES.data());
  if (get_bool_from_feature(f, ROUNDABOUT))
    builder.add_tag("junction", "roundabout");
  if (get_bool_from_feature(f, FOURWHLDR))
    builder.add_tag("4wd_only", YES.data());
}

void StreetConverter::add_one_way_tag(osmium::builder::TagListBuilder &builder,
                                      const char *value) {
  if (!strcmp(value, "F")) // F --> FROM reference node
    builder.add_tag("oneway", YES.data());
  else if (!strcmp(value, "T"))      // T --> TO reference node
    builder.add_tag("oneway", "-1"); // todo reverse way instead using "-1"
  else if (!strcmp(value, "B"))      // B --> BOTH ways are allowed
    return;
  else
    throw(format_error("value '" + std::string(value) +
                       "' for oneway not valid"));
}

void StreetConverter::add_access_tags(osmium::builder::TagListBuilder &builder,
                                      const OGRFeatureUniquePtr &f) {
  bool automobile_allowed = get_bool_from_feature(f, AR_AUTO);
  if (!automobile_allowed)
    builder.add_tag("motorcar", NO.data());
  if (!get_bool_from_feature(f, AR_BUS))
    builder.add_tag("bus", NO.data());
  if (!get_bool_from_feature(f, AR_TAXIS))
    builder.add_tag("taxi", NO.data());
  //    if (! parse_bool(get_field_from_feature(f, AR_CARPOOL)))
  //    builder->add_tag("hov",  NO);
  if (!get_bool_from_feature(f, AR_PEDESTRIANS))
    builder.add_tag("foot", NO.data());
  if (!get_bool_from_feature(f, AR_TRUCKS)) {
    // truck access handling:
    if (!get_bool_from_feature(f, AR_DELIV))
      builder.add_tag(
          "hgv",
          NO.data()); // no truck + no delivery => hgv not allowed at all
    else if (!automobile_allowed)
      builder.add_tag("access",
                      "delivery"); // no automobile + no truck but delivery
                                   // => general access is 'delivery'
    else if (automobile_allowed)
      builder.add_tag("hgv", "delivery"); // automobile generally allowed =>
                                          // only truck is 'delivery'
  }
  if (!get_bool_from_feature(f, AR_EMERVEH))
    builder.add_tag("emergency", NO.data());
  if (!get_bool_from_feature(f, AR_MOTORCYCLES))
    builder.add_tag("motorcycle", NO.data());
  if (!get_bool_from_feature(f, PUB_ACCESS) ||
      get_bool_from_feature(f, PRIVATE)) {
    builder.add_tag("access", "private");
  } else if (!get_bool_from_feature(f, AR_THROUGH_TRAFFIC)) {
    builder.add_tag("access", "destination");
  }
}

std::string StreetConverter::add_highway_name_tags(
    osmium::builder::TagListBuilder &builder,
    const std::map<uint64_t, std::map<uint, std::string>> &names_map,
    uint64_t link_id, bool ramp) {
  std::string ref_tag;

  auto it = names_map.find(link_id);
  if (it != names_map.end()) {
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
      builder.add_tag("name", to_camel_case_with_spaces(street_name));
    if (!ref_tag.empty()) // national ref (Autobahn)
      builder.add_tag("ref", ref_tag);
    if (!int_ref_tag.empty()) // international ref (European street)
      builder.add_tag("int_ref", int_ref_tag);
    if (!nat_ref_tag.empty()) // national ref (European street)
      builder.add_tag("nat_ref", nat_ref_tag);
  }

  return ref_tag;
}

/**
 * \brief converts inches to feet
 */
std::string StreetConverter::inch_to_feet(unsigned int inches) {
  return std::to_string((unsigned int)floor(inches / INCH_BASE)) + "'" +
         std::to_string(inches % INCH_BASE) + "\"";
}

void StreetConverter::add_here_speed_cat_tag(
    osmium::builder::TagListBuilder &builder, const OGRFeatureUniquePtr &f) {
  auto speed_cat = get_uint_from_feature(f, SPEED_CAT);
  if (0 < speed_cat && speed_cat < speed_cat_metric.size())
    builder.add_tag("here:speed_cat", speed_cat_metric[speed_cat].data());
  else
    throw format_error("SPEED_CAT=" + std::to_string(speed_cat) +
                       " is not valid.");
}

bool StreetConverter::is_motorized_allowed(const OGRFeatureUniquePtr &f) {
  if (get_bool_from_feature(f, AR_AUTO))
    return true;
  if (get_bool_from_feature(f, AR_TRUCKS))
    return true;
  if (get_bool_from_feature(f, AR_DELIV))
    return true;
  if (get_bool_from_feature(f, AR_EMERVEH))
    return true;
  if (get_bool_from_feature(f, AR_MOTORCYCLES))
    return true;
  if (get_bool_from_feature(f, AR_BUS))
    return true;
  if (get_bool_from_feature(f, AR_TAXIS))
    return true;

  return false;
}

bool StreetConverter::need_to_consider(const OGRFeatureUniquePtr &f) {
  return get_bool_from_feature(f, AR_AUTO) ||
         get_bool_from_feature(f, AR_TRUCKS) ||
         get_bool_from_feature(f, "AR_DELIVER") ||
         get_bool_from_feature(f, "AR_THRUTR");
}

bool StreetConverter::is_hgv_only(const OGRFeatureUniquePtr &f) {
  if (get_bool_from_feature(f, AR_AUTO))
    return false;

  return get_bool_from_feature(f, AR_TRUCKS) ||
         get_bool_from_feature(f, "AR_DELIVER");
}

void StreetConverter::add_hazmat_tag(osmium::builder::TagListBuilder &builder,
                                     uint64_t mod_val) {
  if (mod_val == 20) { // || mod_val == 21
    builder.add_tag("hazmat", "no");
  } else if (mod_val == 22) {
    builder.add_tag("hazmat:water", "no");
  } else if (mod_val == 24) {
    builder.add_tag("hazmat:B", "no");
  } else if (mod_val == 28) {
    builder.add_tag("hazmat:C", "no");
  } else if (mod_val == 32) {
    builder.add_tag("hazmat:D", "no");
  } else if (mod_val == 34) {
    builder.add_tag("hazmat:E", "no");
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
    BOOST_LOG_TRIVIAL(error) << "Hazardous material value " << mod_val
                             << " hasn't been parsed!" << std::endl;
  }
}

void StreetConverter::add_highway_tag(osmium::builder::TagListBuilder &builder,
                                      const OGRFeatureUniquePtr &f,
                                      ushort route_type, ushort func_class,
                                      const cntry_ref_type &cntry_ref,
                                      const std::string &ref_name,
                                      bool underConstruction) {

  bool paved = get_bool_from_feature(f, PAVED);
  bool motorized_allowed = is_motorized_allowed(f);

  std::string highwayTagName = HIGHWAY.data();

  if (underConstruction) {
    builder.add_tag(HIGHWAY.data(), CONSTRUCTION.data());
    highwayTagName = CONSTRUCTION.data();
  }

  if (!paved) {
    if (!motorized_allowed) {
      // unpaved + non-motorized => path
      builder.add_tag(highwayTagName, PATH.data());
    } else {
      // unpaved + motorized allowed => track
      builder.add_tag(highwayTagName, TRACK.data());
    }
  } else {
    if (!motorized_allowed) {
      // paved + non-motorized => footway
      // it seems imposref_nameible to distinguish footways from cycle ways or
      // pedestrian zones
      builder.add_tag(highwayTagName, FOOTWAY.data());
    } else {
      // paved + motorized allowed
      bool controlled_access = get_bool_from_feature(f, CONTRACC);
      bool urban = get_bool_from_feature(f, URBAN);
      bool ramp = get_bool_from_feature(f, RAMP);

      if (controlled_access) {
        // controlled_access => motorway
        if (ramp)
          builder.add_tag(highwayTagName, MOTORWAY_LINK.data());
        else
          builder.add_tag(highwayTagName, MOTORWAY.data());
      } else if (func_class || route_type) {
        std::string_view hwy_value = get_hwy_value(
            route_type, func_class, cntry_ref.area_code_1, ref_name, urban);
        if (!hwy_value.empty()) {
          builder.add_tag(highwayTagName, hwy_value.data());
        } else {
          BOOST_LOG_TRIVIAL(error)
              << "ignoring highway_level'" << std::to_string(route_type)
              << "' for " << cntry_ref.area_code_1 << std::endl;
        }
      } else {
        BOOST_LOG_TRIVIAL(error)
            << " highway misses route_type and func_class! " << std::endl;
      }
    }
  }
}

std::string_view StreetConverter::get_hwy_value(ushort route_type,
                                                ushort func_class,
                                                uint area_code_1,
                                                const std::string &ref_name,
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

  const auto &hwy_vector = HWY_FUNC_CLASS_MAP.contains(area_code_1)
                               ? HWY_FUNC_CLASS_MAP.at(area_code_1)
                               : DEFAULT_HWY_FUNC_TYPE;

  /* default case */
  return hwy_vector.at(apply_func_class);
}

void StreetConverter::add_additional_restrictions(
    osmium::builder::TagListBuilder &builder, uint64_t link_id,
    const std::multimap<uint64_t, cond_type> &cdms_map,
    const cntry_ref_type &cntry_ref) {
  if (cdms_map.empty())
    return;

  // default is metric units
  bool imperial_units = is_imperial(cntry_ref);

  auto range = cdms_map.equal_range(link_id);
  for (auto it = range.first; it != range.second; ++it) {

    if (it->second.dt_mod.hasDateTimeMod) {
      if (debugMode)
        BOOST_LOG_TRIVIAL(debug)
            << "Skip Condition because of DateTimeMod for link_id " << link_id
            << std::endl;
      continue;
    }

    if (it->second.cond_type_type == CT_TRANSPORT_ACCESS_RESTRICTION) {
      addTransportAccessRestriction(it, builder, link_id, cntry_ref,
                                    imperial_units);
    } else if (it->second.cond_type_type ==
               CT_TRANSPORT_SPECIAL_SPEED_SITUATION) {
      addSpecialSpeedSituation(it, link_id, cntry_ref, builder, imperial_units);
    }
  }
}

void StreetConverter::addSpecialSpeedSituation(
    std::multimap<uint64_t, StreetConverter::cond_type>::const_iterator &it,
    uint64_t link_id, const StreetConverter::cntry_ref_type &cntry_ref,
    osmium::builder::TagListBuilder &builder, bool imperial_units) {
  uint64_t speed_limit = 0;
  int direction = 0; // 1 = both, 2 = forward, 3 = backward
  int weight_dependent = 0;

  for (auto mod_group : it->second.mod_group_map | std::views::values) {
    auto mod_type = mod_group.mod_type;
    auto mod_val = mod_group.mod_val;
    if (mod_type == MT_SSS_SPEED_SITUATION_TYPE) {
      if (mod_val != 3) // Weight dependent only
        return;
    } else if (mod_type == MT_SSS_SPEED_LIMIT) {
      speed_limit = mod_val;
    } else if (mod_type == MT_SSS_DIRECTION_CLOSURE) {
      direction = mod_val;
    } else if (mod_type == MT_SSS_SPEED_LIMIT_TYPE) {
      if ((mod_val == 2)) // ignore advisory speed
        return;
    } else if (mod_type == MT_SSS_WEATHER_TYPE) {
      // ignore weather type
    } else if (mod_type == MT_SSS_HAZARDOUS_DEPENDENT) {
      // ignore hazardous dependent
    } else if (mod_type == MT_SSS_TRAILER_DEPENDENT) {
      // ignore trailer dependent
    } else if (mod_type == MT_SSS_WEIGHT_DEPENDENT) {
      weight_dependent = mod_val;
    } else {

      BOOST_LOG_TRIVIAL(debug)
          << "Unknown modifier type " << mod_type << " value " << mod_val
          << " for Condition Type " << it->second.cond_type_type
          << " for link_id " << link_id << std::endl;
    }
  }

  std::string key = "maxspeed";

  // hgvs have a different speed limit
  if (it->second.only_hgv)
    key += ":hgv";

  // handle direction
  if (direction == 1)
    key += ":forward";
  else if (direction == 2)
    key += ":backward";

  std::string convertedWeigthDepended;
  if (weight_dependent != 0) {
    // add conditional weight dependent speed limit
    convertedWeigthDepended = imperial_units
                                  ? lbs_to_metric_ton(weight_dependent)
                                  : kg_to_t(weight_dependent);
    key += ":conditional";
  }

  std::string speed_limit_str = cntry_ref.speed_limit_unit == "MPH"
                                    ? std::to_string(speed_limit) + " mph"
                                    : std::to_string(speed_limit);

  if (!convertedWeigthDepended.empty())
    speed_limit_str += " @ weight>" + convertedWeigthDepended;

  builder.add_tag(key, speed_limit_str);
}

void StreetConverter::addTransportAccessRestriction(
    std::multimap<uint64_t, StreetConverter::cond_type>::const_iterator &it,
    osmium::builder::TagListBuilder &builder, uint64_t link_id,
    const StreetConverter::cntry_ref_type &cntry_ref, bool imperial_units) {
  uint64_t max_height = 0;
  uint64_t max_width = 0;
  uint64_t max_length = 0;
  uint64_t max_weight = 0;
  uint64_t max_axleload = 0;
  int direction = 1; // 1 = both, 2 = forward, 3 = backward

  for (auto mod_group : it->second.mod_group_map | std::views::values) {
    auto mod_type = mod_group.mod_type;
    auto mod_val = mod_group.mod_val;
    if (mod_type == MT_TA_HEIGHT_RESTRICTION) {
      if (!max_height || mod_val < max_height)
        max_height = mod_val;
    } else if (mod_type == MT_TA_WIDTH_RESTRICTION) {
      if (!max_width || mod_val < max_width)
        max_width = mod_val;
    } else if (mod_type == MT_TA_LENGTH_RESTRICTION) {
      if (!max_length || mod_val < max_length)
        max_length = mod_val;
    } else if (mod_type == MT_TA_WEIGHT_RESTRICTION) {
      if (!max_weight || mod_val < max_weight)
        max_weight = mod_val;
    } else if (mod_type == MT_TA_WEIGHT_PER_AXLE_RESTRICTION) {
      if (!max_axleload || mod_val < max_axleload)
        max_axleload = mod_val;
    } else if (mod_type == MT_TA_HAZARDOUS_RESTRICTION) {
      add_hazmat_tag(builder, mod_val);
    } else if (mod_type == MT_TA_DIRECTION_CLOSURE) {
      // not supported
      // direction = mod_val;
    } else {
      if (debugMode)
        BOOST_LOG_TRIVIAL(debug)
            << "Unknown modifier type " << mod_type << " value " << mod_val
            << " for Condition Type " << it->second.cond_type_type
            << " for link_id " << link_id << std::endl;
    }
  }

  if (max_height > 0)
    addRestrictionTag(builder, "maxheight", direction, imperial_units,
                      max_height);
  if (max_width > 0)
    addRestrictionTag(builder, "maxwidth", direction, imperial_units,
                      max_width);
  if (max_length > 0)
    addRestrictionTag(builder, "maxlength", direction, imperial_units,
                      max_length);
  if (max_weight > 0)
    addRestrictionTag(builder, "maxweight", direction, imperial_units,
                      max_weight);
  if (max_axleload > 0)
    addRestrictionTag(builder, "maxaxleload", direction, imperial_units,
                      max_weight);
}

void StreetConverter::addRestrictionTag(
    osmium::builder::TagListBuilder &builder, const std::string &restriction,
    int direction, bool is_imperial_units, uint64_t max_value) {
  {
    std::string convertedValue =
        is_imperial_units ? inch_to_feet(max_value) : cm_to_m(max_value);

    if (direction == 2)
      builder.add_tag(restriction + ":forward", convertedValue);
    else if (direction == 3)
      builder.add_tag(restriction + ":backward", convertedValue);
    else
      builder.add_tag(restriction, convertedValue);
  }
}

uint StreetConverter::get_number_after(const std::string &str,
                                       const char *start_str) {
  if (!str.starts_with(start_str))
    return 0; /* doesn't start with start_str */

  /* Get number string after start_str until first
   * non-digit appears */
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

bool StreetConverter::is_imperial(const cntry_ref_type &cntry_ref) {
  auto unit_measure = cntry_ref.unit_measure;
  if (unit_measure == "E") {
    return true;
  } else if (unit_measure != "M") {
    format_error("unit_measure in navteq data is invalid: '" + unit_measure +
                 "'");
  }
  return false;
}

StreetConverter::cntry_ref_type StreetConverter::get_cntry_ref(
    const OGRFeatureUniquePtr &f,
    const std::map<uint64_t, uint64_t> &area_to_areacode1_map,
    const std::map<uint64_t, cntry_ref_type> &cntry_map) {
  uint64_t l_area_id = get_uint_from_feature(f, L_AREA_ID);
  if (auto l_area_iter = area_to_areacode1_map.find(l_area_id);
      l_area_iter != area_to_areacode1_map.end()) {
    if (auto l_cntry_ref = std::ranges::find_if(
            cntry_map,
            [l_area_iter](const auto &pair) {
              return pair.second.area_code_1 == l_area_iter->second;
            });
        l_cntry_ref != cntry_map.end())
      return l_cntry_ref->second;
  }

  uint64_t r_area_id = get_uint_from_feature(f, R_AREA_ID);
  if (auto r_area_iter = area_to_areacode1_map.find(r_area_id);
      r_area_iter != area_to_areacode1_map.end()) {
    if (auto r_cntry_ref = std::ranges::find_if(
            cntry_map,
            [r_area_iter](const auto &pair) {
              return pair.second.area_code_1 == r_area_iter->second;
            });
        r_cntry_ref != cntry_map.end())
      return r_cntry_ref->second;
  }

  throw format_error("cntry_ref not found for link_id " +
                     std::to_string(get_uint_from_feature(f, LINK_ID)));
}