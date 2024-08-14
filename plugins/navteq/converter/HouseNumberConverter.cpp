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

#include "HouseNumberConverter.hpp"

#include <boost/log/trivial.hpp>
#include <osmium/io/writer.hpp>

#include "../../ogr_util.hpp"

HouseNumberConverter::HouseNumberConverter(
    const std::filesystem::path &executable_path)
    : Converter(executable_path) {}

HouseNumberConverter::~HouseNumberConverter() {}

void HouseNumberConverter::convert(const std::filesystem::path &dir,
                                   osmium::io::Writer &writer) {

  process_house_numbers(dir, writer);
}

void HouseNumberConverter::process_house_numbers(
    const std::filesystem::path &dir, osmium::io::Writer &writer) {

  // create point addresses from PointAddress.dbf
  auto pointMap = createPointAddressMapList(dir);

  auto path = dir / STREETS_SHP;
  auto ds = openDataSource(path);
  if (!ds)
    throw(shp_error(path.string()));

  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(path.string()));

  osmium::memory::Buffer node_buffer(BUFFER_SIZE);
  osmium::memory::Buffer way_buffer(BUFFER_SIZE);

  int linkIdField = layer->FindFieldIndex(LINK_ID.data(), true);

  for (auto &feat : *layer) {
    int linkId = feat->GetFieldAsInteger(linkIdField);
    process_house_numbers(feat, *pointMap, linkId, node_buffer, way_buffer);
  }

  node_buffer.commit();
  way_buffer.commit();
  {
    std::lock_guard<std::mutex> lock(osmiumWriterMutex);
    writer(std::move(node_buffer));
    writer(std::move(way_buffer));
  }
  delete pointMap;
}

/**
 * \brief creates Way from linestring.
 * 		  creates missing Nodes needed for Way and Way itself.
 * \param ogr_ls linestring which provides the geometry.
 * \param z_level_map holds z_levels to Nodes of Ways.
 */
void HouseNumberConverter::process_house_numbers(
    const OGRFeatureUniquePtr &feat,
    const std::map<uint64_t,
                   std::vector<std::pair<osmium::Location, std::string>>>
        &pointAddresses,
    int linkId, osmium::memory::Buffer &node_buffer,
    osmium::memory::Buffer &way_buffer) {

  auto ogr_ls = static_cast<const OGRLineString *>(feat->GetGeometryRef());

  auto it = pointAddresses.find(linkId);
  if (it != pointAddresses.end()) {
    create_premium_house_numbers(feat, it->second, linkId, node_buffer);
  } else {
    if (!strcmp(get_field_from_feature(feat, ADDR_TYPE), "B")) {
      create_house_numbers(feat, ogr_ls, node_buffer, way_buffer);
    }
  }
}

std::map<uint64_t, std::vector<std::pair<osmium::Location, std::string>>> *
HouseNumberConverter::createPointAddressMapList(
    const std::filesystem::path &dir) {

  auto pointAddressMap =
      new std::map<uint64_t,
                   std::vector<std::pair<osmium::Location, std::string>>>();

  static const std::filesystem::path POINT_ADDRESS_SHP = "PointAddress.shp";

  auto ds = openDataSource(dir / POINT_ADDRESS_SHP);
  if (!ds)
    return pointAddressMap;

  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error((dir / POINT_ADDRESS_SHP).string()));

  int linkIdField = layer->FindFieldIndex(LINK_ID.data(), true);
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

  return pointAddressMap;
}

void HouseNumberConverter::create_house_numbers(
    const OGRFeatureUniquePtr &feat, const OGRLineString *ogr_ls,
    osmium::memory::Buffer &node_buffer, osmium::memory::Buffer &way_buffer) {
  create_house_numbers(feat, ogr_ls, true, node_buffer, way_buffer);
  create_house_numbers(feat, ogr_ls, false, node_buffer, way_buffer);
}

void HouseNumberConverter::create_premium_house_numbers(
    const OGRFeatureUniquePtr &feat,
    const std::vector<std::pair<osmium::Location, std::string>> &addressList,
    int linkId, osmium::memory::Buffer &node_buffer) {

  for (auto &[location, houseNo] : addressList) {

    // scope node_builder
    osmium::builder::NodeBuilder node_builder(node_buffer);
    build_node(location, node_builder);
    {
      // scope tl_builder
      osmium::builder::TagListBuilder tl_builder(node_buffer, &node_builder);
      tl_builder.add_tag(LINK_ID.data(), std::to_string(linkId));
      tl_builder.add_tag("addr:housenumber", houseNo);
      tl_builder.add_tag(
          "addr:street",
          to_camel_case_with_spaces(get_field_from_feature(feat, ST_NAME)));
    }
  }
}

void HouseNumberConverter::create_house_numbers(
    const OGRFeatureUniquePtr &feat, const OGRLineString *ogr_ls, bool left,
    osmium::memory::Buffer &node_buffer, osmium::memory::Buffer &way_buffer) {
  const std::string_view &ref_addr = left ? L_REFADDR : R_REFADDR;
  const std::string_view &nref_addr = left ? L_NREFADDR : R_NREFADDR;
  const std::string_view &addr_schema = left ? L_ADDRSCH : R_ADDRSCH;

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
      build_node(location, node_builder);
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
    setObjectProperties(way_builder);
    {
      // scope wnl_builder
      osmium::builder::WayNodeListBuilder wnl_builder(way_buffer, &way_builder);

      for (int i = 0; i < offset_ogr_ls->getNumPoints(); i++) {
        osmium::Location location(offset_ogr_ls->getX(i),
                                  offset_ogr_ls->getY(i));
        {
          // scope node_builder
          osmium::builder::NodeBuilder node_builder(node_buffer);
          auto node_id = build_node(location, node_builder);
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

const char *
HouseNumberConverter::parse_house_number_schema(const char *schema) {
  if (!strcmp(schema, "E"))
    return "even";
  if (!strcmp(schema, "O"))
    return "odd";
  BOOST_LOG_TRIVIAL(error) << "schema = " << schema << " unsupported"
                           << std::endl;
  return "";
  throw std::runtime_error("scheme " + std::string(schema) +
                           " is currently not supported");
}