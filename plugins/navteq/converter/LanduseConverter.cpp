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

#include "LanduseConverter.hpp"

#include <boost/log/trivial.hpp>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/io/writer.hpp>
#include <osmium/memory/buffer.hpp>
#include <osmium/osm/location.hpp>
#include <osmium/osm/types.hpp>

#include "../../comm2osm_exceptions.hpp"
#include "../../util.hpp"

LanduseConverter::LanduseConverter() {}

LanduseConverter::~LanduseConverter() {}

void LanduseConverter::convert(const std::vector<boost::filesystem::path> &dirs,
                               osmium::io::Writer &writer) {
  const boost::filesystem::path LAND_USE_A_SHP = "LandUseA.shp";
  const boost::filesystem::path LAND_USE_B_SHP = "LandUseB.shp";

  for (auto dir : dirs) {
    add_landuse_shape(dir / LAND_USE_A_SHP, writer);
    add_landuse_shape(dir / LAND_USE_B_SHP, writer);
  }
}

void LanduseConverter::add_landuse_shape(
    boost::filesystem::path landuse_shape_file, osmium::io::Writer &writer) {
  std::map<osmium::Location, osmium::unsigned_object_id_type>
      g_way_end_points_map;
  auto ds = GDALDatasetUniquePtr(GDALDataset::Open(landuse_shape_file.c_str()));
  if (ds == nullptr) {
    BOOST_LOG_TRIVIAL(debug)
        << "No landuse shp found in " << landuse_shape_file;
    return;
  }
  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(landuse_shape_file.string()));
  assert(layer->GetGeomType() == wkbPolygon);
  osmium::memory::Buffer node_buffer(BUFFER_SIZE);
  osmium::memory::Buffer way_buffer(BUFFER_SIZE);
  osmium::memory::Buffer rel_buffer(BUFFER_SIZE);
  for (auto &feat : *layer) {
    process_landuse(feat, g_way_end_points_map, node_buffer, way_buffer,
                    rel_buffer);
  }

  writer(std::move(node_buffer));
  writer(std::move(way_buffer));
  writer(std::move(rel_buffer));
}

void LanduseConverter::process_landuse(
    const OGRFeatureUniquePtr &feat,
    std::map<osmium::Location, osmium::unsigned_object_id_type>
        &g_way_end_points_map,
    osmium::memory::Buffer &node_buffer, osmium::memory::Buffer &way_buffer,
    osmium::memory::Buffer &rel_buffer) {
  auto geom = feat->GetGeometryRef();
  auto geom_type = geom->getGeometryType();

  std::vector<osmium::unsigned_object_id_type> exterior_way_ids;
  std::vector<osmium::unsigned_object_id_type> interior_way_ids;
  if (geom_type == wkbMultiPolygon) {
    create_multi_polygon(static_cast<OGRMultiPolygon *>(geom), exterior_way_ids,
                         interior_way_ids, g_way_end_points_map, node_buffer,
                         way_buffer);
  } else if (geom_type == wkbPolygon) {
    create_polygon(static_cast<OGRPolygon *>(geom), exterior_way_ids,
                   interior_way_ids, g_way_end_points_map, node_buffer,
                   way_buffer);
  } else {
    throw(std::runtime_error(
        "Landuse item with geometry=" + std::string(geom->getGeometryName()) +
        " is not yet supported."));
  }

  build_landuse_relation_with_tags(feat, exterior_way_ids, interior_way_ids,
                                   rel_buffer);

  node_buffer.commit();
  way_buffer.commit();
  rel_buffer.commit();
}

osmium::unsigned_object_id_type
LanduseConverter::build_landuse_relation_with_tags(
    const OGRFeatureUniquePtr &feat,
    std::vector<osmium::unsigned_object_id_type> &ext_osm_way_ids,
    std::vector<osmium::unsigned_object_id_type> &int_osm_way_ids,
    osmium::memory::Buffer &rel_buffer) {
  osmium::builder::RelationBuilder builder(rel_buffer);
  setObjectProperties(builder);
  build_landuse_taglist(builder, feat);
  build_relation_members(builder, ext_osm_way_ids, int_osm_way_ids);
  return builder.object().id();
}

void LanduseConverter::build_landuse_taglist(
    osmium::builder::RelationBuilder &builder,
    const OGRFeatureUniquePtr &feat) {
  // Mind tl_builder scope!
  osmium::builder::TagListBuilder tl_builder(builder);
  tl_builder.add_tag("type", "multipolygon");

  std::string polygonName = feat->GetFieldAsString(POLYGON_NM.data());
  if (!polygonName.empty()) {
    std::string poly_name = to_camel_case_with_spaces(polygonName);
    if (!poly_name.empty())
      tl_builder.add_tag("name", poly_name);
  }

  std::string featureCode = feat->GetFieldAsString(FEAT_COD.data());

  // Land Use A types
  if (featureCode == "509998") {
    // FEAT_TYPE 'BEACH'
    tl_builder.add_tag("natural", "beach");
  } else if (featureCode == "900103") {
    // FEAT_TYPE 'PARK/MONUMENT (NATIONAL)'
    tl_builder.add_tag("boundary", "national_park");
  } else if (featureCode == "900130") {
    // FEAT_TYPE 'PARK (STATE)'
    // In many cases this is meant to be a national park or
    // protected area but this is not guaranteed
    tl_builder.add_tag("leisure", "park");
  } else if (featureCode == "900140") {
    // FEAT_TYPE 'PARK IN WATER'
    tl_builder.add_tag("boundary", "national_park");
  } else if (featureCode == "900150") {
    // FEAT_TYPE 'PARK (CITY/COUNTY)'
    tl_builder.add_tag("leisure", "park");
  } else if (featureCode == "900159") {
    // FEAT_TYPE 'UNDEFINED TRAFFIC AREA'
    // Possible handling: area=yes, highway=pedestrian
  } else if (featureCode == "900202") {
    // FEAT_TYPE 'WOODLAND'
    tl_builder.add_tag("landuse", "forest");
  } else if (featureCode == "1700215") {
    // FEAT_TYPE 'PARKING LOT'
    tl_builder.add_tag("amenity", "parking");
  } else if (featureCode == "1900403") {
    // FEAT_TYPE 'AIRPORT'
    tl_builder.add_tag("aeroway", "aerodrome");
  } else if (featureCode == "2000124") {
    // FEAT_TYPE 'SHOPPING CENTRE'
    tl_builder.add_tag("shop", "mall");
    tl_builder.add_tag("building", "retail");
  } else if (featureCode == "2000200") {
    // FEAT_TYPE 'INDUSTRIAL COMPLEX'
    tl_builder.add_tag("landuse", "commercial");
  } else if (featureCode == "2000403") {
    // FEAT_TYPE 'UNIVERSITY/COLLEGE'
    tl_builder.add_tag("amenity", "university");
  } else if (featureCode == "2000408") {
    // FEAT_TYPE 'HOSPITAL'
    tl_builder.add_tag("amenity", "hospital");
  } else if (featureCode == "2000420") {
    // FEAT_TYPE 'CEMETERY'
    tl_builder.add_tag("landuse", "cemetery");
  } else if (featureCode == "2000457") {
    // FEAT_TYPE 'SPORTS COMPLEX'
    tl_builder.add_tag("leisure", "stadium");
    // tl_builder.add_tag("building", "yes");
  } else if (featureCode == "2000460") {
    // FEAT_TYPE 'AMUSEMENT PARK'
    tl_builder.add_tag("leisure", "park");
    tl_builder.add_tag("tourism", "theme_park");
  } else if (featureCode == "908002") {
    // FEAT_TYPE 'Neighbourhood'
    tl_builder.add_tag("place", "suburb");
  } else if (featureCode == "2000461") {
    // FEAT_TYPE 'ANIMAL PARK'
    tl_builder.add_tag("tourism", "zoo");
  }

  // Not implemented so far due to missing sample in data:
  // MILITARY BASE (900108), NATIVE AMERICAN
  // RESERVATION (900107), RAILYARD (9997007)
  // Land Use B types
  else if (featureCode == "900158") {
    // FEAT_TYPE 'PEDESTRIAN ZONE'
    tl_builder.add_tag("highway", "pedestrian");
  } else if (featureCode == "1907403") {
    // FEAT_TYPE 'AIRCRAFT ROADS'
    tl_builder.add_tag("aeroway", "runway");
  } else if (featureCode == "2000123") {
    // FEAT_TYPE 'GOLF COURSE'
    tl_builder.add_tag("leisure", "golf_course");
    tl_builder.add_tag("sport", "golf");
  } else if (featureCode == "9997004") {
    // FEAT_TYPE 'CONGESTION ZONE'
    // skipping due to no osm equivalent
  } else if (featureCode == "9997010") {
    // FEAT_TYPE 'ENVIRONMENTAL ZONE'
    tl_builder.add_tag("boundary", "low_emission_zone");
    tl_builder.add_tag("type", "boundary");
  }
  // Unknown if Land Use A or B types, but seems to appear somewhere
  else if (featureCode == "509997") {
    // FEAT_TYPE 'GLACIER'
    tl_builder.add_tag("natural", "glacier");
  } else {
    BOOST_LOG_TRIVIAL(error) << "Skipping unknown landuse type " << featureCode;
  }
}
