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

#include "WaterConverter.hpp"

#include <boost/log/trivial.hpp>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/io/writer.hpp>
#include <osmium/memory/buffer.hpp>
#include <osmium/osm/types.hpp>

WaterConverter::WaterConverter(const std::filesystem::path &executable_path)
    : Converter(executable_path) {}

WaterConverter::~WaterConverter() {}

void WaterConverter::convert(const std::filesystem::path &dir,
                             osmium::io::Writer &writer) {

  const std::filesystem::path WATER_SEG_SHP = "WaterSeg.shp";
  const std::filesystem::path WATER_POLY_SHP = "WaterPoly.shp";

  add_water_shape(dir / WATER_POLY_SHP, writer);
  add_water_shape(dir / WATER_SEG_SHP, writer);
}

void WaterConverter::add_water_shape(std::filesystem::path water_shape_file,
                                     osmium::io::Writer &writer) {

  auto ds = openDataSource(water_shape_file);
  if (!ds) {
    return;
  }

  auto layer = ds->GetLayer(0);
  if (layer == nullptr) {
    throw(shp_empty_error(water_shape_file.string()));
  }

  assert(layer->GetGeomType() == wkbPolygon ||
         layer->GetGeomType() == wkbLineString);
  osmium::memory::Buffer node_buffer(BUFFER_SIZE);
  osmium::memory::Buffer way_buffer(BUFFER_SIZE);
  osmium::memory::Buffer rel_buffer(BUFFER_SIZE);

  std::map<osmium::Location, osmium::unsigned_object_id_type>
      g_way_end_points_map;
  for (const auto &feat : *layer) {
    process_water(feat, g_way_end_points_map, node_buffer, way_buffer,
                  rel_buffer);
  }
  writer(std::move(node_buffer));
  writer(std::move(way_buffer));
  writer(std::move(rel_buffer));
}

/**
 * \brief adds water polygons as Relations to m_buffer
 */
void WaterConverter::process_water(
    const OGRFeatureUniquePtr &feat,
    std::map<osmium::Location, osmium::unsigned_object_id_type>
        &g_way_end_points_map,
    osmium::memory::Buffer &node_buffer, osmium::memory::Buffer &way_buffer,
    osmium::memory::Buffer &rel_buffer) {
  auto geom = feat->GetGeometryRef();
  auto geom_type = geom->getGeometryType();

  if (geom_type == OGRwkbGeometryType::wkbLineString) {
    // handle water ways
    build_water_ways_with_tagList(feat, static_cast<OGRLineString *>(geom),
                                  g_way_end_points_map, node_buffer,
                                  way_buffer);
  } else {
    // handle water polygons
    std::vector<osmium::unsigned_object_id_type> exterior_way_ids;
    std::vector<osmium::unsigned_object_id_type> interior_way_ids;
    if (geom_type == OGRwkbGeometryType::wkbMultiPolygon) {
      create_multi_polygon(static_cast<OGRMultiPolygon *>(geom),
                           exterior_way_ids, interior_way_ids,
                           g_way_end_points_map, node_buffer, way_buffer);
    } else if (geom_type == OGRwkbGeometryType::wkbPolygon) {
      create_polygon(static_cast<OGRPolygon *>(geom), exterior_way_ids,
                     interior_way_ids, g_way_end_points_map, node_buffer,
                     way_buffer);
    } else {
      throw(std::runtime_error(
          "Water item with geometry=" + std::string(geom->getGeometryName()) +
          " is not yet supported."));
    }
    build_water_relation_with_tags(feat, exterior_way_ids, interior_way_ids,
                                   rel_buffer);
  }

  node_buffer.commit();
  way_buffer.commit();
  rel_buffer.commit();
}

std::vector<osmium::unsigned_object_id_type>
WaterConverter::build_water_ways_with_tagList(
    const OGRFeatureUniquePtr &feat, OGRLineString *line,
    std::map<osmium::Location, osmium::unsigned_object_id_type>
        &g_way_end_points_map,
    osmium::memory::Buffer &node_buffer, osmium::memory::Buffer &way_buffer) {

  auto osm_way_node_ids =
      create_open_way_nodes(line, g_way_end_points_map, node_buffer);

  std::vector<osmium::unsigned_object_id_type> osm_way_ids;
  size_t i = 0;
  do {
    osmium::builder::WayBuilder builder(way_buffer);
    setObjectProperties(builder);
    build_water_way_taglist(builder, feat);
    osmium::builder::WayNodeListBuilder wnl_builder(builder);
    for (size_t j = i;
         j < std::min(i + OSM_MAX_WAY_NODES, osm_way_node_ids.size()); j++) {
      const auto [location, osm_id] = osm_way_node_ids.at(j);
      wnl_builder.add_node_ref(osm_id, location);
    }
    osm_way_ids.push_back(builder.object().id());
    i += OSM_MAX_WAY_NODES - 1;
  } while (i < osm_way_node_ids.size());

  return osm_way_ids;
}

/**
 * \brief adds navteq water tags to way
 */
void WaterConverter::build_water_way_taglist(
    osmium::builder::WayBuilder &builder, const OGRFeatureUniquePtr &feat) {
  // Mind tl_builder scope!
  osmium::builder::TagListBuilder tl_builder(builder);
  //    tl_builder.add_tag("natural", "water");

  std::string polygonName = feat->GetFieldAsString(POLYGON_NM.data());
  if (!polygonName.empty()) {
    std::string waters_name = to_camel_case_with_spaces(polygonName);
    if (!waters_name.empty())
      tl_builder.add_tag("name", waters_name);
  }

  std::string featureCode = feat->GetFieldAsString(FEAT_COD.data());
  if (featureCode == "500412") {
    // FEAT_TYPE 'RIVER'
    tl_builder.add_tag("waterway", "river");
  } else if (featureCode == "500414") {
    // FEAT_TYPE 'CANAL/WATER CHANNEL'
    tl_builder.add_tag("waterway", "canal");
  } else if (featureCode == "500421") {
    // FEAT_TYPE 'LAKE'
    BOOST_LOG_TRIVIAL(error)
        << "Skipping water way as type LAKE should only exist as polygon";
  } else if (featureCode == "507116") {
    // FEAT_TYPE 'BAY/HARBOUR'
    BOOST_LOG_TRIVIAL(error)
        << "Skipping water way as type BAY/HARBOUR should only exist as "
           "polygon";
  } else {
    BOOST_LOG_TRIVIAL(error)
        << "Skipping unknown water way type " << featureCode;
  }
}

osmium::unsigned_object_id_type WaterConverter::build_water_relation_with_tags(
    const OGRFeatureUniquePtr &feat,
    std::vector<osmium::unsigned_object_id_type> ext_osm_way_ids,
    std::vector<osmium::unsigned_object_id_type> int_osm_way_ids,
    osmium::memory::Buffer &rel_buffer) {
  osmium::builder::RelationBuilder builder(rel_buffer);
  setObjectProperties(builder);
  build_water_poly_taglist(builder, feat);

  build_relation_members(builder, ext_osm_way_ids, int_osm_way_ids);
  return builder.object().id();
}

void WaterConverter::build_water_poly_taglist(
    osmium::builder::RelationBuilder &builder,
    const OGRFeatureUniquePtr &feat) {
  // Mind tl_builder scope!
  osmium::builder::TagListBuilder tl_builder(builder);
  tl_builder.add_tag("type", "multipolygon");
  tl_builder.add_tag("natural", "water");

  std::string polygonName = feat->GetFieldAsString(POLYGON_NM.data());
  if (!polygonName.empty()) {
    std::string waters_name = to_camel_case_with_spaces(polygonName);
    if (!waters_name.empty())
      tl_builder.add_tag("name", waters_name);
  }

  std::string featureCode = feat->GetFieldAsString(FEAT_COD.data());
  if (featureCode == "500412") {
    // FEAT_TYPE 'RIVER'
    tl_builder.add_tag("water", "river");
  } else if (featureCode == "500414") {
    // FEAT_TYPE 'CANAL/WATER CHANNEL'
    tl_builder.add_tag("water", "canal");
  } else if (featureCode == "500421") {
    // FEAT_TYPE 'LAKE'
    tl_builder.add_tag("water", "lake");
  } else if (featureCode == "507116") {
    // Type 'BAY/HARBOUR' just gets the 'natural=water' tag
  } else {
    BOOST_LOG_TRIVIAL(error)
        << "Skipping unknown water poly type " << featureCode;
  }
}