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

#include "BuildingConverter.hpp"

#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>
#include <ogrsf_frmts.h>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/io/writer.hpp>
#include <osmium/memory/buffer.hpp>
#include <osmium/osm/types.hpp>

BuildingConverter::BuildingConverter(
    const std::filesystem::path &executable_path)
    : Converter(executable_path) {}

BuildingConverter::~BuildingConverter() {}

void BuildingConverter::convert(const std::vector<std::filesystem::path> &dirs,
                                osmium::io::Writer &writer) {

  static const std::filesystem::path LANDMARK_SHP = "Landmark.shp";

  for (auto dir : dirs) {
    add_building_shape(dir / LANDMARK_SHP, writer);
  }
}

void BuildingConverter::add_building_shape(
    std::filesystem::path landmark_shape_file, osmium::io::Writer &writer) {

  auto ds = openDataSource(landmark_shape_file);
  if (!ds) {
    return;
  }

  auto layer = ds->GetLayer(0);
  if (!layer) {
    throw(shp_empty_error(landmark_shape_file));
  }

  assert(layer->GetGeomType() == wkbPolygon ||
         layer->GetGeomType() == wkbLineString);
  osmium::memory::Buffer node_buffer(BUFFER_SIZE);
  osmium::memory::Buffer way_buffer(BUFFER_SIZE);

  std::map<osmium::Location, osmium::unsigned_object_id_type>
      g_way_end_points_map;
  for (auto &feat : *layer) {
    if (!strcmp(feat->GetFieldAsString(FEAT_COD.data()), "2005999")) {
      process_building(feat, g_way_end_points_map, node_buffer, way_buffer);
    }
  }
  writer(std::move(node_buffer));
  writer(std::move(way_buffer));
}

void BuildingConverter::process_building(
    const OGRFeatureUniquePtr &feat,
    std::map<osmium::Location, osmium::unsigned_object_id_type>
        &g_way_end_points_map,
    osmium::memory::Buffer &node_buffer, osmium::memory::Buffer &way_buffer) {
  auto geom = feat->GetGeometryRef();

  auto osm_way_node_ids = create_closed_way_nodes(
      static_cast<OGRPolygon *>(geom)->getExteriorRing(), g_way_end_points_map,
      node_buffer);

  std::vector<osmium::unsigned_object_id_type> osm_way_ids;
  size_t i = 0;
  do {
    osmium::builder::WayBuilder builder(way_buffer);
    setObjectProperties(builder);
    {
      osmium::builder::WayNodeListBuilder wnl_builder(builder);
      for (size_t j = i;
           j < std::min(i + OSM_MAX_WAY_NODES, osm_way_node_ids.size()); j++) {
        const auto [location, osm_id] = osm_way_node_ids.at(j);
        wnl_builder.add_node_ref(osm_id, location);
      }
    }
    osm_way_ids.push_back(builder.object().id());
    i += OSM_MAX_WAY_NODES - 1;
    build_building_poly_taglist(builder, feat);
  } while (i < osm_way_node_ids.size());
  node_buffer.commit();
  way_buffer.commit();
}

void BuildingConverter::build_building_poly_taglist(
    osmium::builder::WayBuilder &builder, const OGRFeatureUniquePtr &feat) {
  // Mind tl_builder scope!
  osmium::builder::TagListBuilder tl_builder(builder);
  tl_builder.add_tag("building", "yes");

  std::string name = feat->GetFieldAsString(POLYGON_NM.data());
  std::string building_name = to_camel_case_with_spaces(name);
  if (!building_name.empty())
    tl_builder.add_tag("name", building_name);
}