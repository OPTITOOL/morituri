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

#include "RailwayConverter.hpp"

#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/io/writer.hpp>
#include <osmium/memory/buffer.hpp>
#include <osmium/osm/types.hpp>

#include "../../comm2osm_exceptions.hpp"
#include "../../util.hpp"

RailwayConverter::RailwayConverter(
    const boost::filesystem::path &executable_path)
    : Converter(executable_path) {}

RailwayConverter::~RailwayConverter() {}

void RailwayConverter::convert(const std::vector<boost::filesystem::path> &dirs,
                               osmium::io::Writer &writer) {

  const boost::filesystem::path RAILWAYS_POLY_SHP = "RailRds.shp";

  for (auto dir : dirs) {
    add_railways_shape(dir / RAILWAYS_POLY_SHP, writer);
  }
}

void RailwayConverter::add_railways_shape(
    boost::filesystem::path railway_shape_file, osmium::io::Writer &writer) {

  auto ds = GDALDatasetUniquePtr(GDALDataset::Open(railway_shape_file.c_str()));
  if (!ds) {
    BOOST_LOG_TRIVIAL(debug)
        << "No railway shp found in " << railway_shape_file;
    return;
  }
  auto layer = ds->GetLayer(0);
  if (!layer) {
    throw(shp_empty_error(railway_shape_file.string()));
  }
  assert(layer->GetGeomType() == OGRwkbGeometryType::wkbPolygon ||
         layer->GetGeomType() == OGRwkbGeometryType::wkbLineString);
  osmium::memory::Buffer node_buffer(BUFFER_SIZE);
  osmium::memory::Buffer way_buffer(BUFFER_SIZE);
  std::map<osmium::Location, osmium::unsigned_object_id_type>
      g_way_end_points_map;
  for (auto &feat : *layer) {
    process_railways(feat, g_way_end_points_map, node_buffer, way_buffer);
  }
  writer(std::move(node_buffer));
  writer(std::move(way_buffer));
}

void RailwayConverter::process_railways(
    const OGRFeatureUniquePtr &feat,
    std::map<osmium::Location, osmium::unsigned_object_id_type>
        &g_way_end_points_map,
    osmium::memory::Buffer &node_buffer, osmium::memory::Buffer &way_buffer) {
  auto osm_way_node_ids = create_open_way_nodes(
      static_cast<const OGRLineString *>(feat->GetGeometryRef()),
      g_way_end_points_map, node_buffer);
  {
    osmium::builder::WayBuilder builder(way_buffer);
    setObjectProperties(builder);
    {
      osmium::builder::TagListBuilder tl_builder(builder);
      tl_builder.add_tag("railway", "rail");
      tl_builder.add_tag("usage", "main");

      if (parse_bool(feat->GetFieldAsString(BRIDGE.data())))
        tl_builder.add_tag("bridge", YES.data());

      if (parse_bool(feat->GetFieldAsString(TUNNEL.data())))
        tl_builder.add_tag("tunnel", YES.data());
    }
    {
      osmium::builder::WayNodeListBuilder wnl_builder(builder);
      for (auto osm_way_node_id : osm_way_node_ids) {
        wnl_builder.add_node_ref(osm_way_node_id.second, osm_way_node_id.first);
      }
    }
  }
  node_buffer.commit();
  way_buffer.commit();
}