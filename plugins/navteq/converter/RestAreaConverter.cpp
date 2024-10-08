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

#include "RestAreaConverter.hpp"

#include <boost/log/trivial.hpp>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/io/writer.hpp>
#include <osmium/memory/buffer.hpp>
#include <osmium/osm/location.hpp>
#include <osmium/osm/types.hpp>

RestAreaConverter::RestAreaConverter(
    const std::filesystem::path &executable_path)
    : Converter(executable_path) {}

RestAreaConverter::~RestAreaConverter() {}

void RestAreaConverter::convert(const std::filesystem::path &dir,
                                osmium::io::Writer &writer) {

  const std::filesystem::path TRAVDEST_SHP = "TravDest.shp";

  add_rest_area(dir / TRAVDEST_SHP, writer);
}

void RestAreaConverter::add_rest_area(std::filesystem::path rest_area_file,
                                      osmium::io::Writer &writer) {

  auto ds = openDataSource(rest_area_file);
  if (!ds) {
    return;
  }
  auto layer = ds->GetLayer(0);
  if (!layer) {
    throw(shp_empty_error(rest_area_file.string()));
  }

  osmium::memory::Buffer node_buffer(BUFFER_SIZE);

  int facTypeField = layer->FindFieldIndex(FAC_TYPE.data(), true);
  int poiNmTypeField = layer->FindFieldIndex(POI_NMTYPE.data(), true);

  for (auto &feat : *layer) {
    uint fac_type = feat->GetFieldAsInteger(facTypeField);
    if (fac_type != 7897) {
      continue;
    }

    std::string name_type = feat->GetFieldAsString(poiNmTypeField);
    if (name_type != "B") {
      // Skip this entry as it's just a translated namePlc of former one
      continue;
    }
    process_rest_area(feat, node_buffer);
  }
  {
    std::lock_guard<std::mutex> lock(osmiumWriterMutex);
    writer(std::move(node_buffer));
  }
}

void RestAreaConverter::process_rest_area(const OGRFeatureUniquePtr &feat,
                                          osmium::memory::Buffer &node_buffer) {

  auto geom = feat->GetGeometryRef();
  auto geom_type = geom->getGeometryType();

  if (geom_type != wkbPoint) {
    throw(std::runtime_error(
        "Rest area item with geometry=" + std::string(geom->getGeometryName()) +
        " is not yet supported."));
  }

  auto point = static_cast<OGRPoint *>(geom);
  osmium::Location location(point->getX(), point->getY());
  {
    // scope node_builder
    // Add new node
    osmium::builder::NodeBuilder node_builder(node_buffer);
    build_node(location, node_builder);
    osmium::builder::TagListBuilder tl_builder(node_builder);

    std::string name = feat->GetFieldAsString(POI_NAME.data());
    tl_builder.add_tag("name", to_camel_case_with_spaces(name));
    tl_builder.add_tag("highway", "rest_area");
  }
  node_buffer.commit();
}