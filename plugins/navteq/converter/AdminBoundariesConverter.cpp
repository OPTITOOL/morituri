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

#include "AdminBoundariesConverter.hpp"

#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>
#include <ogrsf_frmts.h>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/io/writer.hpp>
#include <osmium/memory/buffer.hpp>
#include <osmium/osm/types.hpp>
#include <ranges>

#include "../../comm2osm_exceptions.hpp"
#include "../../util.hpp"

AdminBoundariesConverter::AdminBoundariesConverter() {}

AdminBoundariesConverter::~AdminBoundariesConverter() {}

void AdminBoundariesConverter::convert(
    const std::vector<boost::filesystem::path> &dirs,
    osmium::io::Writer &writer) {

  const boost::filesystem::path ADMINBNDY_2_SHP = "Adminbndy2.shp";
  const boost::filesystem::path ADMINBNDY_3_SHP = "Adminbndy3.shp";
  const boost::filesystem::path ADMINBNDY_4_SHP = "Adminbndy4.shp";
  const boost::filesystem::path ADMINBNDY_5_SHP = "Adminbndy5.shp";

  g_way_end_points_map.clear();
  addLevel1Boundaries(dirs, writer);

  for (auto dir : dirs) {
    addLevelNBoundaries(dir / ADMINBNDY_2_SHP, writer, 2);
    addLevelNBoundaries(dir / ADMINBNDY_3_SHP, writer, 3);
    addLevelNBoundaries(dir / ADMINBNDY_4_SHP, writer, 4);
    addLevelNBoundaries(dir / ADMINBNDY_5_SHP, writer, 5);
  }

  // build relations for the admin line
  g_mtd_area_map.clear();

  g_way_end_points_map.clear();
}

void AdminBoundariesConverter::addLevel1Boundaries(
    const std::vector<boost::filesystem::path> &dirs,
    osmium::io::Writer &writer) {

  const boost::filesystem::path ADMINLINE_1_SHP = "AdminLine1.shp";
  const boost::filesystem::path ADMINBNDY_1_SHP = "Adminbndy1.shp";

  std::map<int, std::pair<std::vector<osmium::unsigned_object_id_type>,
                          std::vector<osmium::unsigned_object_id_type>>>
      map;

  for (auto dir : dirs) {
    // for some countries the Adminbndy1.shp doesn't contain the whole country
    // border therefore we additionally add the links from AdminLine1.shp
    if (shp_file_exists(dir / ADMINLINE_1_SHP)) {
      auto adminLine = add_admin_lines(dir / ADMINLINE_1_SHP, writer);
      // merge maps
      for (auto &mapEntry : adminLine) {
        std::ranges::copy(mapEntry.second,
                          std::back_inserter(map[mapEntry.first].first));
      }
    } else if (shp_file_exists(dir / ADMINBNDY_1_SHP)) {
      add_admin_shape(dir / ADMINBNDY_1_SHP, writer, map);
    }
  }

  // create relations for admin boundary 1
  osmium::memory::Buffer rel_buffer(BUFFER_SIZE);
  for (auto &adminBoundary : map) {

    build_admin_boundary_relation_with_tags(
        adminBoundary.first, adminBoundary.second.first,
        adminBoundary.second.second, rel_buffer, 1);
  }
  rel_buffer.commit();
  writer(std::move(rel_buffer));
}

void AdminBoundariesConverter::add_admin_shape(
    boost::filesystem::path admin_shape_file, osmium::io::Writer &writer,
    std::map<int, std::pair<std::vector<osmium::unsigned_object_id_type>,
                            std::vector<osmium::unsigned_object_id_type>>>
        &adminLineMap) {

  auto ds = open_shape_file(admin_shape_file);
  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(admin_shape_file.string()));
  assert(layer->GetGeomType() == wkbPolygon);
  osmium::memory::Buffer node_buffer(BUFFER_SIZE);
  osmium::memory::Buffer way_buffer(BUFFER_SIZE);
  osmium::memory::Buffer rel_buffer(BUFFER_SIZE);
  for (auto &feat : *layer) {
    process_admin_boundary(feat, node_buffer, way_buffer, rel_buffer,
                           adminLineMap);
  }
  writer(std::move(node_buffer));
  writer(std::move(way_buffer));
  writer(std::move(rel_buffer));
}

std::map<int, std::vector<osmium::unsigned_object_id_type>>
AdminBoundariesConverter::add_admin_lines(
    boost::filesystem::path admin_line_shape_file, osmium::io::Writer &writer) {
  std::map<int, std::vector<osmium::unsigned_object_id_type>> result;

  auto ds = open_shape_file(admin_line_shape_file);
  auto layer = ds->GetLayer(0);
  if (layer == nullptr)
    throw(shp_empty_error(admin_line_shape_file.string()));
  assert(layer->GetGeomType() == wkbLineString);
  osmium::memory::Buffer node_buffer(BUFFER_SIZE);
  osmium::memory::Buffer way_buffer(BUFFER_SIZE);

  std::set<std::pair<uint64_t, uint64_t>> convertedLinkIds;

  for (auto &feat : *layer) {
    auto areaId = feat->GetFieldAsInteger(AREA_ID.data());
    auto linkId = feat->GetFieldAsInteger(LINK_ID.data());

    if (convertedLinkIds.find(std::make_pair(linkId, areaId)) ==
        convertedLinkIds.end()) {
      auto osmIdVector = build_admin_line(feat, node_buffer, way_buffer);
      std::ranges::copy(osmIdVector, std::back_inserter(result[areaId]));
      convertedLinkIds.insert(std::make_pair(linkId, areaId));
    }
  }
  node_buffer.commit();
  way_buffer.commit();

  writer(std::move(node_buffer));
  writer(std::move(way_buffer));

  return result;
}

void AdminBoundariesConverter::process_admin_boundary(
    const OGRFeatureUniquePtr &feat, osmium::memory::Buffer &node_buffer,
    osmium::memory::Buffer &way_buffer, osmium::memory::Buffer &rel_buffer,
    std::map<int, std::pair<std::vector<osmium::unsigned_object_id_type>,
                            std::vector<osmium::unsigned_object_id_type>>>
        &adminLineMap) {
  auto geom = feat->GetGeometryRef();

  std::vector<osmium::unsigned_object_id_type> exterior_way_ids;
  std::vector<osmium::unsigned_object_id_type> interior_way_ids;
  if (geom->getGeometryType() == wkbMultiPolygon) {
    create_multi_polygon(static_cast<OGRMultiPolygon *>(geom), exterior_way_ids,
                         interior_way_ids, node_buffer, way_buffer);
  } else if (geom->getGeometryType() == wkbPolygon) {
    create_polygon(static_cast<OGRPolygon *>(geom), exterior_way_ids,
                   interior_way_ids, node_buffer, way_buffer);
  } else {
    throw(std::runtime_error("Adminboundaries with geometry=" +
                             std::string(geom->getGeometryName()) +
                             " are not yet supported."));
  }
  osmium::unsigned_object_id_type area_id =
      feat->GetFieldAsInteger(AREA_ID.data());
  if (area_id != 0) {
    std::ranges::copy(exterior_way_ids,
                      std::back_inserter(adminLineMap[area_id].first));
    std::ranges::copy(interior_way_ids,
                      std::back_inserter(adminLineMap[area_id].second));
  } else {
    // need to tag here because there is no unique Area_ID
    build_admin_boundary_relation_with_tags(feat, exterior_way_ids,
                                            interior_way_ids, rel_buffer);
    rel_buffer.commit();
  }

  node_buffer.commit();
  way_buffer.commit();
}

std::vector<osmium::unsigned_object_id_type>
AdminBoundariesConverter::build_admin_line(OGRFeatureUniquePtr &feat,
                                           osmium::memory::Buffer &node_buffer,
                                           osmium::memory::Buffer &way_buffer) {

  auto line = static_cast<OGRLineString *>(feat->GetGeometryRef());

  auto osm_way_node_ids = create_open_way_nodes(line, node_buffer);

  std::vector<osmium::unsigned_object_id_type> osm_way_ids;
  size_t i = 0;
  do {
    osmium::builder::WayBuilder builder(way_buffer);
    setObjectProperties(builder);
    osmium::builder::WayNodeListBuilder wnl_builder(builder);
    for (size_t j = i;
         j < std::min(i + OSM_MAX_WAY_NODES, osm_way_node_ids.size()); j++) {
      const auto &[location, osm_id] = osm_way_node_ids.at(j);
      wnl_builder.add_node_ref(osm_id, location);
    }
    osm_way_ids.push_back(builder.object().id());
    i += OSM_MAX_WAY_NODES - 1;
  } while (i < osm_way_node_ids.size());
  return osm_way_ids;
}

osmium::unsigned_object_id_type
AdminBoundariesConverter::build_admin_boundary_relation_with_tags(
    osmium::unsigned_object_id_type area_id,
    const std::vector<osmium::unsigned_object_id_type> &ext_osm_way_ids,
    const std::vector<osmium::unsigned_object_id_type> &int_osm_way_ids,
    osmium::memory::Buffer &rel_buffer, uint level) {
  osmium::builder::RelationBuilder builder(rel_buffer);
  setObjectProperties(builder);
  build_admin_boundary_taglist(builder, area_id, level);
  build_relation_members(builder, ext_osm_way_ids, int_osm_way_ids);
  return builder.object().id();
}

osmium::unsigned_object_id_type
AdminBoundariesConverter::build_admin_boundary_relation_with_tags(
    const OGRFeatureUniquePtr &feat,
    const std::vector<osmium::unsigned_object_id_type> &ext_osm_way_ids,
    const std::vector<osmium::unsigned_object_id_type> &int_osm_way_ids,
    osmium::memory::Buffer &rel_buffer) {
  osmium::builder::RelationBuilder builder(rel_buffer);
  setObjectProperties(builder);
  build_admin_boundary_taglist(builder, feat);
  build_relation_members(builder, ext_osm_way_ids, int_osm_way_ids);
  return builder.object().id();
}

void AdminBoundariesConverter::addLevelNBoundaries(boost::filesystem::path dir,
                                                   osmium::io::Writer &writer,
                                                   uint level) {
  std::map<int, std::pair<std::vector<osmium::unsigned_object_id_type>,
                          std::vector<osmium::unsigned_object_id_type>>>
      map;
  add_admin_shape(dir, writer, map);

  // bild boundary relation
  osmium::memory::Buffer rel_buffer(BUFFER_SIZE);
  for (auto &adminBoundary : map) {

    build_admin_boundary_relation_with_tags(
        adminBoundary.first, adminBoundary.second.first,
        adminBoundary.second.second, rel_buffer, level);
  }
  rel_buffer.commit();
  writer(std::move(rel_buffer));
}

void AdminBoundariesConverter::build_admin_boundary_taglist(
    osmium::builder::Builder &builder, osmium::unsigned_object_id_type area_id,
    uint level) {
  osmium::builder::TagListBuilder tl_builder(builder);
  // Mind tl_builder scope in calling method!
  if (level == 5) {
    // only landuse residential
    tl_builder.add_tag("type", "multipolygon");
    tl_builder.add_tag("landuse", "residential");
  } else {
    tl_builder.add_tag("type", "boundary");
    tl_builder.add_tag("boundary", "administrative");
  }

  auto it = g_mtd_area_map.find(area_id);
  if (it != g_mtd_area_map.end()) {
    auto d = it->second;
    if (!d.admin_lvl.empty())
      tl_builder.add_tag("navteq_admin_level", d.admin_lvl);

    if (!d.admin_lvl.empty())
      tl_builder.add_tag("admin_level", navteq_2_osm_admin_lvl(d.admin_lvl));
    if (!d.name.empty())
      tl_builder.add_tag("name", d.name);
    if (!d.short_name.empty())
      tl_builder.add_tag("short_name", d.short_name);
    if (level != 5) {
      for (auto it : d.lang_code_2_area_name)
        tl_builder.add_tag(std::string("name:" + parse_lang_code(it.first)),
                           it.second);
    }
  } else {
    BOOST_LOG_TRIVIAL(error) << "Skipping unknown navteq_admin_level";
  }
}

void AdminBoundariesConverter::build_admin_boundary_taglist(
    osmium::builder::Builder &builder, const OGRFeatureUniquePtr &feat) {
  osmium::builder::TagListBuilder tl_builder(builder);

  int level = 0;

  std::string featureCode = feat->GetFieldAsString(FEAT_COD.data());
  if (featureCode == "907196") {
    level = 1;
  } else if (featureCode == "909996") {
    level = 2;
  } else if (featureCode == "900170") {
    level = 3;
  } else if (featureCode == "900101") {
    level = 4;
  } else if (featureCode == "900156") {
    level = 5;
  } else {
    BOOST_LOG_TRIVIAL(error) << "Unknown admin level " << featureCode;
    return;
  }

  // Mind tl_builder scope in calling method!
  if (level == 5) {
    // only landuse residential
    tl_builder.add_tag("type", "multipolygon");
    tl_builder.add_tag("landuse", "residential");
  } else {
    tl_builder.add_tag("type", "boundary");
    tl_builder.add_tag("boundary", "administrative");
  }

  std::string polygonName = feat->GetFieldAsString(POLYGON_NM.data());
  if (!polygonName.empty()) {
    std::string waters_name = to_camel_case_with_spaces(polygonName);
    if (!waters_name.empty())
      tl_builder.add_tag("name", waters_name);
  }

  tl_builder.add_tag("admin_level", navteq_2_osm_admin_lvl(level));
}
