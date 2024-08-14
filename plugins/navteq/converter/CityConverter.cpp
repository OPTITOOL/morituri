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

#include "CityConverter.hpp"

#include <boost/log/trivial.hpp>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/io/writer.hpp>
#include <osmium/memory/buffer.hpp>
#include <osmium/osm/location.hpp>
#include <osmium/osm/types.hpp>

CityConverter::CityConverter(const std::filesystem::path &executable_path)
    : Converter(executable_path) {}

CityConverter::~CityConverter() {}

void CityConverter::convert(const std::filesystem::path &dir,
                            osmium::io::Writer &writer) {

  const std::filesystem::path NAMED_PLC_SHP = "NamedPlc.shp";
  add_city_shape(dir / NAMED_PLC_SHP, writer);
}

void CityConverter::add_city_shape(std::filesystem::path city_shape_file,
                                   osmium::io::Writer &writer) {

  auto ds = openDataSource(city_shape_file);
  if (!ds) {
    return;
  }

  auto layer = ds->GetLayer(0);
  if (!layer) {
    throw(shp_empty_error(city_shape_file));
  }

  osmium::memory::Buffer node_buffer(BUFFER_SIZE);

  int facTypeField = layer->FindFieldIndex(FAC_TYPE.data(), true);
  int poiNmTypeField = layer->FindFieldIndex(POI_NMTYPE.data(), true);
  int poiLangCodeField = layer->FindFieldIndex(POI_LANGCD.data(), true);
  int poiIdField = layer->FindFieldIndex(POI_ID.data(), true);
  int poiNameField = layer->FindFieldIndex(POI_NAME.data(), true);

  std::map<uint64_t, std::map<std::string, std::string>> translations;
  // read translations
  for (auto &feat : *layer) {
    uint fac_type = feat->GetFieldAsInteger(facTypeField);
    if (fac_type != 4444 && fac_type != 9709) {
      BOOST_LOG_TRIVIAL(error)
          << "Skipping city node because of wrong POI type";
      continue;
    }
    std::string name_type = feat->GetFieldAsString(poiNmTypeField);
    if (name_type == "B") {
      // Skip this entry as it's just a translated namePlc of former one
      continue;
    }
    int poiId = feat->GetFieldAsInteger(poiIdField);
    std::string langCode = feat->GetFieldAsString(poiLangCodeField);
    std::string locName = feat->GetFieldAsString(poiNameField);

    translations[poiId].emplace(parse_lang_code(langCode),
                                to_camel_case_with_spaces(locName));
  }

  layer->ResetReading();

  for (auto &feat : *layer) {
    uint fac_type = feat->GetFieldAsInteger(facTypeField);
    if (fac_type != 4444 && fac_type != 9709) {
      BOOST_LOG_TRIVIAL(error)
          << "Skipping city node because of wrong POI type";
      continue;
    }

    std::string name_type = feat->GetFieldAsString(poiNmTypeField);
    if (name_type != "B") {
      // Skip this entry as it's just a translated namePlc of former one
      continue;
    }
    int poiId = feat->GetFieldAsInteger(poiIdField);

    process_city(feat, fac_type, node_buffer, translations[poiId]);
  }
  node_buffer.commit();
  {
    std::lock_guard<std::mutex> lock(osmiumWriterMutex);
    writer(std::move(node_buffer));
  }
}

void CityConverter::process_city(
    const OGRFeatureUniquePtr &feat, uint fac_type,
    osmium::memory::Buffer &node_buffer,
    const std::map<std::string, std::string> &translations) {

  auto geom = feat->GetGeometryRef();
  auto geom_type = geom->getGeometryType();

  if (geom_type != wkbPoint) {
    throw(std::runtime_error(
        "City item with geometry=" + std::string(geom->getGeometryName()) +
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
    if (fac_type == 9709) {
      // 9709 means 'neighbourhood'
      tl_builder.add_tag("place", "suburb");
    } else { //=> fac_type = 4444 means 'named place' which is the city centre
      int population = feat->GetFieldAsInteger(POPULATION.data());
      if (population > 0)
        tl_builder.add_tag("population", std::to_string(population));
      uint capital = feat->GetFieldAsInteger(CAPITAL.data());
      tl_builder.add_tag("place", get_place_value(population, capital));
      if (capital == 1) {
        // for capitals of countries use 'capital' = 'yes'
        tl_builder.add_tag("capital", "yes");
      } else if (capital > 1) {
        // for subdivisions of countries use 'capital' = admin_lvl
        tl_builder.add_tag("capital", navteq_2_osm_admin_lvl(capital));
      }
    }

    // add translation tags
    for (const auto &[langCode, name] : translations) {
      tl_builder.add_tag("name:" + langCode, name);
    }
  }
  node_buffer.commit();
}

std::string CityConverter::get_place_value(uint population, uint capital) {
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