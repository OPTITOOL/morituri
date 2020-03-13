/*
 * readers.hpp
 *
 *  Created on: 17.06.2015
 *      Author: philip
 */

#ifndef PLUGINS_READERS_HPP_
#define PLUGINS_READERS_HPP_

#include <boost/filesystem/path.hpp>
#include <gdal/ogrsf_frmts.h>
#include <shapefil.h>

#include "comm2osm_exceptions.hpp"

/*
 * \brief Checks shapefile existance and validity.
 * \param shp_file path and file name of shapefile.
 * \return Pointer to first layer in Shapefile.
 * */

OGRLayer *read_shape_file(boost::filesystem::path shp_file,
                          std::ostream &out = std::cerr) {
  out << "reading " << shp_file << std::endl;

  GDALDataset *input_data_source = (GDALDataset *)GDALOpenEx(
      shp_file.c_str(), GDAL_OF_READONLY, nullptr, nullptr, nullptr);
  if (input_data_source == nullptr)
    throw(shp_error(shp_file.string()));

  OGRLayer *input_layer = input_data_source->GetLayer(0);
  if (input_layer == nullptr)
    throw(shp_empty_error(shp_file.string()));

  return input_layer;
}

GDALDataset *open_shape_file(boost::filesystem::path shp_file,
                             std::ostream &out = std::cerr) {
  out << "reading " << shp_file << std::endl;

  GDALDataset *input_data_source = (GDALDataset *)GDALOpenEx(
      shp_file.c_str(), GDAL_OF_READONLY, nullptr, nullptr, nullptr);
  if (input_data_source == nullptr)
    throw(shp_error(shp_file.string()));

  return input_data_source;
}

DBFHandle read_dbf_file(boost::filesystem::path dbf_file,
                        std::ostream &out = std::cerr) {
  out << "reading " << dbf_file << std::endl;
  DBFHandle handle = DBFOpen(dbf_file.c_str(), "rb");
  if (handle == nullptr)
    throw(dbf_error(dbf_file.string()));
  return handle;
}

#endif /* PLUGINS_READERS_HPP_ */
