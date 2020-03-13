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
