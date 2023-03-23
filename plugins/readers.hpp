/*
 * readers.hpp
 *
 *  Created on: 17.06.2015
 *      Author: philip
 */

#ifndef PLUGINS_READERS_HPP_
#define PLUGINS_READERS_HPP_

#include <boost/filesystem/path.hpp>
#include <boost/log/trivial.hpp>
#include <ogrsf_frmts.h>
#include <shapefil.h>

#include "comm2osm_exceptions.hpp"

GDALDatasetUniquePtr open_shape_file(boost::filesystem::path shp_file) {
  BOOST_LOG_TRIVIAL(info) << "\treading " << shp_file;

  auto input_data_source =
      GDALDatasetUniquePtr(GDALDataset::Open(shp_file.c_str()));
  if (!input_data_source)
    throw(shp_error(shp_file.string()));

  return input_data_source;
}

DBFHandle read_dbf_file(boost::filesystem::path dbf_file) {
  BOOST_LOG_TRIVIAL(info) << "\treading " << dbf_file;
  DBFHandle handle = DBFOpen(dbf_file.c_str(), "rb");
  if (handle == nullptr)
    throw(dbf_error(dbf_file.string()));
  return handle;
}

#endif /* PLUGINS_READERS_HPP_ */
