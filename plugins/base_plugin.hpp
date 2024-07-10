/*
 * base_plugin.h
 *
 *  Created on: 11.06.2015
 *      Author: philip
 */

#ifndef BASEPLUGIN_HPP_
#define BASEPLUGIN_HPP_

#include <assert.h>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <osmium/io/error.hpp>

class base_plugin {
public:
  std::string name;
  boost::filesystem::path input_path;
  boost::filesystem::path output_path;
  boost::filesystem::path executable_path;

  base_plugin() { this->name = ""; };
  base_plugin(const std::string &name) { this->name = name; };
  base_plugin(const std::string &name,
              const boost::filesystem::path &executable_path) {
    this->name = name;
    this->executable_path = executable_path.parent_path();
  }

  virtual ~base_plugin(){};

  std::string get_name() { return name; }

  /*
   * \brief	Sets input_path and output_path.
   *
   * 			Sets the variables input_path and output_path in
   * BasePlugin
   *
   * \param	inputh_path_rhs	input path to set
   * \param	output_path_rhs	output path to set (may be ommited)
   * */
  void plugin_setup(const boost::filesystem::path &input_path_rhs,
                    const boost::filesystem::path &output_path_rhs =
                        boost::filesystem::path()) {
    input_path = input_path_rhs;
    if (!boost::filesystem::is_directory(input_path))
      throw(osmium::io_error("input_path '" + input_path.string() +
                             "' is not valid."));
    if (boost::filesystem::is_directory(output_path_rhs.parent_path()))
      output_path = output_path_rhs;
  }

  /**
   * \brief	Checks validity of input.
   *
   * 			This function is pure virtual and has to be implemented
   *by every plugin. It should assure that all necessary files are existing and
   *valid.
   *
   * \param    input_path    path in which the input_files are
   * \param   output_path    path, filename and suffix (e.g. /path/to/file.osm)
   *                         to which the result will be written.
   *                         All suffixes supported by libosmium may be used.
   *                         Currently supported formats are:
   * 	                       		.osm (XML),
   *		                        .pbf (PBF),
   *      	                    .opl (OPL),
   *           	        	    .gz (gzip),
   *           			        .bz2 (bzip2).
   *                         If ommited a standard value can be applied.
   *
   * \return returns true if input is existing and valid
   *  */
  virtual bool check_input(const boost::filesystem::path &input_path,
                           const boost::filesystem::path &output_path =
                               boost::filesystem::path("")) = 0;

  /**
   * \brief	Converts input to OSM files
   *
   *			This function is pure virtual and has to be implemented
   *by every plugin. It does the actual conversion to the OSM-format.
   * */
  virtual void execute() = 0;

  /**
   * \brief Only convert elements in the given bounding box
   * */
  virtual void setBoundingBox(double minX, double minY, double maxX,
                              double maxY) = 0;

  virtual void setCountries(const std::vector<std::string> &_countries) = 0;

  virtual void setDebug(bool debug) = 0;
};

#endif /* BASEPLUGIN_HPP_ */
