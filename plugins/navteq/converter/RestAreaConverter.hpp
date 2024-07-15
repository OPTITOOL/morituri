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

#ifndef RESTAREACONVERTER_HPP
#define RESTAREACONVERTER_HPP

#include "Converter.hpp"

#include <ogrsf_frmts.h>

class RestAreaConverter : public Converter {

public:
  RestAreaConverter(const std::filesystem::path &executable_path);
  virtual ~RestAreaConverter();

  virtual void convert(const std::vector<std::filesystem::path> &dirs,
                       osmium::io::Writer &writer);

private:
  void add_rest_area(std::filesystem::path rest_area_file,
                     osmium::io::Writer &writer);

  void process_rest_area(const OGRFeatureUniquePtr &feat,
                         osmium::memory::Buffer &node_buffer);
};

#endif // RESTAREACONVERTER_HPP