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

#ifndef HAMELTCONVERTER_HPP
#define HAMELTCONVERTER_HPP

#include "Converter.hpp"

#include <ogrsf_frmts.h>

class HamletConverter : public Converter {

public:
  HamletConverter(const std::filesystem::path &executable_path);
  virtual ~HamletConverter();

  virtual void convert(const std::filesystem::path &dir,
                       osmium::io::Writer &writer) override;

private:
  void add_hamlet(std::filesystem::path hamlet_file,
                  osmium::io::Writer &writer);

  void process_hamlets(const OGRFeatureUniquePtr &feat,
                       osmium::memory::Buffer &node_buffer);
};

#endif // HAMELTCONVERTER_HPP
