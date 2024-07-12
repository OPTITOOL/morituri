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

#ifndef POINTLAYERCONVERTER_HPP
#define POINTLAYERCONVERTER_HPP

#include "Converter.hpp"

#include <ogrsf_frmts.h>

class PointLayerConverter : public Converter {

public:
  PointLayerConverter();
  virtual ~PointLayerConverter();

  virtual void convert(const std::vector<boost::filesystem::path> &dirs,
                       osmium::io::Writer &writer);

private:
};

#endif // POINTLAYERCONVERTER_HPP