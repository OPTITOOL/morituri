/*
 * ogr_util.hpp
 *
 *  Created on: 11.12.2015
 *      Author: philip
 */

#ifndef PLUGINS_OGR_UTIL_HPP_
#define PLUGINS_OGR_UTIL_HPP_

#include <assert.h>

#include <geos/io/WKBReader.h>
#include <geos/io/WKBWriter.h>

#include <geos/geom/CoordinateFilter.h>
#include <geos/geom/CoordinateSequenceFactory.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/LineString.h>
#include <geos/geom/Point.h>
#include <geos/operation/buffer/BufferParameters.h>
#include <geos/operation/buffer/OffsetCurveBuilder.h>

// create static geom factory
geos::geom::GeometryFactory::Ptr geos_factory =
    geos::geom::GeometryFactory::create();

geos::operation::buffer::BufferParameters
    ocb(geos::operation::buffer::BufferParameters::DEFAULT_QUADRANT_SEGMENTS,
        geos::operation::buffer::BufferParameters::EndCapStyle::CAP_FLAT,
        geos::operation::buffer::BufferParameters::JoinStyle::JOIN_BEVEL,
        geos::operation::buffer::BufferParameters::DEFAULT_MITRE_LIMIT);

// create static buffer operation
std::unique_ptr<geos::operation::buffer::OffsetCurveBuilder>
    offset_curve_builder =
        std::make_unique<geos::operation::buffer::OffsetCurveBuilder>(
            geos_factory->getPrecisionModel(), ocb);

/**
 * Following functions convert OGRGeometry to geos::geom::Geometry and vice
 * versa
 */

geos::io::WKBReader wkb_reader;
geos::geom::Geometry::Ptr ogr2geos(const OGRGeometry *ogr_geom) {
  if (!ogr_geom || ogr_geom->IsEmpty())
    throw std::runtime_error("geometry is nullptr");

  unsigned char staticbuffer[1024 * 1024];
  unsigned char *buffer = staticbuffer;
  size_t wkb_size = ogr_geom->WkbSize();
  if (wkb_size > sizeof(staticbuffer))
    buffer = (unsigned char *)malloc(wkb_size);
  ogr_geom->exportToWkb(wkbNDR, buffer);

  geos::geom::Geometry::Ptr geos_geom = wkb_reader.read(buffer, wkb_size);
  if (buffer != staticbuffer)
    free(buffer);

  if (!geos_geom)
    throw std::runtime_error("creating geos::geom::Geometry from wkb failed");

  return geos_geom;
}

geos::io::WKBWriter wkb_writer;
OGRGeometry *geos2ogr(const geos::geom::Geometry *geos_geom) {
  std::ostringstream ss;
  wkb_writer.setOutputDimension(geos_geom->getCoordinateDimension());
  wkb_writer.write(*geos_geom, ss);
  auto str = ss.str();

  OGRGeometry *ogr_geom = nullptr;
  OGRErr res = OGRGeometryFactory::createFromWkb(
      (unsigned char *)(str.c_str()), nullptr, &ogr_geom, str.size());
  if (res != OGRERR_NONE)
    throw std::runtime_error("creating OGRGeometry from wkb failed: " +
                             std::to_string(res));
  return ogr_geom;
}

geos::geom::Coordinate move_point(const geos::geom::Coordinate &moving_coord,
                                  const geos::geom::Coordinate &reference_coord,
                                  double move_distance) {

  double distance = moving_coord.distance(reference_coord);
  // assert(move_distance < distance);

  // intercept theorem
  double ratio = move_distance / distance;
  auto move_x = ratio * (reference_coord.x - moving_coord.x);
  auto move_y = ratio * (reference_coord.y - moving_coord.y);

  return geos::geom::Coordinate(moving_coord.x + move_x,
                                moving_coord.y + move_y);
}

std::unique_ptr<geos::geom::CoordinateSequence>
trimAndCloneCoordinateSequence(const geos::geom::CoordinateSequence *geos_cs) {
  geos::geom::Coordinate::Vect *coordVector =
      new geos::geom::Coordinate::Vect();

  const auto &frontC = geos_cs->front();
  const auto &backC = geos_cs->back();

  int size = geos_cs->getSize();

  // getSingleSidedLineCurve() always produces a ring (bug?). therefore:
  // first_coord == last_coord => drop last_coord
  if (frontC == backC)
    size--;

  double cut_ratio = 0.1;
  double max_cut = 0.00025;
  double length = 0.0;

  for (int i = 1; i < size; ++i) {
    length += geos_cs->getAt(i).distance(geos_cs->getAt(i - 1));
  }

  double cut = std::min(max_cut, length * cut_ratio);

  bool lastFrontSkipped = false;
  bool frontFinished = false;

  double lastFrontCut = 0.0;

  for (int i = 0; i < size; ++i) {
    auto &currentC = geos_cs->getAt(i);

    // trim front
    if (!frontFinished && frontC.distance(currentC) < cut) {
      lastFrontCut = cut - frontC.distance(currentC);
      lastFrontSkipped = true;
      continue;
    } else if (lastFrontSkipped) {
      // create a new point in front at cut distance
      if (lastFrontCut > 0)
        coordVector->push_back(
            move_point(geos_cs->getAt(i - 1), currentC, lastFrontCut));

      lastFrontSkipped = false;
      frontFinished = true;
    }

    // trim back
    double cutBackDistance = backC.distance(currentC);
    if (cutBackDistance < cut) {
      // create a new point in back at cut distance
      double backCut = cut - cutBackDistance;
      if (backCut > 0)
        coordVector->push_back(
            move_point(currentC, coordVector->back(), backCut));
      break;
    }

    // insert copy
    coordVector->push_back(currentC);
  }

  return geos_factory->getCoordinateSequenceFactory()->create(coordVector);
}

std::unique_ptr<geos::geom::LineString>
cut_caps(const geos::geom::CoordinateSequence *cs) {

  auto geos_cs = trimAndCloneCoordinateSequence(cs);

  return geos_factory->createLineString(std::move(geos_cs));
}

OGRLineString *create_offset_curve(const OGRLineString *ogr_ls, double offset,
                                   bool left) {

  std::vector<geos::geom::CoordinateSequence *> cs_vec;
  auto convGeometry = (geos::geom::LineString::Ptr)(ogr2geos(ogr_ls));
  offset_curve_builder->getSingleSidedLineCurve(
      convGeometry->getCoordinates().get(), offset, cs_vec, left, !left);

  auto cs = cs_vec.front();

  auto cut_caps_geos_ls = cut_caps(cs);
  auto offset_ogr_geom = geos2ogr(cut_caps_geos_ls.get());
  delete cs;

  return static_cast<OGRLineString *>(offset_ogr_geom);
}

#endif /* PLUGINS_OGR_UTIL_HPP_ */
