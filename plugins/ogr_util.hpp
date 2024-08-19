/*
 * ogr_util.hpp
 *
 *  Created on: 11.12.2015
 *      Author: philip
 */

#ifndef PLUGINS_OGR_UTIL_HPP_
#define PLUGINS_OGR_UTIL_HPP_

#include <assert.h>

#include <geos/geom/CoordinateFilter.h>
#include <geos/geom/Geometry.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/LineString.h>
#include <geos/geom/Point.h>
#include <geos/operation/buffer/BufferParameters.h>
#include <geos/operation/buffer/OffsetCurveBuilder.h>
#include <iterator>

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
const GEOSContextHandle_t gctx = OGRGeometry::createGEOSContext();

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

geos::geom::CoordinateSequence trimAndCloneCoordinateSequence(
    const geos::geom::CoordinateSequence::Ptr &geos_cs) {
  geos::geom::CoordinateSequence coordSequence;

  std::vector<geos::geom::Coordinate> coords;
  geos_cs->toVector(coords);

  // remove last point if it is the same as the first getOffestCurve always
  // creates a closed ring
  if (geos_cs->getAt(0) == geos_cs->getAt(geos_cs->getSize() - 1)) {
    coords.pop_back();
  }

  auto frontIter = coords.begin();
  auto backIter = std::prev(coords.end());

  double cut_ratio = 0.1;
  double max_cut = 0.00025;
  double length = 0.0;

  // calculate length of line
  for (auto it = frontIter; it != backIter; ++it) {
    length += it->distance(*std::next(it));
  }

  double cut = std::min(max_cut, length * cut_ratio);

  bool lastFrontSkipped = false;
  bool frontFinished = false;

  double lastFrontCut = 0.0;

  for (auto currentIter = frontIter; currentIter != coords.end();
       ++currentIter) {

    // trim front
    if (!frontFinished && frontIter->distance(*currentIter) < cut) {
      lastFrontCut = cut - frontIter->distance(*currentIter);
      lastFrontSkipped = true;
      continue;
    } else if (lastFrontSkipped) {
      // create a new point in front at cut distance
      if (lastFrontCut > 0)
        coordSequence.add(
            move_point(*std::prev(currentIter), *currentIter, lastFrontCut));

      lastFrontSkipped = false;
      frontFinished = true;
    }

    // trim back
    double cutBackDistance = backIter->distance(*currentIter);
    if (cutBackDistance < cut) {
      // create a new point in back at cut distance
      double backCut = cut - cutBackDistance;
      if (backCut > 0)
        coordSequence.add(
            move_point(*currentIter, *std::prev(currentIter), backCut));
      break;
    }

    // insert copy
    coordSequence.add(*currentIter);
  }

  return coordSequence;
}

std::unique_ptr<geos::geom::LineString>
cut_caps(const geos::geom::CoordinateSequence::Ptr &cs) {
  auto geos_cs = trimAndCloneCoordinateSequence(cs);
  return geos_factory->createLineString(geos_cs);
}

std::unique_ptr<OGRLineString> create_offset_curve(const OGRLineString *ogr_ls,
                                                   double offset, bool left) {

  if (!ogr_ls || ogr_ls->IsEmpty())
    throw std::runtime_error("geometry is nullptr");

  auto geos_geom = geos::geom::Geometry::Ptr(
      (geos::geom::Geometry *)ogr_ls->exportToGEOS(gctx));
  if (!geos_geom)
    throw std::runtime_error("creating geos::geom::Geometry from wkb failed");

  auto curve = offset_curve_builder->getOffsetCurve(
      geos_geom->getCoordinates().get(), left ? offset : -offset);

  if (!left)
    curve->reverse();

  auto cut_caps_geos_ls = cut_caps(curve);

  // convert geos to ogr
  auto offset_ogr_geom = OGRGeometryFactory::createFromGEOS(
      gctx, (GEOSGeom)cut_caps_geos_ls.get());
  if (!offset_ogr_geom)
    throw std::runtime_error("creating OGRGeometry from wkb failed");

  return std::unique_ptr<OGRLineString>(
      static_cast<OGRLineString *>(offset_ogr_geom));
}

#endif /* PLUGINS_OGR_UTIL_HPP_ */
