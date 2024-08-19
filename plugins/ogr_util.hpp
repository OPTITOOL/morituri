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
        coordSequence.add(
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
        coordSequence.add(move_point(currentC, coordSequence.back(), backCut));
      break;
    }

    // insert copy
    coordSequence.add(currentC);
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

  std::vector<geos::geom::CoordinateSequence *> coord_seq_list;

  offset_curve_builder->getSingleSidedLineCurve(
      geos_geom->getCoordinates().get(), offset, coord_seq_list, left, !left);

  if (coord_seq_list.size() > 1)
    BOOST_LOG_TRIVIAL(debug) << "offset curve created";

  auto cs = geos::geom::CoordinateSequence::Ptr(coord_seq_list.front());

  auto cut_caps_geos_ls = cut_caps(cs);

  // convert geos to ogr
  auto offset_ogr_geom = OGRGeometryFactory::createFromGEOS(
      gctx, (GEOSGeom)cut_caps_geos_ls.get());
  if (!offset_ogr_geom)
    throw std::runtime_error("creating OGRGeometry from wkb failed");

  return std::unique_ptr<OGRLineString>(
      static_cast<OGRLineString *>(offset_ogr_geom));
}

#endif /* PLUGINS_OGR_UTIL_HPP_ */
