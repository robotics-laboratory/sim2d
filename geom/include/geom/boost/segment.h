#pragma once

#include "geom/segment.h"
#include "geom/boost/point.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/segment.hpp>

BOOST_GEOMETRY_REGISTER_SEGMENT(geom::Segment, geom::Vec2, begin, end)
