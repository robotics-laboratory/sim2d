#pragma once

#include "geom/bounding_box.h"
#include "geom/boost/point.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/box.hpp>

BOOST_GEOMETRY_REGISTER_BOX(geom::BoundingBox, geom::Vec2, min, max)
