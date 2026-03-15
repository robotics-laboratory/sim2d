#pragma once

#include "geom/pose.h"

namespace geom {

struct Localization {
    Pose pose;
    double velocity;
};

}  // namespace geom
