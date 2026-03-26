#include "geom/complex_polygon.h"

namespace geom {

Segments ComplexPolygon::segments() const noexcept {
    size_t size = outer.size();
    for (const auto& inner : inners) {
        size += inner.size();
    }
    Segments segments;
    segments.reserve(size);
    auto current_segments = outer.segments();
    segments.insert(segments.end(), current_segments.begin(), current_segments.end());
    for (const auto& inner : inners) {
        current_segments = inner.segments();
        segments.insert(segments.end(), current_segments.begin(), current_segments.end());
    }
    return segments;
}

}  // namespace geom
