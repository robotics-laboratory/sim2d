#pragma once

namespace geom {

inline void swap(Angle& a, Angle& b) noexcept {
    const auto a_rad = a.radians();
    a = Angle(b.radians());
    b = Angle(a_rad);
}

}  // namespace geom
