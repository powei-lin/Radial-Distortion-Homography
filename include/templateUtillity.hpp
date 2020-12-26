#pragma once
#include <cstdint>

template <typename point>
inline bool inBoard(const point &pt, uint16_t w, uint16_t h, uint16_t mrg) {
  return pt.x > mrg && pt.x < w - mrg && pt.y > mrg && pt.y < h - mrg;
}