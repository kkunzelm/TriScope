#pragma once

#include "interfaces/ICamera.h"

#include <span>
#include <vector>

namespace scanner {

// Per-frame result: one subpixel row coordinate per camera column.
// A value of -1.0 means no valid laser reflection was found in that column.
struct LaserProfile {
    std::vector<double> rowPositions;  // index = camera column
    int frameWidth{0};
    int frameHeight{0};
    int validColumns{0};               // columns where rowPosition >= 0
};

enum class PeakMethod {
    Gaussian,  // 3-point log-Gaussian interpolation (default, ~20× subpixel, per Weber 1995)
    CoG,       // Center of Gravity fallback
};

struct ExtractorParams {
    uint8_t    threshold{50};                // minimum pixel value to consider as laser (0–255)
    int        windowRows{5};               // CoG fallback: restrict to ±windowRows around peak (0 = full column)
    PeakMethod method{PeakMethod::Gaussian}; // subpixel interpolation method
    double     medianRejectRows{50.0};       // post-filter: invalidate columns deviating more than this from the median (0 = disabled)
};

// Extract the laser line position (subpixel) from a 16-bit monochrome frame.
LaserProfile extractLaserProfile(const Frame& frame, const ExtractorParams& params = {});

// Variant accepting a raw span (useful for unit tests without a full Frame object).
LaserProfile extractLaserProfile(std::span<const uint8_t> data,
                                  int width, int height,
                                  const ExtractorParams& params = {});

} // namespace scanner
