#pragma once

#include "interfaces/ICamera.h"

#include <span>
#include <vector>

namespace scanner {

// Per-frame result: one subpixel column coordinate per camera row.
// A value of -1.0 means no valid laser reflection was found in that row.
struct LaserProfile {
    std::vector<double> colPositions;  // index = camera row
    int frameWidth{0};
    int frameHeight{0};
    int validRows{0};                  // rows where colPosition >= 0
};

enum class PeakMethod {
    Gaussian,  // 3-point log-Gaussian interpolation (default, ~20× subpixel, per Weber 1995)
    CoG,       // Center of Gravity fallback
};

struct ExtractorParams {
    uint8_t    threshold{50};                 // minimum pixel value to consider as laser (0–255)
    int        windowCols{5};                 // CoG fallback: restrict to ±windowCols around peak (0 = full row)
    PeakMethod method{PeakMethod::Gaussian};  // subpixel interpolation method
    double     medianRejectCols{50.0};        // post-filter: invalidate rows deviating more than this from the median (0 = disabled)
};

// Extract the laser line position (subpixel) from an 8-bit monochrome frame.
// The laser line is assumed to run vertically in the image (parallel to world Y).
// Returns one column position per row; -1.0 means no valid laser in that row.
LaserProfile extractLaserProfile(const Frame& frame, const ExtractorParams& params = {});

// Variant accepting a raw span (useful for unit tests without a full Frame object).
LaserProfile extractLaserProfile(std::span<const uint8_t> data,
                                  int width, int height,
                                  const ExtractorParams& params = {});

} // namespace scanner
