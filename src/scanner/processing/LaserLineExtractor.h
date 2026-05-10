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
    uint16_t   threshold{500};               // minimum pixel value to consider as laser
    int        windowRows{0};                // CoG only: restrict to ±windowRows around peak
    PeakMethod method{PeakMethod::Gaussian}; // subpixel interpolation method
};

// Extract the laser line position (subpixel) from a 16-bit monochrome frame.
LaserProfile extractLaserProfile(const Frame& frame, const ExtractorParams& params = {});

// Variant accepting a raw span (useful for unit tests without a full Frame object).
LaserProfile extractLaserProfile(std::span<const uint16_t> data,
                                  int width, int height,
                                  const ExtractorParams& params = {});

} // namespace scanner
