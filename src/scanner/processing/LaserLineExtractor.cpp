#include "LaserLineExtractor.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace scanner {

LaserProfile extractLaserProfile(const Frame& frame, const ExtractorParams& params)
{
    return extractLaserProfile(
        std::span<const uint8_t>(frame.data.data(), frame.data.size()),
        frame.width, frame.height, params);
}

LaserProfile extractLaserProfile(std::span<const uint8_t> data,
                                  int width, int height,
                                  const ExtractorParams& params)
{
    LaserProfile result;
    result.frameWidth  = width;
    result.frameHeight = height;
    result.colPositions.assign(static_cast<std::size_t>(height), -1.0);

    for (int row = 0; row < height; ++row) {
        // Step 1: find peak column across entire row
        uint8_t peakVal = 0;
        int     peakCol = 0;
        for (int col = 0; col < width; ++col) {
            const uint8_t v = data[static_cast<std::size_t>(row * width + col)];
            if (v > peakVal) { peakVal = v; peakCol = col; }
        }
        if (peakVal <= params.threshold) continue;

        // Step 2a: Gaussian 3-point log-interpolation (Weber 1995, Eq. 4)
        // Works on the 3 pixels centred on the peak; falls through to CoG on failure.
        if (params.method == PeakMethod::Gaussian &&
            peakCol > 0 && peakCol < width - 1)
        {
            const double i0 = static_cast<double>(
                data[static_cast<std::size_t>(row * width + peakCol - 1)]);
            const double i1 = static_cast<double>(
                data[static_cast<std::size_t>(row * width + peakCol)]);
            const double i2 = static_cast<double>(
                data[static_cast<std::size_t>(row * width + peakCol + 1)]);

            if (i0 > 0.0 && i2 > 0.0) {
                const double li0 = std::log(i0);
                const double li1 = std::log(i1);
                const double li2 = std::log(i2);
                const double denom = li0 - 2.0 * li1 + li2;
                if (denom < -1e-10) {
                    result.colPositions[static_cast<std::size_t>(row)] =
                        peakCol + 0.5 * (li0 - li2) / denom;
                    ++result.validRows;
                    continue;
                }
            }
        }

        // Step 2b: Center-of-Gravity (fallback, or explicit CoG method)
        const int colMin = (params.windowCols > 0)
                         ? std::max(0, peakCol - params.windowCols) : 0;
        const int colMax = (params.windowCols > 0)
                         ? std::min(width, peakCol + params.windowCols + 1) : width;

        double sumW = 0.0, sumWX = 0.0;
        for (int col = colMin; col < colMax; ++col) {
            const double v = static_cast<double>(
                data[static_cast<std::size_t>(row * width + col)]);
            if (v > static_cast<double>(params.threshold)) {
                sumW  += v;
                sumWX += v * col;
            }
        }
        if (sumW > 0.0) {
            result.colPositions[static_cast<std::size_t>(row)] = sumWX / sumW;
            ++result.validRows;
        }
    }

    // Post-extraction median filter: collect valid col positions, find the
    // median, then invalidate any row that deviates more than medianRejectCols
    // from it.  This removes isolated hot-pixel hits and ambient-light
    // reflections that survive the threshold but are far from the main laser stripe.
    if (params.medianRejectCols > 0.0 && result.validRows >= 2) {
        std::vector<double> valid;
        valid.reserve(static_cast<std::size_t>(result.validRows));
        for (double c : result.colPositions)
            if (c >= 0.0) valid.push_back(c);

        std::sort(valid.begin(), valid.end());
        const std::size_t mid = valid.size() / 2;
        const double median = (valid.size() % 2 == 1)
                            ? valid[mid]
                            : (valid[mid - 1] + valid[mid]) * 0.5;

        result.validRows = 0;
        for (double &c : result.colPositions) {
            if (c >= 0.0) {
                if (std::abs(c - median) > params.medianRejectCols)
                    c = -1.0;
                else
                    ++result.validRows;
            }
        }
    }

    return result;
}

} // namespace scanner
