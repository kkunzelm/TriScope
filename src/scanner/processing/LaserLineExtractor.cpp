#include "LaserLineExtractor.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace scanner {

LaserProfile extractLaserProfile(const Frame& frame, const ExtractorParams& params)
{
    return extractLaserProfile(
        std::span<const uint16_t>(frame.data.data(), frame.data.size()),
        frame.width, frame.height, params);
}

LaserProfile extractLaserProfile(std::span<const uint16_t> data,
                                  int width, int height,
                                  const ExtractorParams& params)
{
    LaserProfile result;
    result.frameWidth  = width;
    result.frameHeight = height;
    result.rowPositions.assign(static_cast<std::size_t>(width), -1.0);

    for (int col = 0; col < width; ++col) {
        // Step 1: find peak row across entire column
        uint16_t peakVal = 0;
        int      peakRow = 0;
        for (int row = 0; row < height; ++row) {
            const uint16_t v = data[static_cast<std::size_t>(row * width + col)];
            if (v > peakVal) { peakVal = v; peakRow = row; }
        }
        if (peakVal <= params.threshold) continue;

        // Step 2a: Gaussian 3-point log-interpolation (Weber 1995, Eq. 4)
        // Works on the 3 pixels centred on the peak; falls through to CoG on failure.
        if (params.method == PeakMethod::Gaussian &&
            peakRow > 0 && peakRow < height - 1)
        {
            const double i0 = static_cast<double>(
                data[static_cast<std::size_t>((peakRow - 1) * width + col)]);
            const double i1 = static_cast<double>(
                data[static_cast<std::size_t>( peakRow      * width + col)]);
            const double i2 = static_cast<double>(
                data[static_cast<std::size_t>((peakRow + 1) * width + col)]);

            if (i0 > 0.0 && i2 > 0.0) {
                const double li0 = std::log(i0);
                const double li1 = std::log(i1);
                const double li2 = std::log(i2);
                const double denom = li0 - 2.0 * li1 + li2;
                if (denom < -1e-10) {
                    result.rowPositions[static_cast<std::size_t>(col)] =
                        peakRow + 0.5 * (li0 - li2) / denom;
                    ++result.validColumns;
                    continue;
                }
            }
        }

        // Step 2b: Center-of-Gravity (fallback, or explicit CoG method)
        const int rowMin = (params.windowRows > 0)
                         ? std::max(0, peakRow - params.windowRows) : 0;
        const int rowMax = (params.windowRows > 0)
                         ? std::min(height, peakRow + params.windowRows + 1) : height;

        double sumW = 0.0, sumWY = 0.0;
        for (int row = rowMin; row < rowMax; ++row) {
            const double v = static_cast<double>(
                data[static_cast<std::size_t>(row * width + col)]);
            if (v > params.threshold) {
                sumW  += v;
                sumWY += v * row;
            }
        }
        if (sumW > 0.0) {
            result.rowPositions[static_cast<std::size_t>(col)] = sumWY / sumW;
            ++result.validColumns;
        }
    }

    return result;
}

} // namespace scanner
