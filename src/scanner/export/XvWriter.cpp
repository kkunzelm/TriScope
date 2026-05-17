#include "XvWriter.h"
#include "io/ViffWriter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace scanner {

void XvWriter::write(std::span<const Eigen::Vector3d> points,
                     const Params& params,
                     const std::filesystem::path& outPath)
{
    error_.clear();

    if (points.empty())
        throw std::runtime_error("XvWriter: empty point cloud");
    if (params.xStepMm <= 0.0 || params.scaleYMm <= 0.0)
        throw std::runtime_error("XvWriter: pixel size parameters must be positive");

    double xMin = std::numeric_limits<double>::max();
    double xMax = std::numeric_limits<double>::lowest();
    double yMin = std::numeric_limits<double>::max();
    double yMax = std::numeric_limits<double>::lowest();

    for (const auto& p : points) {
        xMin = std::min(xMin, p.x());
        xMax = std::max(xMax, p.x());
        yMin = std::min(yMin, p.y());
        yMax = std::max(yMax, p.y());
    }

    const uint32_t nCols = static_cast<uint32_t>(
        std::lround((xMax - xMin) / params.xStepMm)) + 1;
    const uint32_t nRows = static_cast<uint32_t>(
        std::lround((yMax - yMin) / params.scaleYMm)) + 1;

    std::vector<float> grid(
        static_cast<std::size_t>(nRows) * nCols,
        std::numeric_limits<float>::quiet_NaN());

    for (const auto& p : points) {
        const auto col = static_cast<uint32_t>(
            std::lround((p.x() - xMin) / params.xStepMm));
        const auto row = static_cast<uint32_t>(
            std::lround((p.y() - yMin) / params.scaleYMm));
        if (col < nCols && row < nRows)
            grid[static_cast<std::size_t>(row) * nCols + col] =
                static_cast<float>(p.z());
    }

    ViffWriter writer;
    const bool ok = writer.save(
        outPath.string(),
        nRows, nCols,
        grid.data(),
        static_cast<float>(params.xStepMm  * 1e-3),   // mm → m
        static_cast<float>(params.scaleYMm * 1e-3),   // mm → m
        static_cast<float>(xMin),
        static_cast<float>(yMin));

    if (!ok)
        throw std::runtime_error("XvWriter: " + writer.lastError());
}

} // namespace scanner
