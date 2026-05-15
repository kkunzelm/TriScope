#include "Triangulator.h"

#include <cmath>
#include <limits>
#include <numeric>

namespace scanner {

// ── Projection ────────────────────────────────────────────────────────────────

PointCloud projectTo3D(const LaserProfile& profile,
                       double xTableMm,
                       const CalibParams& params)
{
    PointCloud cloud;
    cloud.reserve(static_cast<std::size_t>(profile.validRows));

    const int h = profile.frameHeight;
    for (int row = 0; row < h; ++row) {
        const double xPx = profile.colPositions[static_cast<std::size_t>(row)];
        if (xPx < 0.0) continue;

        const double z = (params.x_ref - xPx)                  * params.scale_z;
        const double y = (params.cy    - static_cast<double>(row)) * params.scale_y;

        cloud.emplace_back(xTableMm, y, z);
    }
    return cloud;
}

// ── Direct Z calibration ──────────────────────────────────────────────────────

CalibParams calibrateFromZPoints(std::span<const ZCalibPoint> points,
                                  CalibParams params)
{
    const int n = static_cast<int>(points.size());
    if (n < 2) return params;

    // Least-squares linear fit: col = a + b * z
    // where  a = x_ref,  b = 1 / scale_z
    double sum_z  = 0.0, sum_c  = 0.0;
    double sum_z2 = 0.0, sum_zc = 0.0;
    for (const auto& p : points) {
        sum_z  += p.zMm;
        sum_c  += p.colMean;
        sum_z2 += p.zMm * p.zMm;
        sum_zc += p.zMm * p.colMean;
    }

    const double denom = static_cast<double>(n) * sum_z2 - sum_z * sum_z;
    if (std::abs(denom) < 1e-12) return params;

    const double b = (static_cast<double>(n) * sum_zc - sum_z * sum_c) / denom;
    const double a = (sum_c - b * sum_z) / static_cast<double>(n);

    if (std::abs(b) < 1e-12) return params;

    params.x_ref   = a;
    params.scale_z = 1.0 / b;
    return params;
}

// ── Y calibration ─────────────────────────────────────────────────────────────

CalibParams calibrateFromYEdges(double topRow, double bottomRow,
                                 double knownWidthMm,
                                 CalibParams params)
{
    const double deltaRows = bottomRow - topRow;
    if (deltaRows < 1.0) return params;          // degenerate: edges too close
    params.scale_y = knownWidthMm / deltaRows;
    params.cy      = (topRow + bottomRow) * 0.5;
    return params;
}

// ── Legacy angle-search calibration ──────────────────────────────────────────

namespace {

double evalTheta(double theta, const CalibScan& scan, CalibParams& params)
{
    const double tanTheta = std::tan(theta);
    if (std::abs(tanTheta) < 1e-9) return std::numeric_limits<double>::max();

    std::vector<double> xPxAll;
    const double knownZ = scan.knownHeightMm;

    for (const auto& prof : scan.profiles) {
        for (int row = 0; row < prof.frameHeight; ++row) {
            const double xPx = prof.colPositions[static_cast<std::size_t>(row)];
            if (xPx >= 0.0)
                xPxAll.push_back(xPx);
        }
    }
    if (xPxAll.empty()) return std::numeric_limits<double>::max();

    const double meanXPx = std::accumulate(xPxAll.begin(), xPxAll.end(), 0.0)
                         / static_cast<double>(xPxAll.size());
    params.x_ref = meanXPx + knownZ / params.scale_z;

    double sse = 0.0;
    for (double xp : xPxAll) {
        const double err = (params.x_ref - xp) * params.scale_z - knownZ;
        sse += err * err;
    }
    return std::sqrt(sse / static_cast<double>(xPxAll.size()));
}

} // namespace

CalibParams calibrateAngle(const CalibScan& scan,
                           CalibParams      initial,
                           int              maxIterations,
                           double           tolRad)
{
    constexpr double phi     = 1.6180339887498948482;
    const double     thetaMin = 0.01745329;
    const double     thetaMax = 1.55334303;

    double lo = thetaMin, hi = thetaMax;
    double c  = hi - (hi - lo) / phi;
    double d  = lo + (hi - lo) / phi;

    CalibParams workC = initial, workD = initial;
    double fc = evalTheta(c, scan, workC);
    double fd = evalTheta(d, scan, workD);

    for (int iter = 0; iter < maxIterations && (hi - lo) > tolRad; ++iter) {
        if (fc < fd) {
            hi = d; d = c; fd = fc; workD = workC;
            c  = hi - (hi - lo) / phi;
            fc = evalTheta(c, scan, workC);
        } else {
            lo = c; c = d; fc = fd; workC = workD;
            d  = lo + (hi - lo) / phi;
            fd = evalTheta(d, scan, workD);
        }
    }

    CalibParams best = initial;
    best.theta_rad = (lo + hi) / 2.0;
    evalTheta(best.theta_rad, scan, best);
    best.scale_z = 1.0 / std::tan(best.theta_rad);
    return best;
}

} // namespace scanner
