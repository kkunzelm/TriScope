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
    cloud.reserve(static_cast<std::size_t>(profile.validColumns));

    const int w = profile.frameWidth;
    for (int col = 0; col < w; ++col) {
        const double yPx = profile.rowPositions[static_cast<std::size_t>(col)];
        if (yPx < 0.0) continue;

        const double z = (params.y_ref - yPx) * params.scale_z;
        const double y = (static_cast<double>(col) - params.cx) * params.scale_y;

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

    // Least-squares linear fit: row = a + b * z
    // where  a = y_ref,  b = 1 / scale_z
    double sum_z  = 0.0, sum_r  = 0.0;
    double sum_z2 = 0.0, sum_zr = 0.0;
    for (const auto& p : points) {
        sum_z  += p.zMm;
        sum_r  += p.rowMean;
        sum_z2 += p.zMm * p.zMm;
        sum_zr += p.zMm * p.rowMean;
    }

    const double denom = static_cast<double>(n) * sum_z2 - sum_z * sum_z;
    if (std::abs(denom) < 1e-12) return params;

    const double b = (static_cast<double>(n) * sum_zr - sum_z * sum_r) / denom;
    const double a = (sum_r - b * sum_z) / static_cast<double>(n);

    if (std::abs(b) < 1e-12) return params;

    params.y_ref   = a;
    params.scale_z = 1.0 / b;   // positive: row increases as z increases (CCW rotation)
    return params;
}

// ── Y calibration ─────────────────────────────────────────────────────────────

CalibParams calibrateFromYEdges(double leftCol, double rightCol,
                                 double knownWidthMm,
                                 CalibParams params)
{
    const double deltaCols = rightCol - leftCol;
    if (deltaCols < 1.0) return params;          // degenerate: edges too close
    params.scale_y = knownWidthMm / deltaCols;
    params.cx      = (leftCol + rightCol) * 0.5;
    return params;
}

// ── Legacy angle-search calibration ──────────────────────────────────────────

namespace {

double evalTheta(double theta, const CalibScan& scan, CalibParams& params)
{
    const double tanTheta = std::tan(theta);
    if (std::abs(tanTheta) < 1e-9) return std::numeric_limits<double>::max();

    std::vector<double> yPxAll;
    const double knownZ = scan.knownHeightMm;

    for (const auto& prof : scan.profiles) {
        for (int col = 0; col < prof.frameWidth; ++col) {
            const double yPx = prof.rowPositions[static_cast<std::size_t>(col)];
            if (yPx >= 0.0)
                yPxAll.push_back(yPx);
        }
    }
    if (yPxAll.empty()) return std::numeric_limits<double>::max();

    const double meanYPx = std::accumulate(yPxAll.begin(), yPxAll.end(), 0.0)
                         / static_cast<double>(yPxAll.size());
    params.y_ref = meanYPx + knownZ / params.scale_z;

    double sse = 0.0;
    for (double yp : yPxAll) {
        const double err = (params.y_ref - yp) * params.scale_z - knownZ;
        sse += err * err;
    }
    return std::sqrt(sse / static_cast<double>(yPxAll.size()));
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
