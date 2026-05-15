#pragma once

#include "processing/LaserLineExtractor.h"
#include "interfaces/IMotionControl.h"  // for Position3D

#include <Eigen/Core>

#include <span>
#include <vector>

namespace scanner {

// Calibration parameters for the double-telecentric laser triangulation model.
//
// The laser line runs vertically in the raw camera image (parallel to world Y).
// Z-displacement shifts the laser stripe horizontally (column direction).
// All parameters are defined in the raw camera frame — no rotation applied.
//
//   z_world = (col_px - x_ref) * scale_z    [mm]
//   y_world = (cy    - row_px) * scale_y    [mm]
//   x_world = table_x_mm
//
// scale_z = pixel_pitch_mm / (β · sin Θ)    [mm / pixel]
// scale_y = pixel_pitch_mm / β              [mm / pixel]
// β = magnification of observation optics (constant across field: double telecentric)
// All distances in mm.
struct CalibParams {
    double theta_rad{0.436332};  // triangulation angle Θ (~25°), informational only
    double x_ref{640.0};         // camera column where laser hits z=0 reference plane
    double scale_z{0.050};       // mm / pixel  (depth axis, from Z calibration)
    double scale_y{0.017};       // mm / pixel  (lateral axis, from Y calibration)
    double cy{512.0};            // camera row of optical axis (world Y = 0)
};

// One 3D point from one camera row at one table X position.
using PointCloud = std::vector<Eigen::Vector3d>;

// Project a single laser profile to 3D world coordinates.
// xTableMm: current table X position in software coordinates (mm).
PointCloud projectTo3D(const LaserProfile& profile,
                       double xTableMm,
                       const CalibParams& params);

// ── Direct Z-axis calibration ─────────────────────────────────────────────────

// One measurement point for Z calibration: known Z position → mean laser column.
struct ZCalibPoint {
    double zMm;      // Z position from motion controller (mm, software coords)
    double colMean;  // mean laser line column position at this Z (subpixel)
};

// Determine scale_z and x_ref from 2+ Z calibration points.
//
// Model: col = x_ref + z / scale_z   (col increases as z increases for vertical line)
// Fit by least-squares linear regression. Only scale_z and x_ref are updated.
CalibParams calibrateFromZPoints(std::span<const ZCalibPoint> points,
                                  CalibParams params);

// Determine scale_y and cy from the two edges of a known-width object in the laser plane.
//
// topRow, bottomRow: detected edge row positions (subpixel), top < bottom in image.
// knownWidthMm: physical width of the calibration object in world Y direction.
// Only scale_y and cy are updated.
CalibParams calibrateFromYEdges(double topRow, double bottomRow,
                                 double knownWidthMm,
                                 CalibParams params);

// ── Legacy angle-search calibration ──────────────────────────────────────────

// One calibration measurement: profiles of a flat surface at a known height.
struct CalibScan {
    std::vector<LaserProfile> profiles;
    std::vector<double>       xPositionsMm;
    double                    knownHeightMm;
};

// Golden-section search over Θ to minimise RMS deviation from a flat plane.
CalibParams calibrateAngle(const CalibScan& scan,
                           CalibParams      initial,
                           int              maxIterations = 200,
                           double           tolRad        = 1e-6);

} // namespace scanner
