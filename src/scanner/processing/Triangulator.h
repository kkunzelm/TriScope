#pragma once

#include "processing/LaserLineExtractor.h"
#include "interfaces/IMotionControl.h"  // for Position3D

#include <Eigen/Core>

#include <span>
#include <vector>

namespace scanner {

// Calibration parameters for the double-telecentric laser triangulation model.
//
// Geometry (Weber 1995, Gl. 1): laser projects a line in the scene Y-Z plane.
// Camera views from triangulation angle Θ. Table moves along world X.
//
//   z_world = (y_ref - row_px) * scale_z    [mm]
//   y_world = (cx    - col_px) * scale_y    [mm]
//   x_world = table_x_mm
//
// scale_z = pixel_pitch_mm / (β · sin Θ)    [mm / pixel]
// scale_y = pixel_pitch_mm / β              [mm / pixel]
// β = magnification of observation optics (constant across field: double telecentric)
// All distances in mm.
struct CalibParams {
    double theta_rad{0.436332};  // triangulation angle Θ (~25°), informational only
    double y_ref{512.0};         // camera row where laser hits z=0 reference plane
    double scale_z{0.050};       // mm / pixel  (depth axis, from Z calibration)
    double scale_y{0.017};       // mm / pixel  (lateral axis, from Y calibration)
    double cx{640.0};            // camera column of optical axis (world Y = 0)
};

// One 3D point from one camera column at one table X position.
using PointCloud = std::vector<Eigen::Vector3d>;

// Project a single laser profile to 3D world coordinates.
// xTableMm: current table X position in software coordinates (mm).
PointCloud projectTo3D(const LaserProfile& profile,
                       double xTableMm,
                       const CalibParams& params);

// ── Direct Z-axis calibration ─────────────────────────────────────────────────

// One measurement point for Z calibration: known Z position → mean laser row.
struct ZCalibPoint {
    double zMm;      // Z position from motion controller (mm, software coords)
    double rowMean;  // mean laser line row position at this Z (subpixel)
};

// Determine scale_z and y_ref from 2+ Z calibration points.
//
// Model: row = y_ref + (1 / scale_z) * z
// Fit by least-squares linear regression. Only scale_z and y_ref are updated.
CalibParams calibrateFromZPoints(std::span<const ZCalibPoint> points,
                                  CalibParams params);

// Determine scale_y and cx from the two edges of a known-width object in the laser plane.
//
// leftCol, rightCol: detected edge column positions (subpixel).
// knownWidthMm: physical width of the calibration object in world Y direction.
// Only scale_y and cx are updated.
CalibParams calibrateFromYEdges(double leftCol, double rightCol,
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
