#pragma once

#include <concepts>
#include <string>

namespace scanner {

struct Position3D {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

// All coordinates are in mm, using right-handed software convention (ISO 841):
//   +X = scan direction, +Y = laser line width, +Z = up (Z=0 at home/top)
class IMotionControl {
public:
    virtual ~IMotionControl() = default;

    // Connect to controller and run mandatory init sequence.
    virtual bool initialize(const std::string& portName) = 0;

    // Home all axes (Calibrate) then measure travel range. Required before scanning.
    virtual bool calibrate() = 0;

    // Move all three axes to the given software-coordinate position (mm).
    // Blocks until motion complete or timeout.
    virtual bool moveAbsolute(Position3D target) = 0;

    // Move X axis only by deltaX mm relative to current position.
    // Positive deltaX = forward scan direction.
    virtual bool moveRelativeX(double deltaXMm) = 0;

    // Returns last known position in software coordinates (mm).
    virtual Position3D currentPosition() const = 0;

    // Returns maximum travel in software coordinates (mm).
    virtual Position3D travelRange() const = 0;

    // Send emergency stop. Does not wait for response.
    virtual void abort() = 0;

    // True when no motion command is pending.
    virtual bool isIdle() const = 0;
};

} // namespace scanner
