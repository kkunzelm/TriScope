#pragma once

#include "processing/Triangulator.h"

#include <filesystem>
#include <span>
#include <string>

namespace scanner {

class XvWriter {
public:
    struct Params {
        double xStepMm  = 0.1;    // stage step size in mm (X pixel pitch)
        double scaleYMm = 0.028;  // camera row pitch in mm (scale_y from calibration)
    };

    // Write point cloud as a VIFF float32 height map (.xv).
    // Throws std::runtime_error on failure.
    void write(std::span<const Eigen::Vector3d> points,
               const Params& params,
               const std::filesystem::path& outPath);

    const std::string& lastError() const { return error_; }

private:
    std::string error_;
};

} // namespace scanner
