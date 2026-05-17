#pragma once

#include "ViffReader.h"  // for ViffHeader and ViffImage
#include <string>

class ViffWriter {
public:
    // Write a ViffImage to path. Always writes Little-Endian (x86 native).
    bool save(const std::string& path, const ViffImage& img);

    // Convenience: write raw float array with explicit dimensions/pixel sizes.
    bool save(const std::string& path,
              uint32_t rows, uint32_t cols,
              const float* data,
              float xPixelSize = 0.0f, float yPixelSize = 0.0f,
              float originX = 0.0f, float originY = 0.0f);

    const std::string& lastError() const { return error_; }

private:
    std::string error_;

    static ViffHeader makeHeader(uint32_t rows, uint32_t cols,
                                 float xPixelSize, float yPixelSize,
                                 float originX, float originY);
};
