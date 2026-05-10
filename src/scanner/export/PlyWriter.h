#pragma once

#include "processing/Triangulator.h"

#include <filesystem>
#include <span>

namespace scanner {

enum class PlyFormat { Ascii, BinaryLittleEndian };

class PlyWriter {
public:
    // Write point cloud to a PLY file.
    // Throws std::runtime_error if the file cannot be opened.
    void write(std::span<const Eigen::Vector3d> points,
               const std::filesystem::path& outPath,
               PlyFormat format);

private:
    void writeHeader(std::ostream& os, std::size_t count, PlyFormat fmt);
    void writeAscii(std::ostream& os, std::span<const Eigen::Vector3d> pts);
    void writeBinary(std::ostream& os, std::span<const Eigen::Vector3d> pts);
};

} // namespace scanner
