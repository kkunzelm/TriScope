#include "PlyWriter.h"

#include <bit>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace scanner {

void PlyWriter::write(std::span<const Eigen::Vector3d> points,
                      const std::filesystem::path& outPath,
                      PlyFormat format)
{
    const auto mode = (format == PlyFormat::BinaryLittleEndian)
                      ? std::ios::binary | std::ios::out
                      : std::ios::out;

    std::ofstream ofs(outPath, mode);
    if (!ofs)
        throw std::runtime_error("PlyWriter: cannot open " + outPath.string());

    writeHeader(ofs, points.size(), format);

    if (format == PlyFormat::Ascii)
        writeAscii(ofs, points);
    else
        writeBinary(ofs, points);
}

void PlyWriter::writeHeader(std::ostream& os, std::size_t count, PlyFormat fmt)
{
    const char* fmtStr = (fmt == PlyFormat::Ascii)
                         ? "ascii 1.0"
                         : "binary_little_endian 1.0";
    os << "ply\n"
       << "format " << fmtStr << "\n"
       << "element vertex " << count << "\n"
       << "property float x\n"
       << "property float y\n"
       << "property float z\n"
       << "end_header\n";
}

void PlyWriter::writeAscii(std::ostream& os, std::span<const Eigen::Vector3d> pts)
{
    os.precision(6);
    for (const auto& p : pts)
        os << static_cast<float>(p.x()) << ' '
           << static_cast<float>(p.y()) << ' '
           << static_cast<float>(p.z()) << '\n';
}

void PlyWriter::writeBinary(std::ostream& os, std::span<const Eigen::Vector3d> pts)
{
    // PLY binary_little_endian: 3 × float32 per vertex
    for (const auto& p : pts) {
        const float xyz[3] = {
            static_cast<float>(p.x()),
            static_cast<float>(p.y()),
            static_cast<float>(p.z())
        };

        // Ensure little-endian byte order regardless of host endianness
        if constexpr (std::endian::native == std::endian::little) {
            os.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
        } else {
            for (float v : xyz) {
                uint32_t bits;
                std::memcpy(&bits, &v, 4);
                bits = __builtin_bswap32(bits);
                os.write(reinterpret_cast<const char*>(&bits), 4);
            }
        }
    }
}

} // namespace scanner
