#include "ViffReader.h"

#include <bit>
#include <cstring>
#include <fstream>

static uint32_t bswap32(uint32_t v)
{
    return ((v & 0xFF000000u) >> 24) | ((v & 0x00FF0000u) >>  8) |
           ((v & 0x0000FF00u) <<  8) | ((v & 0x000000FFu) << 24);
}

static float bswapf(float v)
{
    uint32_t bits;
    std::memcpy(&bits, &v, 4);
    bits = bswap32(bits);
    std::memcpy(&v, &bits, 4);
    return v;
}

bool ViffReader::load(const std::string& path, ViffImage& img)
{
    error_.clear();

    std::ifstream f(path, std::ios::binary);
    if (!f) {
        error_ = "Cannot open file: " + path;
        return false;
    }

    ViffHeader h;
    f.read(reinterpret_cast<char*>(&h), sizeof(h));
    if (!f || f.gcount() != static_cast<std::streamsize>(sizeof(h))) {
        error_ = "Failed to read header";
        return false;
    }

    if (h.fileId != 0xAB || h.fileType != 0x01) {
        error_ = "Not a VIFF file (bad magic bytes)";
        return false;
    }

    if (h.dataStorageType != 0x05) {
        error_ = "Unsupported data type (only float32 / 0x05 supported)";
        return false;
    }

    const bool needSwap = (h.machineDep == 0x02) &&
                          (std::endian::native == std::endian::little);

    auto u32 = [&](uint32_t v) -> uint32_t { return needSwap ? bswap32(v) : v; };
    auto f32 = [&](float    v) -> float    { return needSwap ? bswapf(v)  : v; };

    // Khoros convention: numberOfRows = width, numberOfColumns = height
    img.cols        = u32(h.numberOfRows);
    img.rows        = u32(h.numberOfColumns);
    img.xPixelSize  = f32(h.xPixelSize);
    img.yPixelSize  = f32(h.yPixelSize);
    img.originX     = f32(h.fSpare1);
    img.originY     = f32(h.fSpare2);

    if (img.rows == 0 || img.cols == 0) {
        error_ = "Zero-size image";
        return false;
    }

    const auto n = static_cast<std::size_t>(img.rows) * img.cols;
    img.data.resize(n);
    f.read(reinterpret_cast<char*>(img.data.data()),
           static_cast<std::streamsize>(n * sizeof(float)));
    if (!f) {
        error_ = "Truncated pixel data";
        return false;
    }

    if (needSwap)
        for (float& v : img.data) v = bswapf(v);

    return true;
}
