#include "ViffWriter.h"

#include <cstring>
#include <fstream>

ViffHeader ViffWriter::makeHeader(uint32_t rows, uint32_t cols,
                                   float xPixelSize, float yPixelSize,
                                   float originX, float originY) {
    ViffHeader h;
    std::memset(&h, 0, sizeof(h));

    h.fileId             = 0xAB;
    h.fileType           = 0x01;
    h.release            = 0x03;
    h.version            = 0x01;
    h.machineDep         = 0x08;  // Little-Endian (x86)
    // Khoros convention: numberOfRows = width, numberOfColumns = height.
    h.numberOfRows       = cols;    // width
    h.numberOfColumns    = rows;    // height
    h.numberOfBands      = 1;
    h.numberOfImages     = 1;
    h.dataStorageType    = 0x05;  // float32
    h.dataEncodingScheme = 0x00;  // uncompressed
    h.locationType       = 0x01;  // implicit locations
    h.xPixelSize         = xPixelSize;
    h.yPixelSize         = yPixelSize;
    h.fSpare1            = originX;
    h.fSpare2            = originY;
    std::strncpy(h.comment, "match3d_v2", sizeof(h.comment) - 1);

    return h;
}

bool ViffWriter::save(const std::string& path, const ViffImage& img) {
    return save(path, img.rows, img.cols, img.data.data(),
                img.xPixelSize, img.yPixelSize,
                img.originX, img.originY);
}

bool ViffWriter::save(const std::string& path,
                      uint32_t rows, uint32_t cols,
                      const float* data,
                      float xPixelSize, float yPixelSize,
                      float originX, float originY) {
    error_.clear();

    if (rows == 0 || cols == 0 || data == nullptr) {
        error_ = "Invalid arguments (zero dimensions or null data)";
        return false;
    }

    std::ofstream f(path, std::ios::binary);
    if (!f) {
        error_ = "Cannot create file: " + path;
        return false;
    }

    ViffHeader h = makeHeader(rows, cols, xPixelSize, yPixelSize, originX, originY);

    f.write(reinterpret_cast<const char*>(&h), sizeof(h));
    f.write(reinterpret_cast<const char*>(data),
            static_cast<std::streamsize>(rows) * cols * sizeof(float));

    if (!f) {
        error_ = "Write error";
        return false;
    }
    return true;
}
