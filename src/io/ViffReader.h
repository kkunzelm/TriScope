#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#pragma pack(push, 1)
struct ViffHeader {
    uint8_t  fileId;
    uint8_t  fileType;
    uint8_t  release;
    uint8_t  version;
    uint8_t  machineDep;
    uint8_t  padding[3];
    char     comment[512];
    uint32_t numberOfRows;      // Khoros: image width  (not height!)
    uint32_t numberOfColumns;   // Khoros: image height (not width!)
    uint32_t lengthOfSubrow;
    int32_t  startX;
    int32_t  startY;
    float    xPixelSize;        // meters
    float    yPixelSize;        // meters
    uint32_t locationType;
    uint32_t locationDim;
    uint32_t numberOfImages;
    uint32_t numberOfBands;
    uint32_t dataStorageType;
    uint32_t dataEncodingScheme;
    uint32_t mapScheme;
    uint32_t mapStorageType;
    uint32_t mapRowSize;
    uint32_t mapColumnSize;
    uint32_t mapSubrowSize;
    uint32_t mapEnable;
    uint32_t mapsPerCycle;
    uint32_t colorSpaceModel;
    uint32_t iSpare1;
    uint32_t iSpare2;
    float    fSpare1;           // originX (mm)
    float    fSpare2;           // originY (mm)
    uint8_t  reserve[404];
};
#pragma pack(pop)
static_assert(sizeof(ViffHeader) == 1024, "ViffHeader must be exactly 1024 bytes");

struct ViffImage {
    uint32_t rows       = 0;      // image height (camera Y rows)
    uint32_t cols       = 0;      // image width  (scan X steps)
    float    xPixelSize = 0.0f;   // meters
    float    yPixelSize = 0.0f;   // meters
    float    originX    = 0.0f;   // mm
    float    originY    = 0.0f;   // mm
    std::vector<float> data;      // NaN = no data

    float    at(uint32_t r, uint32_t c) const { return data[r * cols + c]; }
    bool     isValid(uint32_t r, uint32_t c) const { return !std::isnan(at(r, c)); }
    uint32_t totalPixels() const { return rows * cols; }
};

class ViffReader {
public:
    bool load(const std::string& path, ViffImage& img);
    const std::string& lastError() const { return error_; }

private:
    std::string error_;
};
