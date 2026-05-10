#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace scanner {

enum class AcquisitionMode {
    Continuous,      // camera streams freely; caller must tolerate stale frames
    SoftwareTrigger, // camera waits for explicit trigger per frame (stop-and-go)
};

struct CameraParams {
    double exposureUs{500.0};
    double gainDb{0.0};
    // ROI: width=0 means full sensor
    int roiOffsetY{0};
    int roiHeight{0};
    AcquisitionMode mode{AcquisitionMode::SoftwareTrigger};
};

// Frame data is always 16-bit monochrome, row-major.
// IDSCamera expands Mono8→16 (×256) and Mono12→16 (<<4) before returning.
struct Frame {
    std::vector<uint16_t> data;
    int width{0};
    int height{0};

    bool valid() const noexcept { return !data.empty() && width > 0 && height > 0; }

    uint16_t pixel(int col, int row) const noexcept {
        return data[static_cast<std::size_t>(row * width + col)];
    }
};

class ICamera {
public:
    virtual ~ICamera() = default;

    // Open first available camera, or the one matching deviceName if non-empty.
    virtual bool open(const std::string& deviceName = "") = 0;

    virtual bool configure(const CameraParams& params) = 0;

    virtual bool startAcquisition() = 0;

    // Returns an empty Frame on timeout or error.
    virtual Frame captureFrame(int timeoutMs = 5000) = 0;

    virtual void stopAcquisition() = 0;

    virtual void close() = 0;

    virtual int sensorWidth() const = 0;
    virtual int sensorHeight() const = 0;
};

} // namespace scanner
