#pragma once

#include "interfaces/ICameraDevice.h"
#include <peak/peak.hpp>
#include <atomic>
#include <memory>
#include <mutex>

// IDS Peak GenICam camera backend.
// Wraps the peak::core device / DataStream pipeline.
// grabFrame() is safe to call from a thread other than open().
class IDSPeakCamera final : public ICameraDevice
{
    Q_OBJECT
public:
    // deviceName format: "<ModelName>-<SerialNumber>" as returned by CameraDiscovery.
    explicit IDSPeakCamera(const QString &deviceName, QObject *parent = nullptr);
    ~IDSPeakCamera() override;

    bool open()         override;
    void close()        override;
    bool isOpen() const override;

    CameraInfo     info()     const override;
    CameraControls controls() const override;

    bool   setExposure(double microseconds) override;
    bool   setGain(double gain)             override;
    double exposure() const                 override;
    double gain()     const                 override;

    std::optional<QImage> grabFrame(int timeoutMs = 200) override;

private:
    void setupAcquisition();
    void teardownAcquisition();
    void selectBestPixelFormat();
    QImage convertBuffer(const std::shared_ptr<peak::core::Buffer> &buffer);

    QString        m_deviceName;
    CameraInfo     m_info;
    CameraControls m_controls;

    std::shared_ptr<peak::core::Device>      m_device;
    std::shared_ptr<peak::core::NodeMap>     m_nodeMap;
    std::shared_ptr<peak::core::DataStream>  m_dataStream;

    mutable std::mutex m_nodeMutex;   // protects nodeMap access from GUI vs acq threads
    std::atomic<bool>  m_streaming{false};
    bool m_isColor = false;
    int  m_width   = 0;
    int  m_height  = 0;
};
