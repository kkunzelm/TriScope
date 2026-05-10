#pragma once

#include "interfaces/ICameraDevice.h"
#include <atomic>
#include <vector>

// V4L2 (Video4Linux2) camera backend.
// Uses mmap streaming for zero-copy frame transfer.
// grabFrame() is safe to call from a thread other than open().
class V4L2Camera final : public ICameraDevice
{
    Q_OBJECT
public:
    explicit V4L2Camera(const QString &devicePath, QObject *parent = nullptr);
    ~V4L2Camera() override;

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
    // Returns the best pixel format the device supports, in priority order.
    // Priority: MJPEG > YUYV > UYVY > BGR24 > RGB24 > NV12 > YUV420 > GREY.
    uint32_t negotiateFormat();
    bool     setFormat(uint32_t pixfmt, int width, int height);
    bool     initMmap();
    void     deinitMmap();
    bool     startCapture();
    void     stopCapture();
    bool     readControl(int id, int32_t &value) const;
    bool     writeControl(int id, int32_t value);

    QImage convertRaw(const uint8_t *src, size_t len) const;
    QImage yuyv2rgb (const uint8_t *src, int w, int h) const;
    QImage nv12_2rgb(const uint8_t *src, int w, int h) const;

    QString        m_devicePath;
    CameraInfo     m_info;
    CameraControls m_controls;

    int      m_fd          = -1;
    int      m_width       = 0;
    int      m_height      = 0;
    bool     m_isColor     = true;
    uint32_t m_pixelFormat = 0;

    struct Buffer {
        void  *start  = nullptr;
        size_t length = 0;
    };
    std::vector<Buffer> m_buffers;
    std::atomic<bool>   m_streaming{false};
};
