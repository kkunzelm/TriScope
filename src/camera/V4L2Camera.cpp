#include "camera/V4L2Camera.h"

#ifdef HAVE_V4L2
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#endif

#include <algorithm>
#include <cstring>

#include <opencv2/imgcodecs.hpp>   // cv::imdecode for MJPEG
#include <opencv2/imgproc.hpp>     // cv::cvtColor

V4L2Camera::V4L2Camera(const QString &devicePath, QObject *parent)
    : ICameraDevice(parent)
    , m_devicePath(devicePath)
{
    m_info.id          = QStringLiteral("v4l2:") + devicePath;
    m_info.displayName = devicePath;
    m_info.isColor     = true;
}

V4L2Camera::~V4L2Camera()
{
    close();
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

bool V4L2Camera::open()
{
    if (m_streaming.load()) return true;

#ifndef HAVE_V4L2
    emit errorOccurred(QStringLiteral("V4L2 not compiled in"));
    return false;
#else
    m_fd = ::open(m_devicePath.toLocal8Bit().constData(), O_RDWR | O_NONBLOCK);
    if (m_fd < 0) {
        emit errorOccurred(QStringLiteral("Cannot open %1: %2")
                           .arg(m_devicePath, QString::fromLocal8Bit(::strerror(errno))));
        return false;
    }

    // Verify capture capability
    v4l2_capability cap{};
    if (::ioctl(m_fd, VIDIOC_QUERYCAP, &cap) < 0) {
        emit errorOccurred(QStringLiteral("VIDIOC_QUERYCAP failed on %1").arg(m_devicePath));
        ::close(m_fd); m_fd = -1;
        return false;
    }
    m_info.displayName = QStringLiteral("%1 (%2)")
        .arg(QString::fromLatin1(reinterpret_cast<const char*>(cap.card)), m_devicePath);

    if (m_info.displayName.contains(QStringLiteral("Grabby"), Qt::CaseInsensitive))
        m_cropBottomRows = 20;

    // Select best format and set it
    m_pixelFormat = negotiateFormat();
    if (m_pixelFormat == 0) {
        emit errorOccurred(QStringLiteral("No supported pixel format on %1").arg(m_devicePath));
        ::close(m_fd); m_fd = -1;
        return false;
    }

    // Try native 5MP resolution first; driver will adjust to nearest supported size
    if (!setFormat(m_pixelFormat, 2592, 1944)) {
        // Fallback to 1080p
        if (!setFormat(m_pixelFormat, 1920, 1080)) {
            setFormat(m_pixelFormat, 1280, 720);
        }
    }

    // Read back actual negotiated size
    {
        v4l2_format fmt{};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (::ioctl(m_fd, VIDIOC_G_FMT, &fmt) == 0) {
            m_width        = static_cast<int>(fmt.fmt.pix.width);
            m_height       = static_cast<int>(fmt.fmt.pix.height);
            m_pixelFormat  = fmt.fmt.pix.pixelformat;
        }
    }

    m_isColor = (m_pixelFormat != V4L2_PIX_FMT_GREY);
    m_info.isColor = m_isColor;

    // Lock down all auto-controls that can cause flicker.
    // Each writeControl() silently ignores EINVAL if the camera lacks a control.
    writeControl(V4L2_CID_EXPOSURE_AUTO, 1);          // V4L2_EXPOSURE_MANUAL = 1
    writeControl(V4L2_CID_EXPOSURE_AUTO_PRIORITY, 0); // prevent driver from varying frame-rate for exposure
    writeControl(V4L2_CID_AUTOGAIN, 0);               // disable auto-gain
    writeControl(V4L2_CID_AUTO_WHITE_BALANCE, 0);     // disable AWB (color shifts look like flicker)
    writeControl(V4L2_CID_BACKLIGHT_COMPENSATION, 0); // disable backlight compensation

    // Query exposure/gain control ranges
    auto queryRange = [&](uint32_t id, double scale,
                          double &mn, double &mx, double &step) {
        v4l2_queryctrl q{};
        q.id = id;
        if (::ioctl(m_fd, VIDIOC_QUERYCTRL, &q) == 0 &&
            !(q.flags & V4L2_CTRL_FLAG_DISABLED)) {
            mn   = q.minimum * scale;
            mx   = q.maximum * scale;
            step = q.step    * scale;
        }
    };
    // V4L2_CID_EXPOSURE_ABSOLUTE is in units of 100 µs
    queryRange(V4L2_CID_EXPOSURE_ABSOLUTE, 100.0,
               m_controls.exposureMin, m_controls.exposureMax, m_controls.exposureStep);
    queryRange(V4L2_CID_GAIN, 1.0,
               m_controls.gainMin, m_controls.gainMax, m_controls.gainStep);

    if (!initMmap() || !startCapture()) {
        ::close(m_fd); m_fd = -1;
        return false;
    }

    m_streaming = true;
    return true;
#endif
}

void V4L2Camera::close()
{
    if (!m_streaming.load()) return;
#ifdef HAVE_V4L2
    stopCapture();
    deinitMmap();
    ::close(m_fd);
    m_fd = -1;
#endif
    m_streaming = false;
}

bool V4L2Camera::isOpen() const { return m_streaming.load(); }
CameraInfo     V4L2Camera::info()     const { return m_info;     }
CameraControls V4L2Camera::controls() const { return m_controls; }

// ---------------------------------------------------------------------------
// Format negotiation
// ---------------------------------------------------------------------------

uint32_t V4L2Camera::negotiateFormat()
{
#ifndef HAVE_V4L2
    return 0;
#else
    // Priority order: compressed first (saves USB bandwidth), then packed, then planar
    static const uint32_t priority[] = {
        V4L2_PIX_FMT_MJPEG,
        V4L2_PIX_FMT_JPEG,
        V4L2_PIX_FMT_YUYV,
        V4L2_PIX_FMT_UYVY,
        V4L2_PIX_FMT_BGR24,
        V4L2_PIX_FMT_RGB24,
        V4L2_PIX_FMT_NV12,
        V4L2_PIX_FMT_YUV420,
        V4L2_PIX_FMT_GREY,
    };

    // Collect what the device offers
    std::vector<uint32_t> offered;
    for (uint32_t idx = 0; ; ++idx) {
        v4l2_fmtdesc fd{};
        fd.type  = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fd.index = idx;
        if (::ioctl(m_fd, VIDIOC_ENUM_FMT, &fd) < 0) break;
        offered.push_back(fd.pixelformat);
    }

    for (uint32_t pref : priority)
        if (std::find(offered.begin(), offered.end(), pref) != offered.end())
            return pref;

    return offered.empty() ? 0 : offered.front();
#endif
}

bool V4L2Camera::setFormat(uint32_t pixfmt, int width, int height)
{
#ifndef HAVE_V4L2
    return false;
#else
    v4l2_format fmt{};
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = static_cast<uint32_t>(width);
    fmt.fmt.pix.height      = static_cast<uint32_t>(height);
    fmt.fmt.pix.pixelformat = pixfmt;
    fmt.fmt.pix.field       = V4L2_FIELD_ANY;
    return ::ioctl(m_fd, VIDIOC_S_FMT, &fmt) == 0;
#endif
}

// ---------------------------------------------------------------------------
// mmap buffer management
// ---------------------------------------------------------------------------

bool V4L2Camera::initMmap()
{
#ifndef HAVE_V4L2
    return false;
#else
    v4l2_requestbuffers req{};
    req.count  = 4;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (::ioctl(m_fd, VIDIOC_REQBUFS, &req) < 0 || req.count < 2) {
        emit errorOccurred(QStringLiteral("VIDIOC_REQBUFS failed"));
        return false;
    }

    m_buffers.resize(req.count);
    for (size_t i = 0; i < req.count; ++i) {
        v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = static_cast<__u32>(i);
        ::ioctl(m_fd, VIDIOC_QUERYBUF, &buf);

        m_buffers[i].length = buf.length;
        m_buffers[i].start  = ::mmap(nullptr, buf.length,
                                      PROT_READ | PROT_WRITE, MAP_SHARED,
                                      m_fd, buf.m.offset);
        if (m_buffers[i].start == MAP_FAILED) {
            emit errorOccurred(QStringLiteral("mmap failed for buffer %1").arg(i));
            return false;
        }
    }
    return true;
#endif
}

void V4L2Camera::deinitMmap()
{
#ifdef HAVE_V4L2
    for (auto &b : m_buffers)
        if (b.start && b.start != MAP_FAILED)
            ::munmap(b.start, b.length);
    m_buffers.clear();
#endif
}

bool V4L2Camera::startCapture()
{
#ifndef HAVE_V4L2
    return false;
#else
    for (size_t i = 0; i < m_buffers.size(); ++i) {
        v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = static_cast<__u32>(i);
        if (::ioctl(m_fd, VIDIOC_QBUF, &buf) < 0) return false;
    }
    const v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    return ::ioctl(m_fd, VIDIOC_STREAMON, &type) == 0;
#endif
}

void V4L2Camera::stopCapture()
{
#ifdef HAVE_V4L2
    const v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ::ioctl(m_fd, VIDIOC_STREAMOFF, &type);
#endif
}

// ---------------------------------------------------------------------------
// V4L2 controls
// ---------------------------------------------------------------------------

bool V4L2Camera::readControl(int id, int32_t &value) const
{
#ifdef HAVE_V4L2
    v4l2_control ctrl{};
    ctrl.id = static_cast<uint32_t>(id);
    if (::ioctl(m_fd, VIDIOC_G_CTRL, &ctrl) == 0) {
        value = ctrl.value;
        return true;
    }
#endif
    return false;
}

bool V4L2Camera::writeControl(int id, int32_t value)
{
#ifdef HAVE_V4L2
    v4l2_control ctrl{};
    ctrl.id    = static_cast<uint32_t>(id);
    ctrl.value = value;
    return ::ioctl(m_fd, VIDIOC_S_CTRL, &ctrl) == 0;
#else
    return false;
#endif
}

bool V4L2Camera::setExposure(double microseconds)
{
#ifdef HAVE_V4L2
    writeControl(V4L2_CID_EXPOSURE_AUTO, 1); // V4L2_EXPOSURE_MANUAL = 1
    // V4L2_CID_EXPOSURE_ABSOLUTE is in units of 100 µs
    return writeControl(V4L2_CID_EXPOSURE_ABSOLUTE,
                        static_cast<int32_t>(microseconds / 100.0));
#else
    return false;
#endif
}

bool V4L2Camera::setGain(double gain)
{
#ifdef HAVE_V4L2
    return writeControl(V4L2_CID_GAIN, static_cast<int32_t>(gain));
#else
    return false;
#endif
}

double V4L2Camera::exposure() const
{
    int32_t v = 0;
#ifdef HAVE_V4L2
    return readControl(V4L2_CID_EXPOSURE_ABSOLUTE, v) ? v * 100.0 : 0.0;
#else
    return 0.0;
#endif
}

double V4L2Camera::gain() const
{
    int32_t v = 0;
#ifdef HAVE_V4L2
    return readControl(V4L2_CID_GAIN, v) ? static_cast<double>(v) : 0.0;
#else
    return 0.0;
#endif
}

// ---------------------------------------------------------------------------
// Pixel format conversions
// ---------------------------------------------------------------------------

// YUYV (YUY2) packed 4:2:2 → RGB888, ITU-R BT.601
QImage V4L2Camera::yuyv2rgb(const uint8_t *src, int w, int h) const
{
    QImage img(w, h, QImage::Format_RGB888);
    for (int row = 0; row < h; ++row) {
        const uint8_t *s = src + row * w * 2;
        uint8_t *d = img.scanLine(row);
        for (int col = 0; col < w; col += 2) {
            const int Y0 = s[0], U = s[1] - 128, Y1 = s[2], V = s[3] - 128;
            s += 4;
            auto clamp = [](int v) { return static_cast<uint8_t>(std::clamp(v, 0, 255)); };
            d[0] = clamp(Y0 + (1402 * V) / 1000);
            d[1] = clamp(Y0 - (344 * U + 714 * V) / 1000);
            d[2] = clamp(Y0 + (1772 * U) / 1000);
            d[3] = clamp(Y1 + (1402 * V) / 1000);
            d[4] = clamp(Y1 - (344 * U + 714 * V) / 1000);
            d[5] = clamp(Y1 + (1772 * U) / 1000);
            d += 6;
        }
    }
    return img;
}

// UYVY packed 4:2:2 → RGB888 (swapped byte order vs YUYV)
static QImage uyvy2rgb(const uint8_t *src, int w, int h)
{
    QImage img(w, h, QImage::Format_RGB888);
    for (int row = 0; row < h; ++row) {
        const uint8_t *s = src + row * w * 2;
        uint8_t *d = img.scanLine(row);
        for (int col = 0; col < w; col += 2) {
            const int U = s[0] - 128, Y0 = s[1], V = s[2] - 128, Y1 = s[3];
            s += 4;
            auto clamp = [](int v) { return static_cast<uint8_t>(std::clamp(v, 0, 255)); };
            d[0] = clamp(Y0 + (1402 * V) / 1000);
            d[1] = clamp(Y0 - (344 * U + 714 * V) / 1000);
            d[2] = clamp(Y0 + (1772 * U) / 1000);
            d[3] = clamp(Y1 + (1402 * V) / 1000);
            d[4] = clamp(Y1 - (344 * U + 714 * V) / 1000);
            d[5] = clamp(Y1 + (1772 * U) / 1000);
            d += 6;
        }
    }
    return img;
}

// NV12 (YUV 4:2:0 semi-planar) → RGB888
QImage V4L2Camera::nv12_2rgb(const uint8_t *src, int w, int h) const
{
    QImage img(w, h, QImage::Format_RGB888);
    const uint8_t *yPlane  = src;
    const uint8_t *uvPlane = src + w * h;
    auto clamp = [](int v) { return static_cast<uint8_t>(std::clamp(v, 0, 255)); };

    for (int row = 0; row < h; ++row) {
        uint8_t *d = img.scanLine(row);
        for (int col = 0; col < w; ++col) {
            const int Y = yPlane[row * w + col];
            const int U = static_cast<int>(uvPlane[(row / 2) * w + (col & ~1)])     - 128;
            const int V = static_cast<int>(uvPlane[(row / 2) * w + (col & ~1) + 1]) - 128;
            d[0] = clamp(Y + (1402 * V) / 1000);
            d[1] = clamp(Y - (344 * U + 714 * V) / 1000);
            d[2] = clamp(Y + (1772 * U) / 1000);
            d += 3;
        }
    }
    return img;
}

QImage V4L2Camera::convertRaw(const uint8_t *src, size_t len) const
{
#ifdef HAVE_V4L2
    switch (m_pixelFormat) {
    case V4L2_PIX_FMT_MJPEG:
    case V4L2_PIX_FMT_JPEG: {
        // Decode JPEG/MJPEG via OpenCV (handles any subsampling or quality)
        const std::vector<uint8_t> buf(src, src + len);
        const cv::Mat decoded = cv::imdecode(buf, cv::IMREAD_COLOR); // BGR
        if (decoded.empty()) return {};
        QImage img(decoded.cols, decoded.rows, QImage::Format_RGB888);
        for (int r = 0; r < decoded.rows; ++r) {
            const uint8_t *s = decoded.ptr(r);
            uint8_t *d = img.scanLine(r);
            for (int c = 0; c < decoded.cols; ++c) {
                d[c * 3 + 0] = s[c * 3 + 2]; // R
                d[c * 3 + 1] = s[c * 3 + 1]; // G
                d[c * 3 + 2] = s[c * 3 + 0]; // B
            }
        }
        return img;
    }
    case V4L2_PIX_FMT_YUYV:
        return yuyv2rgb(src, m_width, m_height);
    case V4L2_PIX_FMT_UYVY:
        return uyvy2rgb(src, m_width, m_height);
    case V4L2_PIX_FMT_BGR24: {
        QImage img(m_width, m_height, QImage::Format_RGB888);
        for (int r = 0; r < m_height; ++r) {
            const uint8_t *s = src + r * m_width * 3;
            uint8_t *d = img.scanLine(r);
            for (int c = 0; c < m_width; ++c) {
                d[c*3+0] = s[c*3+2];
                d[c*3+1] = s[c*3+1];
                d[c*3+2] = s[c*3+0];
            }
        }
        return img;
    }
    case V4L2_PIX_FMT_RGB24: {
        QImage img(m_width, m_height, QImage::Format_RGB888);
        for (int r = 0; r < m_height; ++r)
            std::memcpy(img.scanLine(r), src + r * m_width * 3,
                        static_cast<size_t>(m_width * 3));
        return img;
    }
    case V4L2_PIX_FMT_NV12:
        return nv12_2rgb(src, m_width, m_height);
    case V4L2_PIX_FMT_YUV420: {
        // YUV420 planar: Y plane + U plane + V plane (each at half resolution)
        QImage img(m_width, m_height, QImage::Format_RGB888);
        const uint8_t *yp = src;
        const uint8_t *up = src + m_width * m_height;
        const uint8_t *vp = up + (m_width * m_height) / 4;
        auto clamp = [](int v) { return static_cast<uint8_t>(std::clamp(v, 0, 255)); };
        for (int r = 0; r < m_height; ++r) {
            uint8_t *d = img.scanLine(r);
            for (int c = 0; c < m_width; ++c) {
                const int Y = yp[r * m_width + c];
                const int U = static_cast<int>(up[(r/2) * (m_width/2) + c/2]) - 128;
                const int V = static_cast<int>(vp[(r/2) * (m_width/2) + c/2]) - 128;
                d[0] = clamp(Y + (1402*V)/1000);
                d[1] = clamp(Y - (344*U + 714*V)/1000);
                d[2] = clamp(Y + (1772*U)/1000);
                d += 3;
            }
        }
        return img;
    }
    case V4L2_PIX_FMT_GREY: {
        QImage img(m_width, m_height, QImage::Format_Grayscale8);
        for (int r = 0; r < m_height; ++r)
            std::memcpy(img.scanLine(r), src + r * m_width,
                        static_cast<size_t>(m_width));
        return img;
    }
    default:
        return {};
    }
#else
    return {};
#endif
}

// ---------------------------------------------------------------------------
// Frame grab
// ---------------------------------------------------------------------------

std::optional<QImage> V4L2Camera::grabFrame(int timeoutMs)
{
    if (!m_streaming.load() || m_fd < 0) return std::nullopt;

#ifndef HAVE_V4L2
    return std::nullopt;
#else
    pollfd pfd{m_fd, POLLIN, 0};
    if (::poll(&pfd, 1, timeoutMs) <= 0)
        return std::nullopt;

    v4l2_buffer buf{};
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (::ioctl(m_fd, VIDIOC_DQBUF, &buf) < 0)
        return std::nullopt;

    const auto *src = static_cast<const uint8_t*>(m_buffers[buf.index].start);
    const size_t len = buf.bytesused; // actual filled bytes (important for MJPEG)

    QImage img = convertRaw(src, len);

    ::ioctl(m_fd, VIDIOC_QBUF, &buf);

    if (img.isNull()) return std::nullopt;
    if (m_cropBottomRows > 0 && img.height() > m_cropBottomRows)
        img = img.copy(0, 0, img.width(), img.height() - m_cropBottomRows);
    return img;
#endif
}
