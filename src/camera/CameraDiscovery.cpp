#include "camera/CameraDiscovery.h"
#include "camera/IDSPeakCamera.h"
#include "camera/V4L2Camera.h"

#include <QFileInfo>

#ifdef HAVE_V4L2
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <unistd.h>
#endif

#include <peak/peak.hpp>

CameraDiscovery::CameraDiscovery(QObject *parent)
    : QObject(parent)
{
    try {
        peak::Library::Initialize();
        m_sdkInitialized = true;
    } catch (...) {
        // IDS Peak SDK not available – IDS cameras will not be discovered.
    }
}

QList<CameraInfo> CameraDiscovery::discover()
{
    QList<CameraInfo> result;
    result += discoverV4L2();
    result += discoverIDSPeak();
    return result;
}

QList<CameraInfo> CameraDiscovery::discoverV4L2()
{
    QList<CameraInfo> list;
#ifdef HAVE_V4L2
    // Known video pixel formats (excludes metadata-only subdevices)
    static const uint32_t videoFmts[] = {
        V4L2_PIX_FMT_MJPEG, V4L2_PIX_FMT_JPEG,
        V4L2_PIX_FMT_YUYV,  V4L2_PIX_FMT_UYVY,
        V4L2_PIX_FMT_BGR24,  V4L2_PIX_FMT_RGB24,
        V4L2_PIX_FMT_NV12,   V4L2_PIX_FMT_YUV420,
        V4L2_PIX_FMT_GREY,
    };

    for (int i = 0; i < 16; ++i) {
        const QString path = QStringLiteral("/dev/video%1").arg(i);
        if (!QFileInfo::exists(path))
            continue;

        const int fd = ::open(path.toLocal8Bit().constData(), O_RDWR | O_NONBLOCK);
        if (fd < 0)
            continue;

        v4l2_capability cap{};
        bool ok = (::ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0);

        if (!ok || !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
            ::close(fd);
            continue;
        }

        // Enumerate pixel formats – skip devices that offer no real video format.
        // This filters out metadata subdevices that share the same card name.
        bool hasVideoFormat = false;
        for (uint32_t idx = 0; ; ++idx) {
            v4l2_fmtdesc fmtdesc{};
            fmtdesc.type  = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            fmtdesc.index = idx;
            if (::ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) < 0)
                break;
            for (uint32_t vf : videoFmts) {
                if (fmtdesc.pixelformat == vf) { hasVideoFormat = true; break; }
            }
            if (hasVideoFormat) break;
        }
        ::close(fd);

        if (!hasVideoFormat)
            continue;

        CameraInfo info;
        info.id = QStringLiteral("v4l2:") + path;
        info.displayName = QStringLiteral("%1 (%2)")
            .arg(QString::fromLatin1(reinterpret_cast<const char*>(cap.card)), path);
        info.isColor = true; // actual format negotiated at open
        list.append(info);
    }
#endif
    return list;
}

QList<CameraInfo> CameraDiscovery::discoverIDSPeak()
{
    QList<CameraInfo> list;
    if (!m_sdkInitialized)
        return list;

    try {
        auto &dm = peak::DeviceManager::Instance();
        dm.Update();

        for (const auto &desc : dm.Devices()) {
            const QString model  = QString::fromStdString(desc->ModelName());
            const QString serial = QString::fromStdString(desc->SerialNumber());

            CameraInfo info;
            info.id          = QStringLiteral("ids:") + model + '-' + serial;
            info.displayName = model + QStringLiteral("  SN:") + serial;
            // IDS mono models end in '-M' (e.g. UI-3060CP-M); color in '-C'.
            info.isColor = !model.endsWith("-M", Qt::CaseInsensitive);
            list.append(info);
        }
    } catch (...) {}

    return list;
}

std::unique_ptr<ICameraDevice> CameraDiscovery::createCamera(const QString &id,
                                                              QObject *parent)
{
    if (id.startsWith(QStringLiteral("v4l2:")))
        return std::make_unique<V4L2Camera>(id.mid(5), parent);

    if (id.startsWith(QStringLiteral("ids:")))
        return std::make_unique<IDSPeakCamera>(id.mid(4), parent);

    return nullptr;
}
