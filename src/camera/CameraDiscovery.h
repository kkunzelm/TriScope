#pragma once

#include "interfaces/ICameraDevice.h"
#include <QList>
#include <memory>

// Discovers all connected cameras across all supported backends
// (V4L2 and IDS Peak) and acts as a factory for ICameraDevice instances.
class CameraDiscovery : public QObject
{
    Q_OBJECT
public:
    explicit CameraDiscovery(QObject *parent = nullptr);

    // Probe all backends; returns combined list of available cameras.
    QList<CameraInfo> discover();

    // Construct (but do not open) the camera identified by CameraInfo::id.
    std::unique_ptr<ICameraDevice> createCamera(const QString &id,
                                                QObject *parent = nullptr);

private:
    QList<CameraInfo> discoverV4L2();
    QList<CameraInfo> discoverIDSPeak();

    bool m_sdkInitialized = false;
};
