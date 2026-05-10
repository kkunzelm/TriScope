#pragma once

#include <QObject>
#include <QImage>
#include <QString>
#include <optional>

struct CameraInfo {
    QString id;           // unique key: "v4l2:/dev/video0" or "ids:ModelName-Serial"
    QString displayName;
    bool    isColor = false;
};

struct CameraControls {
    double exposureMin  = 0.0;   // µs
    double exposureMax  = 0.0;   // µs
    double exposureStep = 1.0;   // µs
    double gainMin      = 0.0;
    double gainMax      = 1.0;
    double gainStep     = 0.01;
};

// Hardware-independent camera interface.
// Concrete implementations: IDSPeakCamera, V4L2Camera.
// grabFrame() is designed to be called from a dedicated acquisition thread.
class ICameraDevice : public QObject
{
    Q_OBJECT
public:
    explicit ICameraDevice(QObject *parent = nullptr) : QObject(parent) {}
    ~ICameraDevice() override = default;

    virtual bool open()         = 0;
    virtual void close()        = 0;
    virtual bool isOpen() const = 0;

    virtual CameraInfo     info()     const = 0;
    virtual CameraControls controls() const = 0;

    virtual bool   setExposure(double microseconds) = 0;
    virtual bool   setGain(double gain)             = 0;
    virtual double exposure() const                 = 0;
    virtual double gain()     const                 = 0;

    // Returns the next frame or std::nullopt on timeout / camera error.
    // Safe to call from a thread other than the one that called open().
    virtual std::optional<QImage> grabFrame(int timeoutMs = 200) = 0;

signals:
    void errorOccurred(const QString &message);
};
