#pragma once

#include <QThread>
#include <QImage>
#include <QMutex>
#include <QWaitCondition>
#include <atomic>

class ICameraDevice;

// Runs the camera grab loop in a dedicated OS thread.
// The GUI never blocks on frame delivery; it receives frameReady() via
// Qt's queued-connection mechanism (different threads → auto-queued).
class AcquisitionThread : public QThread
{
    Q_OBJECT
public:
    explicit AcquisitionThread(QObject *parent = nullptr);
    ~AcquisitionThread() override;

    void setCamera(ICameraDevice *camera);  // call before startAcquisition
    void startAcquisition();
    void stopAcquisition();
    bool isGrabbing() const { return m_running.load(); }

signals:
    void frameReady(const QImage &frame);
    void acquisitionError(const QString &message);

protected:
    void run() override;

private:
    ICameraDevice      *m_camera    = nullptr;
    bool                m_shouldRun = false;
    std::atomic<bool>   m_running{false};
    QMutex              m_mutex;
    QWaitCondition      m_cond;
};
