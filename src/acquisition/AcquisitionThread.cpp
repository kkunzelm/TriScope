#include "acquisition/AcquisitionThread.h"
#include "interfaces/ICameraDevice.h"

AcquisitionThread::AcquisitionThread(QObject *parent)
    : QThread(parent)
{}

AcquisitionThread::~AcquisitionThread()
{
    stopAcquisition();
    requestInterruption();
    m_cond.wakeAll();
    wait();
}

void AcquisitionThread::setCamera(ICameraDevice *camera)
{
    QMutexLocker locker(&m_mutex);
    m_camera = camera;
}

void AcquisitionThread::startAcquisition()
{
    {
        QMutexLocker locker(&m_mutex);
        m_shouldRun = true;
    }
    m_cond.wakeAll();
    if (!isRunning())
        start(QThread::HighPriority);
}

void AcquisitionThread::stopAcquisition()
{
    {
        QMutexLocker locker(&m_mutex);
        m_shouldRun = false;
    }
    m_running = false;
    m_cond.wakeAll();
}

void AcquisitionThread::run()
{
    m_mutex.lock();

    while (!isInterruptionRequested()) {
        // Wait until startAcquisition() is called
        while (!m_shouldRun && !isInterruptionRequested())
            m_cond.wait(&m_mutex, 100 /*ms*/);

        if (isInterruptionRequested()) break;

        ICameraDevice *cam = m_camera;
        m_mutex.unlock();

        m_running = true;

        while (m_running.load()) {
            {
                QMutexLocker locker(&m_mutex);
                if (!m_shouldRun) break;
                cam = m_camera;
            }

            if (!cam || !cam->isOpen()) {
                msleep(20);
                continue;
            }

            auto frame = cam->grabFrame(100 /*ms*/);
            if (frame.has_value())
                emit frameReady(frame.value());
        }

        m_running = false;
        m_mutex.lock();
    }

    m_mutex.unlock();
}
