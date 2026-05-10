#pragma once

#include <QMainWindow>
#include <memory>

class CameraView;
class SidebarWidget;
class ICameraDevice;
class IPositioningStage;
class CameraDiscovery;
class AcquisitionThread;
class QThread;

// Top-level application window.
// Owns the camera, stage, acquisition thread, and UI panels.
// Wires together all signals/slots between subsystems.
class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    // Camera
    void onCameraRefresh();
    void onCameraSelected(const QString &id);
    void onCameraStartStop(bool start);
    void onExposureChanged(double us);
    void onGainChanged(double gain);
    void onAcquisitionError(const QString &msg);

    // Stage
    void onStageConnect(const QString &port, const QString &type);
    void onStageDisconnect();
    void onJog(double dx, double dy, double dz);
    void onMoveAbsolute(double x, double y, double z);
    void onCalibrate();
    void onMeasureLength();
    void onAbort();
    void onPositionChanged(double x, double y, double z);
    void onMovementFinished(const QString &status);
    void onStageError(const QString &msg);

private:
    void buildMenu();
    void connectStageSignals(IPositioningStage *stage);
    void disconnectStage();

    // UI
    CameraView    *m_cameraView = nullptr;
    SidebarWidget *m_sidebar    = nullptr;

    // Camera subsystem
    CameraDiscovery                  *m_discovery   = nullptr;
    std::unique_ptr<ICameraDevice>    m_camera;
    AcquisitionThread                *m_acqThread   = nullptr;

    // Stage subsystem (lives in its own thread)
    QThread                          *m_stageThread = nullptr;
    std::unique_ptr<IPositioningStage> m_stage;
};
