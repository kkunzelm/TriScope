#pragma once

#include <QWidget>
#include <QList>
#include "interfaces/ICameraDevice.h"
#include "interfaces/IPositioningStage.h"
#include "ui/MeasurementOverlay.h"

class QComboBox;
class QDoubleSpinBox;
class QSlider;
class QLabel;
class QPushButton;
class QCheckBox;
class QRadioButton;
class QGroupBox;

// Sidebar panel matching the ids_peak_cockpit layout style.
// Sections: Camera | Stage | Overlays | Measurement.
// Owned pointers to camera and stage are NOT held here;
// this widget only emits control signals.
class SidebarWidget : public QWidget
{
    Q_OBJECT
public:
    explicit SidebarWidget(QWidget *parent = nullptr);

    // Panels for placement into the application's main tab widget.
    QWidget* connectPanel()  const { return m_connectPanel; }
    QWidget* evaluatePanel() const { return m_evaluatePanel; }

    // Populate the camera combo-box from discovery results.
    void setCameraList(const QList<CameraInfo> &cameras);

    // Reflect the actual camera controls after opening.
    void updateCameraControls(const CameraControls &ctrl,
                              double currentExposure, double currentGain);

    // Reflect the stage position in the display.
    void updatePosition(double x, double y, double z);

    // Populate serial-port combo-box.
    void refreshPortList();

    // Return the currently selected jog step (mm).
    double jogStep() const;

signals:
    // Camera
    void cameraRefreshRequested();
    void cameraSelected(const QString &id);
    void cameraStartStop(bool start);
    void exposureChanged(double microseconds);
    void gainChanged(double gain);

    // Stage
    void stageConnectRequested(const QString &port, const QString &type);
    void stageDisconnectRequested();
    void jogRequested(double dx, double dy, double dz);   // mm
    void moveAbsoluteRequested(double x, double y, double z);
    void calibrateRequested();
    void setHomeRequested();
    void measureLengthRequested();
    void abortRequested();

    // Overlays
    void crosshairToggled(bool visible);
    void gridToggled(bool visible);

    // Measurement
    void measurementToolSelected(MeasurementOverlay::Mode mode);
    void measurementCleared();
    void calibrationSet(double pixels, double um);

    // Unit
    void unitToggled(bool micrometers); // true = µm, false = mm

private:
    void buildCameraSection(QGroupBox *gb);
    void buildStageConnectSection(QGroupBox *gb);
    void buildStageControlSection(QGroupBox *gb);
    void buildOverlaySection(QGroupBox *gb);
    void buildMeasureSection(QGroupBox *gb);

    QString formatPosition(double mm) const;

    // Camera
    QComboBox       *m_cameraCombo   = nullptr;
    QDoubleSpinBox  *m_exposureSpin  = nullptr;
    QSlider         *m_exposureSlider = nullptr;
    QDoubleSpinBox  *m_gainSpin      = nullptr;
    QSlider         *m_gainSlider    = nullptr;
    QPushButton     *m_streamBtn     = nullptr;

    // Stage
    QComboBox       *m_portCombo     = nullptr;
    QComboBox       *m_stageTypeCombo = nullptr;
    QPushButton     *m_connectBtn    = nullptr;
    QLabel          *m_posLabelX     = nullptr;
    QLabel          *m_posLabelY     = nullptr;
    QLabel          *m_posLabelZ     = nullptr;
    QComboBox       *m_stepCombo     = nullptr;

    // Units
    QRadioButton    *m_unitMm        = nullptr;
    QRadioButton    *m_unitUm        = nullptr;
    bool             m_showUm        = false;

    // Absolute move spinboxes
    QDoubleSpinBox  *m_gotoX         = nullptr;
    QDoubleSpinBox  *m_gotoY         = nullptr;
    QDoubleSpinBox  *m_gotoZ         = nullptr;
    bool             m_gotoEdited    = false;  // true while user has unsent edits

    // Table measurement (relative XY)
    QPushButton     *m_setOriginBtn  = nullptr;
    QLabel          *m_relLabelX     = nullptr;
    QLabel          *m_relLabelY     = nullptr;
    double           m_absX          = 0.0;
    double           m_absY          = 0.0;
    double           m_originX       = 0.0;
    double           m_originY       = 0.0;
    bool             m_originSet     = false;

    // Measurement calibration
    QDoubleSpinBox  *m_calPixSpin    = nullptr;
    QDoubleSpinBox  *m_calUmSpin     = nullptr;

    // Top-level panels (owned here, parented to main tabs by MainWindow)
    QWidget         *m_connectPanel  = nullptr;
    QWidget         *m_evaluatePanel = nullptr;
};
