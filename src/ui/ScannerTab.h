#pragma once

#include "scanner/processing/Triangulator.h"

#include <QWidget>
#include <QVector>
#include <QPointF>
#include <utility>
#include <vector>

class ICameraDevice;
class IPositioningStage;
class AcquisitionThread;
class QComboBox;
class QDoubleSpinBox;
class QSpinBox;
class QPushButton;
class QProgressBar;
class QLabel;
class QGroupBox;

// Scanner tab: signal-driven stop-and-go laser triangulation scan.
// Uses the application's existing ICameraDevice and IPositioningStage directly.
class ScannerTab : public QWidget
{
    Q_OBJECT
public:
    explicit ScannerTab(QWidget *parent = nullptr);

    // Panels for placement into the application's main tab widget.
    QWidget* scannerPanel()  const { return m_scannerPanel; }
    QWidget* calibratePanel() const { return m_calibratePanel; }

    // Called by MainWindow whenever the active camera or stage changes.
    void setCamera(ICameraDevice *camera, AcquisitionThread *acqThread);
    void setStage(IPositioningStage *stage);

    // Live preview: called by MainWindow for every frame from AcquisitionThread.
    void onFrameReady(const QImage &img);

signals:
    // Emitted at scan start/end so MainWindow can disable the sidebar.
    void scanActiveChanged(bool active);

    // Emitted after each scan step frame grab so MainWindow can forward it to
    // CameraView — lets the user see what the scanner captured at each position.
    void previewFrameReady(const QImage &img);

    // Emitted alongside previewFrameReady with the extracted laser line positions
    // in raw camera image-pixel coordinates (col, row).
    // gauss = Gaussian interpolation, cog = Center of Gravity.
    void laserOverlayReady(QVector<QPointF> gauss, QVector<QPointF> cog);

private slots:
    void onStartOrAbort();
    void onBrowseOutput();
    void onSetStartX();
    void onSetEndX();
    void onStepReady(const QString &status);
    void onLoadCalib();
    void onSaveCalib();
    void onPositionChanged(double x, double y, double z);
    void onCalibrateZ();
    void onCalibrateY();
    void onShowHistogram();

private:
    void buildUI();
    void applyCalibJson(const QJsonObject &obj);
    void startScan();
    void abortScan();
    void finishScan();
    void finishCalibZ(bool success);

    scanner::CalibParams currentCalib() const;

    // Helpers for calibration
    static double meanValidCol(const scanner::LaserProfile &profile);
    static std::pair<double,double> detectEdges(const scanner::LaserProfile &profile);

    // ── UI ──────────────────────────────────────────────────────────────────
    QGroupBox      *m_stageGroup  = nullptr;
    QLabel         *m_posLabelX   = nullptr;
    QLabel         *m_posLabelY   = nullptr;
    QLabel         *m_posLabelZ   = nullptr;
    QComboBox      *m_jogStepCombo = nullptr;
    QDoubleSpinBox *m_gotoX       = nullptr;
    QDoubleSpinBox *m_gotoY       = nullptr;
    QDoubleSpinBox *m_gotoZ       = nullptr;
    bool            m_gotoEdited  = false;
    QDoubleSpinBox *m_startXSpin  = nullptr;
    QDoubleSpinBox *m_endXSpin    = nullptr;
    QDoubleSpinBox *m_stepSpin    = nullptr;
    QSpinBox       *m_threshSpin    = nullptr;
    QDoubleSpinBox *m_expSpin       = nullptr;
    QDoubleSpinBox *m_scatterSpin   = nullptr;
    QLabel         *m_outputLabel       = nullptr;
    QComboBox      *m_exportFormatCombo = nullptr;
    QDoubleSpinBox *m_xRefSpin    = nullptr;
    QDoubleSpinBox *m_scaleZSpin  = nullptr;
    QDoubleSpinBox *m_scaleYSpin  = nullptr;
    QDoubleSpinBox *m_cySpin      = nullptr;
    QProgressBar   *m_progress          = nullptr;
    QLabel         *m_statusLabel       = nullptr;   // scan status (Scanner tab)
    QLabel         *m_calibStatusLabel  = nullptr;   // calib status (Calibrate tab)
    QPushButton    *m_startBtn          = nullptr;
    QLabel         *m_preview           = nullptr;

    // Top-level panels (owned here, parented to main tabs by MainWindow)
    QWidget        *m_scannerPanel      = nullptr;
    QWidget        *m_calibratePanel    = nullptr;

    // ── Hardware (non-owning) ────────────────────────────────────────────────
    ICameraDevice     *m_camera    = nullptr;
    AcquisitionThread *m_acqThread = nullptr;
    IPositioningStage *m_stage     = nullptr;

    enum class ExportFormat { PlyBinary = 0, PlyAscii = 1, Viff = 2 };

    // ── Calibration state ────────────────────────────────────────────────────
    enum class CalibMode { None, CalibZ };
    CalibMode m_calibMode          = CalibMode::None;
    std::vector<scanner::ZCalibPoint> m_calibZPoints;
    double    m_calibZDeltaMm      = 1.0;
    int       m_calibZTotalSteps   = 5;
    int       m_calibZCurrentStep  = 0;
    double    m_calibZStartZ       = 0.0;

    // ── Scan state ───────────────────────────────────────────────────────────
    bool                m_scanning    = false;
    int                 m_currentStep = 0;
    std::vector<double> m_xPositions;
    scanner::PointCloud m_cloud;
    double              m_scanY       = 0.0;
    double              m_scanZ       = 0.0;
};
